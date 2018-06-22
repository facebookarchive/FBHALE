%{
For FBHALE software
Copyright (C) 2018-present Facebook, Inc.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
%}
  
function [ fn, Area, Xb, P, optim ] = GetPanelProperties(optim, entity )
% Get panel normal vectors and area. Function creates a
% tessellation of the upper surface of the entity passed in and outputs:
%   TR2     - a matlab tringualtion
%   Area    - vector with panel areas
%   fn      - vector of panel normal vectors
%   Xb      - the xyz locations of the panel corners
%   P       - the location of the center of each panel

%Find dimensions of solar modules
[ optim ] = GetSolarModuleDimensions( optim, entity );

% Find airfoil properties at each specified section
fn = []; Area = []; Xb = []; P = []; 

% non-dimensionalize solar panel length
sclen_obo2 = optim.(entity).solar.module.spanwiseLength_m/(optim.(entity).bref_m/2);
for i = 2:length(optim.(entity).yobo2)
    % find yobo2 location for next panel joint
    ypanel = optim.(entity).yobo2(i-1);
    while ypanel < optim.(entity).yobo2(i) - sclen_obo2
        ypanel = ypanel + sclen_obo2;
        
        % get airfoil chords for left right and mid points of the panel
        afleft  = AirfoilCoordinates(optim, ypanel-sclen_obo2, entity);
        afright = AirfoilCoordinates(optim, ypanel, entity);
        afmid   = AirfoilCoordinates(optim, ypanel-sclen_obo2/2, entity);
        
        % find middle top surface curve and gradient
        xmid = linspace(afmid.X(1), afmid.X(end), 5000);
        zmid = interp1f(afmid.X, afmid.Z, xmid);
        dx = gradient(xmid); dz = gradient(zmid);
        xmid = xmid(xmid<=afright.X(1));
        zmid = zmid(xmid<=afright.X(1));
        dx = dx(xmid<=afright.X(1)); 
        dz = dz(xmid<=afright.X(1));
        
        %find right hand side airfoil data points
        xright = linspace(afright.X(1), afright.X(end), length(xmid));
        zright = interp1f(afright.X, afright.Z, xright);
        
        % make curve length as a function of x
        s = [0, cumsum(sqrt(diff(xright).^2 + diff(zright).^2))];
        
        % put solar panels on wing
        spanel = s(1)+optim.(entity).solar.module.chordwiseLength_m/2:optim.(entity).solar.module.chordwiseLength_m:s(end)-optim.(entity).solar.module.chordwiseLength_m/2;
        
        % find normal vector (dx, dz) at panel center
        dxpanel = dx(1)*ones(1, length(spanel));
        dzpanel = interp1f(s, dz, spanel);
        fxz = [dzpanel' -dxpanel'];
        fnxz = fxz./sqrt(sum(abs(fxz).^2,2));

        % find mid x location at each panel
        xpanelmid = interp1f(s, xmid, spanel);
        
        % find left and right z locations of each panel
        zpanelleft = interp1f(afleft.X, afleft.Z, xpanelmid);
        zpanelright = interp1f(afright.X, afright.Z, xpanelmid);
        
        % use z panel point to find y component of normal vector
        fnyz = [-(zpanelright-zpanelleft)'./optim.(entity).solar.module.spanwiseLength_m, ones(size(zpanelleft))'].*fnxz(:,2);
        fnpanel = [fnxz(:,1), fnyz(:,1), fnxz(:,2)];
        fn = [fn; fnpanel];
        
        % Xb affects mppt placement so don't rotate if dihedral until structural 
        Xb = [Xb; xpanelmid' optim.(entity).yobo2(i-1).*optim.(entity).bref_m/2*ones(length(xpanelmid), 1) zpanelleft';...
                  xpanelmid' optim.(entity).yobo2(i).*optim.(entity).bref_m/2*ones(length(xpanelmid), 1) zpanelleft'];
        
        % compute beginning on solar panel xlocations and left/right z
        % locations based on surface length
        spanel2 = s(1):optim.(entity).solar.module.chordwiseLength_m:s(end);
        xpanelmidf   = interp1f(s, xmid, spanel2);
        zpanelleftf  = interp1f(afleft.X, afleft.Z, xpanelmidf);
        zpanelrightf = interp1f(afright.X, afright.Z, xpanelmidf);
        
        % compute 4 points of the panel
        % Pp = nPanels x nSpaceDim x nCornerPts 
        % nSpaceDim = 3 (xloc, yloc, zloc); nCornerPts = 4
         Pp = [];
         Pp(:,:,1) = [xpanelmidf(1:end-1)', (ypanel-sclen_obo2).*optim.(entity).bref_m/2.*ones(length(xpanelmid), 1), zpanelleftf(1:end-1)'];
         Pp(:,:,2) = [xpanelmidf(2:end)', (ypanel-sclen_obo2).*optim.(entity).bref_m/2.*ones(length(xpanelmid), 1), zpanelleftf(2:end)'];
         Pp(:,:,3) = [xpanelmidf(1:end-1)', ypanel.*optim.(entity).bref_m/2.*ones(length(xpanelmid), 1), zpanelrightf(1:end-1)'];
         Pp(:,:,4) = [xpanelmidf(2:end)', ypanel.*optim.(entity).bref_m/2.*ones(length(xpanelmid), 1), zpanelrightf(2:end)']; 
         if isfield(optim.wing, 'dihedral')
             % rotate corner points
             if ypanel > optim.(entity).yobo2(2)
             Pp(:,:,1) = (-[zeros(length(xpanelmid), 1), optim.(entity).yobo2(2).*optim.(entity).bref_m/2.*ones(length(xpanelmid), 1), zeros(length(xpanelmid), 1)] ...
                          +[xpanelmidf(1:end-1)', (ypanel-sclen_obo2).*optim.(entity).bref_m/2.*ones(length(xpanelmid), 1), zpanelleftf(1:end-1)'])*rotation ...
                          + [zeros(length(xpanelmid), 1), optim.(entity).yobo2(2).*optim.(entity).bref_m/2.*ones(length(xpanelmid), 1), zeros(length(xpanelmid), 1)];
             Pp(:,:,2) = (-[zeros(length(xpanelmid), 1), optim.(entity).yobo2(2).*optim.(entity).bref_m/2.*ones(length(xpanelmid), 1), zeros(length(xpanelmid), 1)] ...
                          +[xpanelmidf(2:end)', (ypanel-sclen_obo2).*optim.(entity).bref_m/2.*ones(length(xpanelmid), 1), zpanelleftf(2:end)'])*rotation ...
                          + [zeros(length(xpanelmid), 1), optim.(entity).yobo2(2).*optim.(entity).bref_m/2.*ones(length(xpanelmid), 1), zeros(length(xpanelmid), 1)];
             Pp(:,:,3) = (-[zeros(length(xpanelmid), 1), optim.(entity).yobo2(2).*optim.(entity).bref_m/2.*ones(length(xpanelmid), 1), zeros(length(xpanelmid), 1)] ...
                          +[xpanelmidf(1:end-1)', ypanel.*optim.(entity).bref_m/2.*ones(length(xpanelmid), 1), zpanelrightf(1:end-1)'])*rotation ...
                          + [zeros(length(xpanelmid), 1), optim.(entity).yobo2(2).*optim.(entity).bref_m/2.*ones(length(xpanelmid), 1), zeros(length(xpanelmid), 1)];
             Pp(:,:,4) = (-[zeros(length(xpanelmid), 1), optim.(entity).yobo2(2).*optim.(entity).bref_m/2.*ones(length(xpanelmid), 1), zeros(length(xpanelmid), 1)] ...
                          +[xpanelmidf(2:end)', ypanel.*optim.(entity).bref_m/2.*ones(length(xpanelmid), 1), zpanelrightf(2:end)'])*rotation ...
                          + [zeros(length(xpanelmid), 1), optim.(entity).yobo2(2).*optim.(entity).bref_m/2.*ones(length(xpanelmid), 1), zeros(length(xpanelmid), 1)];
             end
         end
        P = [P; Pp];

        % compute areas
        Area = [Area; ones(length(spanel), 1).*(optim.(entity).solar.module.spanwiseLength_m*optim.(entity).solar.module.chordwiseLength_m)];

    end

end

% if the surface is vertical rearrange points and normals
if strcmp(entity, 'vtail')
    fn = [fn(:,1) -fn(:,3), fn(:,2)];
    ytemp = P(:,2,:);
    P(:,2,:) = -P(:,3,:);
    P(:,3,:) = ytemp;
    P(:,1,:) = P(:,1,:) - optim.vtail.x_m;
end
if strcmp(entity, 'htail')
   P(:,1,:) = P(:,1,:) - optim.htail.x_m ;
end

% Repmat based on number of instances of surface
if optim.(entity).N > 1
   fn   = repmat(fn, sum(optim.(entity).N),1);
   Area = repmat(Area, sum(optim.(entity).N),1);
end

end
