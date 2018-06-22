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
  
function optim = PopulateSectionPolars(optim, surface, varargin)
% The purpose of this function is to populate airfoil properties vs 
% deflection, Re, alpha for the aero specified locations + CS locations. 
% The aero properties of interest are cl, cd, cm, but also cd0, clMax, 
% deltaMax, the maximum control surface deflection at a given alpha, 
% cldeltaMax, the corresponding cl.
% The resulting data is used to compute actual lift and drag (see
% LiftAndProfileDragCorrection.m).

% Span locations where aero data needs to be specified: sections where
% airfoils are provided, control surfaces, and surface break points
if isfield(optim.(surface), 'controlSurface')
    controlSurfaceBreakPoints = [optim.(surface).controlSurface(:).yobo2];
    Nc = length(optim.(surface).controlSurface);
else
    controlSurfaceBreakPoints = 0;
    Nc = 0;
end
yobo2s      = [optim.(surface).yobo2 controlSurfaceBreakPoints optim.(surface).yAirfoilsobo2];
[yobo2s, ~]	= sort(yobo2s);
yobo2s      = Mirror(yobo2s, -1);

% Airfoil and geometry span locations
yAirfoilsobo2 = Mirror(optim.(surface).yAirfoilsobo2, -1);
yGeomobo2     = Mirror(optim.(surface).yobo2, -1);

% Remove doubles
[yobo2s, ~] = unique(yobo2s);

% If only one thickness is provided for the airfoil db, then reproduce it
% to make sure that interpolations don't error out
 oneTOCwasAdded = false;
if length(optim.(surface).aero.airfoilADB.dim1.vector) == 1
    oneTOCwasAdded = true;
    optim.(surface).aero.airfoilADB.dim1.vector(end+1) = optim.(surface).aero.airfoilADB.dim1.vector(end)*1.1;
    fieldss = fields(optim.(surface).aero.airfoilADB);
    numberOfTables = sum(cell2mat(strfind(fieldss, 'table')));
    for i=1:numberOfTables
        optim.(surface).aero.airfoilADB.(['table' num2str(i)]).values(2, :, :, :, :) = optim.(surface).aero.airfoilADB.(['table' num2str(i)]).values(1, :, :, :, :);
    end
end

% Select the polars that are relative only to the specified relative
% control surface chord
for cs = 1:Nc
	csIndex(cs) = find(optim.(surface).aero.airfoilADB.dim2.vector == [optim.(surface).controlSurface(cs).csoc]);
end
if Nc == 0
    csIndex = find(optim.(surface).aero.airfoilADB.dim2.vector == 1);
end
csIndex = unique(csIndex);

% Retrieve data
polar.cl = (optim.(surface).aero.airfoilADB.table1.values(:, csIndex, :, :, :));
polar.cd = (optim.(surface).aero.airfoilADB.table2.values(:, csIndex, :, :, :));
polar.cdp = (optim.(surface).aero.airfoilADB.table4.values(:, csIndex, :, :, :));
polar.cm = (optim.(surface).aero.airfoilADB.table3.values(:, csIndex, :, :, :));

% Squeeze polar data
polar.cl = squeeze(polar.cl);
polar.cd = squeeze(polar.cd);
polar.cdp = squeeze(polar.cdp);
polar.cm = squeeze(polar.cm);

% Generate list of polars along span locations where the airfoils where
% specified
% if an extra argument was passed, that means that only a few of these
% quantities is actually needed and therefore replace 'polarQtities by the
% provided vector'
polarQtities = {'cl', 'cd', 'cdp', 'cm'};

if nargin > 2
    polarQtities = varargin{1};
end

for q = 1:length(polarQtities)
    
    siz= size(polar.(polarQtities{q}));
    polar.(polarQtities{q}) = ScaleTimeBulletProof(polar.(polarQtities{q}), optim.(surface).aero.airfoilADB.dim1.vector, optim.(surface).toc);
    polar.(polarQtities{q}) = reshape(polar.(polarQtities{q}), [length( optim.(surface).toc) siz(2:end)]);
        
    % Interpolate polars from the specified sections to the non-dimentional 
    % span locations of interest
    siz  = size(polar.cl);
  	polar.(polarQtities{q}) = ScaleTimeBulletProof(cat(1, polar.(polarQtities{q})(end:-1:2, :, :, :), polar.(polarQtities{q})), yAirfoilsobo2, yobo2s);
    polar.(polarQtities{q}) = reshape(polar.(polarQtities{q}), [length(yobo2s) siz(2:end)]);
end

% Deflection vector / unit deflection for the right side
for cs = 1:Nc
    polar.FlagRightDeflection{cs} = yobo2s >= optim.(surface).controlSurface(cs).yobo2(1) & ...
                    yobo2s <= optim.(surface).controlSurface(cs).yobo2(2);
    polar.FlagLeftDeflection{cs} =  yobo2s <= -optim.(surface).controlSurface(cs).yobo2(1) & ...
                    yobo2s >= -optim.(surface).controlSurface(cs).yobo2(2);
end

% It's also nice to provide chord and twist at that location
polar.c_m        = interp1f(yGeomobo2, Mirror(optim.(surface).c_m, 1),        yobo2s);
polar.twists_deg = interp1f(yGeomobo2, Mirror(optim.(surface).twists_deg, 1), yobo2s);
polar.sweep_deg  = -optim.(surface).sweep_deg * sign(yobo2s);
polar.dihedral_deg = 0 * ones(1, length(polar.twists_deg));
polar.toc        = interp1f(yAirfoilsobo2,  Mirror(optim.(surface).toc, 1), yobo2s);

% Store for further use
polar.delta_deg = optim.(surface).aero.airfoilADB.dim3.vector;
polar.Re        = optim.(surface).aero.airfoilADB.dim4.vector;
polar.AoA_deg	= optim.(surface).aero.airfoilADB.dim5.vector;
polar.c_m       = interp1f(yGeomobo2, [optim.(surface).c_m(end:-1:2) optim.(surface).c_m], yobo2s);
polar.yobo2     = yobo2s;
polar.y_m       = yobo2s * optim.(surface).bref_m/2;
polar.x_m       = optim.(surface).x_m  - tand(polar.sweep_deg).*polar.y_m;
polar.z_m       = optim.(surface).z_m * ones(1, length(polar.y_m));

% Extrude winglet if specified
if isfield(optim.(surface), 'winglet')
    l_m = optim.(surface).winglet.zoc * polar.c_m(1);
    
    if l_m > 0
        polar.x_m = [polar.x_m(1) polar.x_m(1) polar.x_m polar.x_m(end) polar.x_m(end)];
        polar.y_m = [polar.y_m(1) polar.y_m(1) polar.y_m polar.y_m(end) polar.y_m(end)];
        polar.z_m = [polar.z_m(1)-l_m polar.z_m(1)-l_m/10 polar.z_m polar.z_m(end)-l_m/10 polar.z_m(end)-l_m];
        polar.c_m = [polar.c_m(1) polar.c_m(1) polar.c_m polar.c_m(end) polar.c_m(end)];
        polar.toc = [polar.toc(1) polar.toc(1) polar.toc polar.toc(end) polar.toc(end)];
        
        yobo2s          = [-1 -1 yobo2s 1 1];
        polar.yobo2     = yobo2s;

        polar.dihedral_deg  = [-90 -90 polar.dihedral_deg -90 -90];
        polar.sweep_deg     = [0 0 polar.sweep_deg 0 0];
        polar.twists_deg    = [0 0 polar.twists_deg 0 0];

        % Add sectional polars
        for q = 1:length(polarQtities)
            polar.(polarQtities{q}) = cat(1, polar.(polarQtities{q})(1, :, :, :), polar.(polarQtities{q}));
            polar.(polarQtities{q}) = cat(1, polar.(polarQtities{q})(1, :, :, :), polar.(polarQtities{q}));
            polar.(polarQtities{q}) = cat(1, polar.(polarQtities{q}), polar.(polarQtities{q})(end, :, :, :));
            polar.(polarQtities{q}) = cat(1, polar.(polarQtities{q}), polar.(polarQtities{q})(end, :, :, :));
        end
        
        %% Add an element to the span cs-deflection flag vector:
        for cs = 1:Nc
            polar.FlagRightDeflection{cs} = [0 0 polar.FlagRightDeflection{cs} 0 0];
            polar.FlagLeftDeflection{cs}  = [0 0 polar.FlagLeftDeflection{cs} 0 0];
        end
    end
end

% Local beam c-s-n system based on undeformed shape (sweep, twist)
for s = 1:length(yobo2s)
    Rtwist = [  cosd(polar.twists_deg(s)) 	0   -sind(polar.twists_deg(s));
                0                           1   0;
                sind(polar.twists_deg(s))	0   cosd(polar.twists_deg(s))];
    
    Ryaw = [cosd(polar.sweep_deg(s)) 	sind(polar.sweep_deg(s))	0;
            -sind(polar.sweep_deg(s)) 	cosd(polar.sweep_deg(s))	0;
            0                           0                           1];
    
    Rdihedral = [1	0                               0;
                0	cosd(polar.dihedral_deg(s))     sind(polar.dihedral_deg(s));
                0 	-sind(polar.dihedral_deg(s))	cosd(polar.dihedral_deg(s))];
    
    % Tensor matrix
    T = Rtwist * Ryaw * Rdihedral;
    
    % Local coordinate system
    ec(:, s) = T(1, :)';
    es(:, s) = T(2, :)';
    en(:, s) = T(3, :)';
    
    % Local quarter chord location
    x0_m(s)     = optim.(surface).structure.Xax{1}(1)*polar.c_m(s) - polar.c_m(s)/4;
    rQC_m(:, s) = [polar.x_m(s); polar.y_m(s); polar.z_m(s)] - x0_m(s)*ec(:, s);
end

polar.xQC_m = rQC_m(1, :);
polar.yQC_m = rQC_m(2, :);
polar.zQC_m = rQC_m(3, :);

t           = [0 cumsum(sqrt( diff(polar.x_m).^2 + diff(polar.y_m).^2 + diff(polar.z_m).^2))];
tPlannar    = [0 cumsum(sqrt( diff(polar.x_m).^2 + diff(polar.y_m).^2))];
t           = t - max(t)/2;
tPlannar    = tPlannar - max(tPlannar)/2;
polar.t     = t/max(abs(tPlannar));
tAirfoils   = unique([polar.t(1) yAirfoilsobo2 polar.t(end)]);
polar.tAirfoils = tAirfoils;
tQC         = [0 cumsum(sqrt( diff(polar.xQC_m).^2 + diff(polar.yQC_m).^2 + diff(polar.zQC_m).^2 ))];
tQCPlannar  = [0 cumsum(sqrt( diff(polar.xQC_m).^2 + diff(polar.yQC_m).^2))];
tQC         = tQC - max(tQC)/2;
tQCPlannar  = tQCPlannar - max(tQCPlannar)/2;
polar.tQC   = tQC/max(abs(tQCPlannar));

optim.(surface).aero.sectionPolars = polar;

% Remove the extra toc
if oneTOCwasAdded
    optim.(surface).aero.airfoilADB.dim1.vector(end) = [];
end

end
