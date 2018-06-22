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
  
function optim = LongitudinalTrim(optim)
% This function computes the aircraft pitching moment at its CG under load
% based on the contribution of lifting surfaces, and axisymmetric bodies.
% PitchTrim.m will leverage a variable e.g. wing position, to drive CM
% to zero.

% Potential longitudinal surfaces
longSurfaces = {'wing', 'htail'};
surfaces     = {};
for l = 1:length(longSurfaces)
    if max(contains(optim.aircraft.beamNames, longSurfaces{l}))
        surfaces{end+1} = longSurfaces{l};
    end
end    

% Wind to body axis rotation
alphaCruise_deg = optim.aircraft.aero.alphaCruise_deg;
R  = [cosd(alphaCruise_deg)	0 -sind(alphaCruise_deg); 
     0                      1 0; 
     sind(alphaCruise_deg)  0 cosd(alphaCruise_deg)];

% CG position
rCG_m = [optim.xCG_m; 0; optim.zCGDef_m];

% Initialize
CM       = zeros(1, length(surfaces));
CMl      = zeros(1, length(surfaces));
CMS      = zeros(1, length(surfaces));

for s = 1:length(surfaces)
    
    % Populate sectional properties if not already done
    if ~isfield(optim.(surfaces{s}).aero, 'sectionPolars')
        optim = PopulateSectionPolars(optim, (surfaces{s}), {'cl'});
    end

    for i = 1:length(optim.(surfaces{s}).N)
        
        % Stability axis / Body axis force coefficients. x aft, z up
        CFSA = [optim.(surfaces{s}).aero.CDcruise; 0; optim.(surfaces{s}).aero.CLcruise];
        CFBA = R * CFSA;

        % Moment arms
        r_m  = [optim.(surfaces{s}).x_m; 0; optim.(surfaces{s}).z_m];

        % Pitching moment coefficient for the considered surface
        CM(s)  = CM(s)  + (optim.(surfaces{s}).aero.CMcruise + dot(cross(r_m - rCG_m, CFBA), [0 1 0]) / optim.(surfaces{s}).c_m(1)).*optim.(surfaces{s}).N(i);
        CMl(s) = CMl(s) + (dot(cross([1;0;0], CFBA), [0 1 0]) / optim.(surfaces{s}).c_m(1)).*optim.(surfaces{s}).N(i);
        CMS(s) = CMS(s) + (dot(cross(r_m - rCG_m, CFBA), [0 1 0]) / optim.(surfaces{s}).c_m(1)).*optim.(surfaces{s}).N(i);
    end   
    
    % Non-dimentionalization by aircraft reference quantities
    CM(s) = CM(s)  * optim.(surfaces{s}).c_m(1) * optim.(surfaces{s}).Sref_m2 / optim.aircraft.cref_m / optim.aircraft.Sref_m2;
    
    % Derivatives for Jacobian construction
    CMl(s)= CMl(s) * optim.(surfaces{s}).c_m(1) * optim.(surfaces{s}).Sref_m2 / optim.aircraft.cref_m / optim.aircraft.Sref_m2;
    CMS(s)= CMS(s) * optim.(surfaces{s}).c_m(1) / optim.aircraft.cref_m / optim.aircraft.Sref_m2;
end

% Boom/Pod pitching moment
% Assume slender body theory
%   Cmalpha = 2 * V / (S*c), with V the equivalent volume of the boom
if max(contains(optim.aircraft.beamNames, 'boom'))
    CMaboom     = 2 * optim.boom.V_m3 / (optim.aircraft.Sref_m2 * optim.aircraft.cref_m);
    CMboom      = CMaboom * optim.aircraft.aero.alphaCruise_deg * pi / 180 .* optim.boom.N;
else
    CMboom      = 0;
end

if max(contains(optim.aircraft.beamNames, 'pod'))
    CMapod      = 2 * optim.pod.V_m3 / (optim.aircraft.Sref_m2 * optim.aircraft.cref_m);
    CMpod       = CMapod * optim.aircraft.aero.alphaCruise_deg * pi / 180 .* optim.pod.N;
else
    CMpod       = 0;
end

% Aircraft Fz and pitching moment, Static margin
optim.aircraft.aero.CMtrim = sum(CM)  + sum(CMboom)  + sum(CMpod);

% Derivatives for Jacobian calculation
for s = 1:length(surfaces)
    optim.(surfaces{s}).aero.dCMdS = CMS(s);
    optim.(surfaces{s}).aero.dCMdl = CMl(s);
end

end
