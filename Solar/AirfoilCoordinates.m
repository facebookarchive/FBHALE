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
  
function airfoil = AirfoilCoordinates(optim, yobo2, entity)
% Gets suction side coordinated at specified location.

for i = 2:length(optim.(entity).yobo2)
    if yobo2 < optim.(entity).yobo2(i)
        break
    end
end

% find twist and chord based on yloc
twist = optim.(entity).twists_deg(i-1) + (optim.(entity).twists_deg(i)-optim.(entity).twists_deg(i-1))...
    * (yobo2-optim.(entity).yobo2(i-1))/(optim.(entity).yobo2(i)-optim.(entity).yobo2(i-1));
chord = optim.(entity).c_m(i-1) + (optim.(entity).c_m(i)-optim.(entity).c_m(i-1))...
    * (yobo2-optim.(entity).yobo2(i-1))/(optim.(entity).yobo2(i)-optim.(entity).yobo2(i-1)); 
airfoil.chord = chord;
airfoil.twist = twist;

% set curved point location
if iscell(optim.(entity).aero.airfoilADB.geom(1).xySuction)
  xy = optim.(entity).aero.airfoilADB.geom(1).xySuction(1);
  xy = xy{1};
else
  xy = optim.(entity).aero.airfoilADB.geom(1).xySuction;
end

airfoil.X = xy(:,1)';
airfoil.Z = xy(:,2)';

% Align airfoil 1/4 chords
airfoil.X   = (airfoil.X - 1/4)*-1;
% Rotate airfoils based on wing twist 
airfoil.X_rot   = airfoil.X* cosd(twist) - airfoil.Z * sind(twist);
airfoil.Z_rot   = airfoil.Z* cosd(twist) + airfoil.X * sind(twist);
% Scale airfoils based on local chord
airfoil.X   = airfoil.X_rot .*  chord;
airfoil.Z   = airfoil.Z_rot .*  chord;

if strcmp(entity, 'wing')
    if isfield(optim.wing.aero, 'ASWINGpolar')
        if isfield(optim.wing.aero.ASWINGpolar, 'deltaz_m')
   
            [t, I] = unique(optim.wing.aero.ASWINGpolar.t{1});
            z_m    = optim.wing.aero.ASWINGpolar.deltaz_m{1}(:, I);
            
            %this logic assumes only one wing
            zroot = interp2f(t, optim.wing.aero.ASWINGpolar.AoA_deg, z_m, 0, optim.aircraft.aero.alphaCruise_deg);
            
            % Compute t-values at geometric break-point
            tGeom = interp1f(optim.wing.aero.sectionPolars.yobo2, optim.wing.aero.sectionPolars.t, yobo2);
            
            % Compute z-values. Bullet-proof against ill-converged ASWING
            % results
            [~, closestIndex] = min(abs(optim.aircraft.aero.polar.AoA_deg-optim.aircraft.aero.alphaCruise_deg)); 
            closestIndex = min(closestIndex, length(optim.aircraft.aero.polar.AoA_deg)-1);
            closestIndex = max(closestIndex, 2);
            zdeflect = interp2(t, optim.aircraft.aero.ASWINGpolar.AoA_deg(closestIndex-1:closestIndex+1), z_m(closestIndex-1:closestIndex+1, :), ...
                               tGeom, optim.aircraft.aero.alphaCruise_deg, 'spline');
            zoffset = (zdeflect - zroot).*ones(1, length(airfoil.Z));
            airfoil.Z = airfoil.Z + zoffset;
        end
    end
end

% Set y location of airfoil points
if strcmp(entity, 'wing')
    airfoil.Y   = yobo2*optim.(entity).bref_m/2  * ones(length(airfoil.X),1)';
else
    airfoil.Y   = yobo2*optim.(entity).Sref_m2*optim.(entity).AR/2* ones(length(airfoil.X),1)';

end
