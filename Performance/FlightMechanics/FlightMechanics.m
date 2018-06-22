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
  
function [aircraft, freeStream, mission] = FlightMechanics(aircraft, freeStream,mission)
% This function trims the aircraft based on the specified degrees of
% freedom, aero and propulsion models.
% The equations are specified in the wind axis under the form:
%       F_propulsion = F_aero + Weight
% The system is solved using variables coherently with the choice of model.
% For instance, in a 2DOF system, 
%       F_propulsion = [Thrust * cos(installationAngle); Thrust * sin(installationAngle)];
%       F_aero       = [Drag; Lift];
%       Weight       = [W_N * sin(climbAngle); W_N * cos(climbAngle)];

% Bullet proofing
if ~isfield(aircraft.aero, 'DOF')
    error('No DOF specified in the aero structure');
end

% Keep track of aircraft trim variables to avoid solving from scratch
persistent X

% Finding freeStream speed based on
if strcmp(mission.leg.mode,'minV')
    freeStream.V_ms = freeStream.Vx_ms/cos(freeStream.gamma_rad);
end

switch aircraft.aero.DOF
    case 2
        [F, Faero, Fpropulsion, Fweight, X] = FlightMechanics2DOF(aircraft, freeStream, mission, X);
end

% Solve
% Check if system isn't already solved
if F(X.initial) < 1e-4
    aircraft.trimmed = true;
else
    [X.values, F1] = fminsearch(F, X.initial);
    aircraft.trimmed = true;
    if F1 > 1e-4
        aircraft.trimmed = false;
        
        % Try with a fresher initial guess
        X.tags = {};
        [F, Faero, Fpropulsion, Fweight, X] = FlightMechanics2DOF(aircraft, freeStream, mission, X);
        [X.values, F1] = fminsearch(F, X.initial);
      	if F1 > 1e-4
            aircraft.trimmed = false;
            warning('Aircraft not trimmed even with a reset initial guess');
        end
        
    else
        X.initial = X.values;
    end
end

% Assign
for t = 1:length(X.tags)
    eval([X.tags{t} '= ' num2str(X.values(t)) ';']);
end

%Adjust heading, yaw and alpha for solar models
%Bullet Proofing
if ~isfield(mission, 'pointing')
    warning('No heading mode specified 0 used for yaw and roll, freeStream_gamma used for pitch')
    mission.attitude.roll  = 0;
    mission.attitude.pitch = freeStream.gamma_rad;
    mission.attitude.yaw   = 0;
else
    if strcmp(mission.pointing, 'sunPointing')
       mission.attitude.roll  = 0;
       mission.attitude.pitch = freeStream.gamma_rad;
       
       %get sun position
       [ location, curTimeStruct ] = getTimeLocation( mission );
       sunangle                    = sun_position(curTimeStruct, location); 
       mission.attitude.yaw        = sunangle.azimuth ;
       
    elseif strcmp(mission.pointing, 'orbit')
       if ~isfield(mission, 'radius_m')
           error('No orbit radius specified')
       end
       
       mission.attitude.roll  = atan(freeStream.V_ms^2/(mission.radius_m*mission.g_ms2));
       mission.attitude.pitch = freeStream.gamma_rad;
       if ~isfield(mission.attitude, 'yaw')
           mission.attitude.yaw = 0;
       end
       mission.attitude.yaw   = mission.attitude.yaw + freeStream.V_ms^2/mission.radius_m/mission.g_ms2;
    
    end
    
end

end

