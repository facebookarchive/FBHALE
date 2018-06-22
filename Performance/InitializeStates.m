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

function [aircraft, freeStream] = InitializeStates(mission, aircraft, freeStream) 
% Initialize aircraft states based on provided inputs.
% Aircraft states are captured in aircraft.attitude and aircraft.position

% Position
aircraft.states.h_m = mission.h_m;

% Add altitude as aircraft states for integration if not
% already in the list
if isempty( find(contains({aircraft.states.integration.var}, 'h_m'), 1) )
    aircraft.states.integration(end+1).var  = 'aircraft.states.h_m';
    aircraft.states.integration(end).der    = 'aircraft.states.hdot_ms';
    aircraft.states.integration(end).bounds = [mission.minh_m, mission.maxh_m];
end

% Attitude
if ~isfield(mission, 'groundTrackPath') % If the type of ground track hasn't been defined, set a dummy ground track angle
    mission.groundTrackPath = 'constantTrackAngle';
    mission.groundTrackAngle_rad = 0;
end

% Ensures inputs are correctly input then call aircraft kinematics
switch mission.groundTrackPath
    case 'orbit'
        
        % Initialize position on the trajectory
        aircraft.states.s_m       = mission.s_m;
        aircraft.states.groundTrackAngle_rad = interp1(mission.trajectory.s_m, mission.trajectory.groundTrackAngle_rad, aircraft.states.s_m);
        
        % Add trajectory position as aircraft states for integration if not
        % already in the list
        if isempty( find(contains({aircraft.states.integration.var}, 's_m')) )
            aircraft.states.integration(end+1).var = 'aircraft.states.s_m';
            aircraft.states.integration(end).der = 'aircraft.states.groundSpeed_ms';
        end
        
    	% Position
        aircraft.states.xGround_m = interp1(mission.trajectory.s_m, mission.trajectory.x_m, aircraft.states.s_m);
        aircraft.states.yGround_m = interp1(mission.trajectory.s_m, mission.trajectory.y_m, aircraft.states.s_m);
        
        % First estimate for VTAS
        [freeStream.rho_kgm3, freeStream.dynamicViscosity_Nsm2] = GetAtmosphereProperties(aircraft.states.h_m);
        aircraft.aero.mode = 'minPower';
        aircraft.aero = Polar(aircraft.aero, freeStream, aircraft, mission);
        freeStream.VTAS_ms = sqrt( 2 * aircraft.massProperties.MGTOW_N / (aircraft.aero.Sref_m2*aircraft.aero.CLcruise*freeStream.rho_kgm3));
        if isfield(freeStream, 'windSpeed_ms')
            if freeStream.VTAS_ms < freeStream.windSpeed_ms
                freeStream.VTAS_ms = freeStream.windSpeed_ms;
            end
        end
        
        % First estimate for Gamma
        if isfield(aircraft.states, 'hdot_ms')
            freeStream.gamma_rad = asin(aircraft.states.hdot_ms / freeStream.VTAS_ms);
        end
            
    case 'sunPointing'
        
        % Assume aircraft is stationnary
        aircraft.states.xGround_m = mission.xGround_m;
        aircraft.states.yGround_m = mission.yGround_m;
        
        % No wind assumption for initialization
        aircraft.states.roll_rad  = 0;
        
        % Get sun position and direction and set aircraft attitude
        [location, curTimeStruct] = GetTimeLocation( mission );
        sunangle                  = sun_position(curTimeStruct, location);
        aircraft.states.yaw_rad = sunangle.azimuth*pi/180;
        aircraft.states.groundtrackAngle_rad = aircraft.attitude.yaw_rad;
        
    case 'constantTrackAngle'
        
        aircraft.states.groundTrackAngle_rad = mission.groundTrackAngle_rad;
        
        % No wind assumption for initialization
        aircraft.states.roll_rad = 0;
        
        % Add x-y ground position as aircraft states for integration if not
        % already in the list
        if isempty( find(contains({aircraft.states.integration.var}, 'xGround_m'), 1) )
            aircraft.states.integration(end+1).var  = 'aircraft.states.xGround_m';
            aircraft.states.integration(end).der    = 'aircraft.states.groundSpeed_ms*cos(aircraft.states.groundTrackAngle_rad)';
            aircraft.states.integration(end+1).var  = 'aircraft.states.yGround_m';
            aircraft.states.integration(end).der    = 'aircraft.states.groundSpeed_ms*sin(aircraft.states.groundTrackAngle_rad)';
        end
        
        % Position
        aircraft.states.xGround_m = mission.xGround_m;
        aircraft.states.yGround_m = mission.yGround_m;
        
end

[aircraft, freeStream] = FlightKinematics(mission, aircraft, freeStream);

end
