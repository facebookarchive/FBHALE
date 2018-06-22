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
  
function [aircraft, freeStream] = FlightKinematics(mission, aircraft, freeStream)
% This function makes use of kinematic relationships to update some
% aircraft states such as roll, yaw, ground speed.

% Pitch angle = gamma + alpha
Re_num = freeStream.VTAS_ms * aircraft.aero.Cref_m*freeStream.rho_kgm3/freeStream.dynamicViscosity_Nsm2;
CLvsAoA = ScaleTimeBulletProof(aircraft.aero.polar.CL, aircraft.aero.polar.Re, Re_num);
if aircraft.aero.CL < min(CLvsAoA)
    aircraft.aero.CL = min(CLvsAoA);
end
alpha_rad = interp1q(CLvsAoA,  aircraft.aero.polar.alpha_rad, aircraft.aero.CL);
CDvsAoA   = ScaleTimeBulletProof(aircraft.aero.polar.CD, aircraft.aero.polar.Re, Re_num);
aircraft.aero.CD   = interp1q(aircraft.aero.polar.alpha_rad, CDvsAoA', alpha_rad);
aircraft.states.pitch_rad = freeStream.gamma_rad + alpha_rad;

% Roll, Yaw based on ground track, wind direction, and potentially sun
% position
switch mission.groundTrackPath
    case 'orbit'
                
        % If a wind speed is provided as well as a wind heading, update
        % aircraft yaw/roll for coordinated flight
        if ~isfield(freeStream, 'windHeading_rad') % If heading not provided then allow the aircraft to yaw itself along the ground track i.e assume no wind
            windV_ms = [0; 0];
        else
            windV_ms = freeStream.windSpeed_ms * [cos(freeStream.windHeading_rad); sin(freeStream.windHeading_rad)];
        end
        
        % Update position based on state
     	aircraft.states.s_m = mod(aircraft.states.s_m, max(mission.trajectory.s_m));
        aircraft.states.groundTrackAngle_rad = interp1q(mission.trajectory.s_m', mission.trajectory.groundTrackAngle_rad', aircraft.states.s_m);
        radius_m = interp1q(mission.trajectory.s_m', mission.trajectory.radius_m', aircraft.states.s_m);
        
    	% Update position based on state
        aircraft.states.xGround_m = interp1q(mission.trajectory.s_m', mission.trajectory.x_m', aircraft.states.s_m);
        aircraft.states.yGround_m = interp1q(mission.trajectory.s_m', mission.trajectory.y_m', aircraft.states.s_m);
        
            
    case 'sunPointing'
        
        % No wind assumption for the aircraft to perfectly point at the sun
        windV_ms = [0; 0];
        
        % Get sun position and direction and set aircraft attitude
        [location, curTimeStruct] = GetTimeLocation( mission );
        sunangle                  = sun_position(curTimeStruct, location);
        aircraft.states.groundTrackAngle_rad = sunangle.azimuth*pi/180;
        
        % Assume almost straight path
        radius_m = Inf;
        
    case 'constantTrackAngle'
        
       	% If a wind speed is provided as well as a wind heading, update
        % aircraft yaw/roll for coordinated flight
        if isfield(freeStream, 'windHeading_rad') % If heading not provided then allow the aircraft to yaw itself along the ground track i.e assume no wind
            windV_ms = freeStream.windSpeed_ms * [cos(freeStream.windHeading_rad); sin(freeStream.windHeading_rad)];
        else
            windV_ms = [0; 0];
        end
                
     	% Assume almost straight path
        radius_m = Inf;
end
      
% Climb rate
aircraft.states.hdot_ms = sin(freeStream.gamma_rad) * freeStream.VTAS_ms;
 
% Knowing ground track, TAS, and wind speed, figure aircraft yaw and
% ground speed.
delta = 4 * sum( windV_ms.*[cos(aircraft.states.groundTrackAngle_rad); sin(aircraft.states.groundTrackAngle_rad)] )^2 - 4 * (norm(windV_ms)^2 - freeStream.VTAS_ms^2);
aircraft.states.groundSpeed_ms = (2 * sum( windV_ms.*[cos(aircraft.states.groundTrackAngle_rad); sin(aircraft.states.groundTrackAngle_rad)] ) + sqrt(delta) )/2;
aircraft.states.yaw_rad     = atan2( (aircraft.states.groundSpeed_ms *  sin(aircraft.states.groundTrackAngle_rad) -  windV_ms(2)),  (aircraft.states.groundSpeed_ms *  cos(aircraft.states.groundTrackAngle_rad) -  windV_ms(1)));
aircraft.states.roll_rad    = atan(aircraft.states.groundSpeed_ms^2/(radius_m*mission.g_ms2));

end
