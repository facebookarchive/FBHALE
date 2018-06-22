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
  
function mission = Logging(mission, mode, aircraft, freeStream)
% This routine logs selected variables into a logging structure inside the
% mission.

% Logging quantities
quantities = {  'mission.leg.t_s', ...
                'aircraft.states.xGround_m', ...
                'aircraft.states.yGround_m', ...
                'aircraft.states.h_m', ...
                'aircraft.batteries.PeSolar_W', ...
                'aircraft.batteries.PeBatt_W', ...
                'aircraft.batteries.SOC', ...
                'aircraft.aero.CL', ...
                'freeStream.VTAS_ms', ...
				'freeStream.gamma_rad', ...
                'aircraft.states.groundSpeed_ms', ...
                'aircraft.states.yaw_rad', ...
                'aircraft.states.roll_rad', ...
                'aircraft.states.groundTrackAngle_rad', ...
                'aircraft.propulsion.propulsor(1).propeller.thrust_N', ...
                'aircraft.propulsion.propulsor(1).propeller.efficiency', ...
                'aircraft.propulsion.propulsor(1).propeller.shaftPower_W', ...
                'aircraft.propulsion.propulsor(1).propeller.torque_Nm', ...
                'aircraft.propulsion.propulsor(1).propeller.RPM', ...
                'aircraft.propulsion.propulsor(1).propeller.theta_deg', ...
                'aircraft.batteries.solarPower_W', ...
                'aircraft.propulsion.propulsor(1).propeller.thrustResidual_N', ...
                'aircraft.propulsion.propulsor(1).propeller.thrustActual_N'
                };

% Based on logging mode, initialize or append
switch mode
    
    case 'initialize'
    
    for q = 1:length(quantities)
        
        mission.leg.logging(q).name   = quantities{q};
        mission.leg.logging(q).values = [];
        
    end
    
    % If mission logging hasn't been initialized yet, do so now
    if ~isfield(mission, 'logging')
        mission.logging = mission.leg.logging;
    end
    
    case 'appendTimeStep' % Append current time step to current leg logging
        
        for q = 1:length(quantities)
            mission.leg.logging(q).values(end+1) = eval(quantities{q});
        end
        
   	case 'appendLeg' % Append the full leg logging to the mission logging
        
        for q = 1:length(quantities)
        
            mission.logging(q).values = [mission.logging(q).values mission.leg.logging(q).values];
        
        end
        
end
