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
  
function [aircraft, freeStream] = ConstantSOC(aircraft, freeStream, mission, environment)
% This function attempts at keeping the battery state of charge constant 
% when power is still positive by climbing.

% Quasi-static assumption
freeStream.dVdt_ms2 = 0;

% Check how far from satisfied is the power condition
PeSLF_W     = aircraft.SLF.batteries.PeBatt_W;

% If the energy would change by less than 0.1% then it's good enough
energyError = PeSLF_W / aircraft.SLF.batteries.powerDraw_W;

if abs(energyError) > .0001
    
    if strcmp(aircraft.aero.mode, 'minPower') % Easily estimate climb rate required to trim power-wise
        gamma_rad   = asin(-PeSLF_W * aircraft.propulsion.propulsor(1).propeller.efficiency *  aircraft.propulsion.propulsor(1).motor.eta * aircraft.propulsion.propulsor(1).motor.controller_eta / (aircraft.massProperties.MGTOW_N * freeStream.VTAS_ms));
        freeStream.gamma_rad = gamma_rad;
        
    else % CL is unknown and therefore CD changes. Estimate gamma from two points
        gamma_rad   = 2 * pi/180;
        freeStream1 = setfield(freeStream, 'gamma_rad', gamma_rad);
        [aircraft1, freeStream1] = NetPower(aircraft, freeStream1,  mission, environment);
        Pe_W        = aircraft1.batteries.PeBatt_W;
        freeStream.gamma_rad = - PeSLF_W * gamma_rad/(Pe_W - PeSLF_W);
    end
    
    % If the resulting climb rate is not achievable by the propulsion
    % system (torque and power limits), limit climb rate
    [aircraft1, ~] = NetPower(aircraft, freeStream,  mission, environment);
    thrustResidual_N = aircraft1.propulsion.propulsor(1).propeller.thrustResidual_N;
        
    if thrustResidual_N ~= 0
        
        gammaLeft_rad   = 0;
        gammaRight_rad  = freeStream.gamma_rad;
        gammaMid_rad    = (gammaLeft_rad + gammaRight_rad)/2;
        dgamma_rad      = gammaRight_rad - gammaMid_rad;
        
        % Evaluate mid-point
        while dgamma_rad > 10/100 * gammaMid_rad
            
            gammaMid_rad = (gammaLeft_rad + gammaRight_rad)/2;
            dgamma_rad   = gammaRight_rad - gammaMid_rad;
            
            freeStream.gamma_rad = gammaMid_rad;
            [aircraft1, ~] = NetPower(aircraft, freeStream,  mission, environment);
            thrustResidual_N = aircraft1.propulsion.propulsor(1).propeller.thrustResidual_N;
            
            if thrustResidual_N ~= 0
                gammaRight_rad = gammaMid_rad;
            else
                gammaLeft_rad = gammaMid_rad;
            end
        end
        freeStream.gamma_rad = gammaLeft_rad;
    end
    
end

% Now that we have the correct climb angle, recalculate power
% and export it to propulsion
[aircraft, freeStream] = NetPower(aircraft, freeStream, mission, environment);

end

