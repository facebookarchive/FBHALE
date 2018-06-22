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
  
function [aircraft, freeStream] = Glide(aircraft, freeStream, mission, environment)
% This function attempts at minimizing the energy consumption by gliding
% down. If solar power is still positive, then power-glide.

% Record current SOC
aircraft.batteries.energyLeft_J0 = aircraft.batteries.energyLeft_J;
aircraft.batteries.SOC0          = aircraft.batteries.SOC(end);

% Quasi-static assumption
freeStream.dVdt_ms2 = 0;

% Check how far from satisfied is the power condition PeSolar = PeProp.
% Implies that subsystems are paid second after flight.
PeNetSLF_W  = aircraft.SLF.batteries.PeProp_W - aircraft.SLF.batteries.PeSolar_W;

% If the energy would change by less than 0.1% then SLF good enough
energyError = PeNetSLF_W / aircraft.SLF.batteries.powerDraw_W;

if abs(energyError) > .0001 % Find sink rate that will use all left-over solar (if any)
    
    if strcmp(aircraft.aero.mode, 'minPower')
        
        % Compute residual thrust
      	TSLF_N  = aircraft.SLF.propulsion.propulsor(1).propeller.thrust_N;
       	dTSLF_N = -PeNetSLF_W * aircraft.SLF.propulsion.propulsor(1).propeller.efficiency *  aircraft.propulsion.propulsor(1).motor.eta * aircraft.propulsion.propulsor(1).motor.controller_eta / freeStream.VTAS_ms;
       	aircraft.propulsion.propulsor(1).propeller.thrust_N = TSLF_N + dTSLF_N;
        if aircraft.SLF.batteries.PeSolar_W == 0
            aircraft.propulsion.propulsor(1).propeller.thrust_N = 0;           
        end
            
        % Propeller optimal efficiency at that thrust
        aircraft.propulsion = VariableEfficiencyPropeller(aircraft.propulsion, freeStream);
       
        % Resulting climb rate
        gamma_rad   = asin(dTSLF_N / aircraft.massProperties.MGTOW_N);
      	freeStream.gamma_rad = gamma_rad;
        
    else
        
        % CL is unknown and therefore CD changes. Estimate gamma from two points
        gamma_rad   = -2 * pi/180;
        freeStream1 = setfield(freeStream, 'gamma_rad', gamma_rad);
        [aircraft1, ~] = NetPower(aircraft, freeStream1, mission, environment);
                    
        PeNet_W     = aircraft1.batteries.PeProp_W  - aircraft1.batteries.PeSolar_W;
        gamma_rad  	= - PeNetSLF_W * gamma_rad/(PeNet_W - PeNetSLF_W);
        freeStream.gamma_rad = gamma_rad;
	end
    
end

% If solar power is too small (or zero), thrust can't adjust to get
% to zero shaft power as propeller power at CT = 0 isn't zero.
% Check for thrust residual and adjust above computed sink rate
[aircraft1, freeStream] = NetPower(aircraft, freeStream, mission, environment);

if aircraft1.propulsion.propulsor.propeller.thrustResidual_N ~= 0
    gamma_rad = gamma_rad + asin(aircraft1.propulsion.propulsor.propeller.thrustResidual_N / aircraft.massProperties.MGTOW_N);
    freeStream.gamma_rad = gamma_rad;
end
        
% Now that we have the correct climb angle, recalculate power
% and export it to propulsion
[aircraft, freeStream] = NetPower(aircraft, freeStream, mission, environment);

end

