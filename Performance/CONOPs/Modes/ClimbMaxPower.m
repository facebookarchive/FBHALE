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
  
function [aircraft, freeStream] = ClimbMaxPower(aircraft, freeStream, mission, environment)
% This function attempts at keeping the battery state of charge constant 
% when power is still positive by climbing

% Quasi-static assumption
freeStream.dVdt_ms2 = 0;

% Check if the resulting climb rate is achievable by the propulsion
% system (torque and power limits), limit climb rate
%[aircraft1, ~] = NetPower(aircraft, freeStream,  mission, environment);
%thrustResidual_N = aircraft1.propulsion.propulsor(1).propeller.thrustResidual_N;

%gamma_rad = linspace(1, 10, 10) * pi/180;
freeStream.gamma_rad = 10*pi/180;
hdot_ms      = sin(freeStream.gamma_rad) * freeStream.VTAS_ms;
deltaHdot_ms = 100;
i = 1;

while abs(deltaHdot_ms) > 5/100*hdot_ms
    [aircraft1, ~] = NetPower(aircraft, freeStream,  mission, environment);
    thrustResidual_N = aircraft1.propulsion.propulsor(1).propeller.thrustResidual_N;
	thrustResiduals_N(i) = thrustResidual_N;
    gamma_rad(i)         = freeStream.gamma_rad;
    
    deltaHdot_ms = thrustResidual_N/aircraft.massProperties.MGTOW_N*freeStream.VTAS_ms;
    hdot_ms = sin(freeStream.gamma_rad) * freeStream.VTAS_ms + deltaHdot_ms;
    freeStream.gamma_rad = asin(hdot_ms/freeStream.VTAS_ms);
    
    % As soon as there's a sign change get out and interpolate in between    
    if i > 2
        if thrustResiduals_N(i)*thrustResiduals_N(i-1) < 0
            [thrustResiduals_N, I] = sort(thrustResiduals_N);
            freeStream.gamma_rad = interp1(thrustResiduals_N, gamma_rad(I), 0);
            break
        end
    end
    i = i + 1;
end

% Now that we have the correct climb angle, recalculate power
% and export it to propulsion
[aircraft, freeStream] = NetPower(aircraft, freeStream, mission, environment);

end

