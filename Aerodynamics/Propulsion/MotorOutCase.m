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
  
function optim = MotorOutCase(optim)
% This function compute residual yaw when the outter motor goes down during
% climb conditions.
% It assumes that the remaining motors will adjust their thrust to 
% minimize the generated yaw.

% Unpack
y_m     = optim.propulsion.y_m(optim.propulsion.y_m~=0);
y_m     = [-y_m(end:-1:1) y_m];
Tmax_N  = optim.mission.climb.TSLreq_N/sum(optim.propulsion.N);
D_N     = optim.mission.climb.DSL_N;
V_ms    = optim.mission.climb.VSL_ms;
q_Pa    = 1/2*optim.constants.rhoSL_kgm3*V_ms^2;

% Target climb rate for degraded propulsion: Achieve X% of sea level climb
% rate with one motor out
hdot_ms = optim.propulsion.relativeClimbRateMotorOut * optim.mission.climb.hdotSL_ms;
Treq_N  = hdot_ms/V_ms*optim.MGTOW_kg * optim.constants.g_ms2 + D_N;

% Compute residual yaw
switch sum(optim.propulsion.N)
    case 1
        optim.propulsion.CnMotorOut = 0;
        optim.propulsion.CnDiffThrustLanding  = 0;
        optim.propulsion.CnDiffThrustCruise   = 0;
        
    case 2
        
        Touter_N = Treq_N;
        yaw_Nm   = max(0, Touter_N * max(y_m));
        optim.propulsion.CnMotorOut = yaw_Nm / optim.aircraft.bref_m / q_Pa / optim.aircraft.Sref_m2;
       
    case 4
       
        Tinner_N = 2*Tmax_N;
        Touter_N = Treq_N - Tinner_N;
        yaw_Nm   = max(0, Touter_N * max(y_m));
        optim.propulsion.CnMotorOut = yaw_Nm / optim.aircraft.bref_m / q_Pa / optim.aircraft.Sref_m2;
end

end