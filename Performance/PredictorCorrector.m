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
  
function [aircraft, freeStream, mission] = PredictorCorrector(aircraft, freeStream, mission, environment)
% For key performance parameters and aircraft states such as power, 
% ground speed, hdot, provide a two-point estimate for integration to be
% 2nd order.

% Compute current time step
[aircraft, freeStream] = ComputeTimeStep(aircraft, freeStream, mission, environment);

% Predict next step. Compute and store key state derivatives
[aircraft1, freeStream1, mission1] = StateIntegration(aircraft, freeStream, mission);
[aircraft1, ~] = ComputeTimeStep(aircraft1, freeStream1, mission1, environment);

% 2-point average the state derivative vector between steps. 
aircraft = StateDerivativeAverage(aircraft, aircraft1);

end

% This routine devises which strategy should be followed 
function [aircraft, freeStream, dt_s] = ComputeTimeStep(aircraft, freeStream, mission, environment)

dt_s = mission.defaultDt_s;

switch mission.leg.mode
    case 'conops'
        [aircraft, freeStream] = NetPower(aircraft, freeStream, mission, environment);
    case 'climbMaxPower'
        [aircraft, freeStream] = ClimbMaxPower(aircraft, freeStream, mission, environment);
    case 'constantSOC'
        [aircraft, freeStream] = SLF(aircraft, freeStream, mission, environment);
        [aircraft, freeStream] = ConstantSOC(aircraft, freeStream, mission, environment);    
	case 'constantAltitude'
        [aircraft, freeStream] = ConstantAltitude(aircraft, freeStream, mission, environment);
    case 'stateMachine'
        [aircraft, freeStream, dt_s] = StateMachine(aircraft, freeStream, mission, environment);
end

end


function aircraft = StateDerivativeAverage(aircraft0, aircraft1)

statesDer = zeros(1, length(aircraft0.states.integration));

for s = 1:length(aircraft0.states.integration)
    aircraft  = aircraft0;
    statesDer(s) = statesDer(s) + eval(aircraft.states.integration(s).der);
    aircraft  = aircraft1;
    statesDer(s) = (statesDer(s) + eval(aircraft.states.integration(s).der))/2;
end

aircraft = aircraft0;

end


