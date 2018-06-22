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
  
function [aircraft, freeStream, dt_s] = StateMachine(aircraft,freeStream, mission, environment)
% This function codes the state machine for station holding.
% It ensures minimum power spend provided a minimum horizontal
% airspeed/wind speed.

% default time step
dt_s = mission.dt_s;

%% Steady level flight quantities
[aircraft, freeStream] = SLF(aircraft, freeStream, mission, environment);

%% State machine logic

% Harvest critical flags for logic
isSLFPowerBalancePos= aircraft.SLF.batteries.PeBatt_W < 0;
areBatteriesCharged = aircraft.batteries.energyLeft_J >= aircraft.batteries.capacity_J;
areBatteriesAlmostCharged = aircraft.batteries.energyLeft_J/aircraft.batteries.capacity_J >= aircraft.batteries.SOCstartClimb;
isAltitudeMax       = aircraft.states.h_m(end) >= mission.maxh_m;
isAltitudeMin       = aircraft.states.h_m(end) <= mission.minh_m;
isAircraftAccel     = abs(freeStream.dVdt_ms2) ~= 0;

% The aircraft power balance is positive - charge the batteries, climb, 
% or increase airspeed
if isSLFPowerBalancePos
    % Batteries are charging, fly SLF until almost full and start smooth
    % climb
    if ~areBatteriesCharged
        if areBatteriesAlmostCharged
            [aircraft, freeStream] = ChargeAndClimb(aircraft, freeStream, mission, environment);
        else
            freeStream = freeStream.SLF;
            aircraft   = aircraft.SLF;
        end
    % Batteries are charged
    else
        % There is room to climb
        if ~isAltitudeMax
            [aircraft, freeStream] = ConstantSOC(aircraft, freeStream, mission, environment);
        else % No room to climb
            [aircraft, freeStream, dt_s] = ConstantAltitude(aircraft, freeStream, mission, environment);
        end
    end
    
% The aircraft power balance is negative - either decelerate, glide down or
% hold altitude constant
else
    % There is room to glide down
    if ~isAltitudeMin
        % Let the aircraft deccelerate is it is faster than the minimum 
        % airspeed or power airspeed
        if isAircraftAccel
            [aircraft, freeStream, dt_s] = ConstantAltitude(aircraft, freeStream, mission, environment);
        else
            [aircraft, freeStream] = Glide(aircraft, freeStream, mission, environment);
        end
    % There is no room to glide down, fly STL
    else
        freeStream = freeStream.SLF;
        aircraft   = aircraft.SLF;
    end
end

end

