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
  
function [aircraft, freeStream] = ChargeAndClimb(aircraft, freeStream, mission, environment)
% This function represents the intermediate energy mode that consists in
% climbing while charging the batteries. It is used when the batteries
% reach a target SOC < 100% to smoothly transition to full climb at
% constant SOC. The climb rate is adjusted so that the aircraft undergoes a
% smooth transition.

% Record current SOC
SOC  = aircraft.batteries.SOC;

% Quasi-static assumption
freeStream.dVdt_ms2 = 0;

% Compute the climb rate the aircraft would undergo if the batteries
% where charged and the all the solar energy would be pushed into
% propulsion
[~, freeStream1] = ConstantSOC(aircraft, freeStream, mission, environment);
hdotFullClimb_ms = sin(freeStream1.gamma_rad) * freeStream.VTAS_ms;

% Scale the above climb rate by the transition function based on the
% value of the SOC
blend = Transition((SOC-aircraft.batteries.SOCstartClimb)/(1 - aircraft.batteries.SOCstartClimb), 1/2);

% Assign updated climb rate
hdot_ms = hdotFullClimb_ms * blend;
freeStream.gamma_rad = asin(hdot_ms / freeStream1.VTAS_ms);

% Now that we have the correct climb angle, recalculate power
% and export it to propulsion
[aircraft, freeStream] = NetPower(aircraft, freeStream, mission, environment);

end
