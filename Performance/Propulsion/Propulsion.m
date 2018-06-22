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
  
function aircraft = Propulsion(aircraft, freeStream)
% This function converts the required propulsive power into shaft power and
% subsequently electrical power. It is called after the desired propulsive
% power (through thrust) is known.

for p = 1:length(aircraft.propulsion.propulsor)
    
    % Select active propulsor
    aircraft.propulsion.activePropulsor = p;
    
    % Compute required propulsive power
    aircraft.propulsion = Propeller(aircraft.propulsion, freeStream);

    % Compute electrical power
    aircraft.propulsion = Motor(aircraft.propulsion);
    
end

% Pull power from battery
aircraft.batteries = Battery(aircraft);

% Add battery energy to aircraft states for integration
if isempty( find(contains({aircraft.states.integration.var}, 'energyLeft_J'), 1) )
    aircraft.states.integration(end+1).var = 'aircraft.batteries.energyLeft_J';
    aircraft.states.integration(end).der   = '-aircraft.batteries.PeBatt_W';
    aircraft.states.integration(end).bounds= [-aircraft.batteries.capacity_J, aircraft.batteries.capacity_J];
end
end


