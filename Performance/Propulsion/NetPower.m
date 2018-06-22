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
  
function [aircraft, freeStream] = NetPower(aircraft, freeStream, mission, environment)
% This subroutine computes the net power to the batteries taken to achieve
% a given flight condition. It is the building block of mission
% integration. In addition to being used for state of charge integration, 
% at each time step it is used to compare the power required
% for steady level flight against incoming power. This feeds accordingly 
% into the power management state machine used for CONOPs.

% If no mode is specified then assume the trim is specified by the mission
if ~isfield(aircraft.aero, 'mode')
	aircraft.aero.mode = 'minPower';
end
        
% Trim the airplane
[aircraft, freeStream, mission] = FlightMechanicsExplicit(aircraft, freeStream, mission);

% Compute solar power based on trim, position, etc... Store it in
% propulsion.battery
switch environment.solar.description
    case 'perf3cmodel'
        aircraft.batteries = IntegratedSolarPower(aircraft.batteries, freeStream, environment, mission, aircraft);
    otherwise
        aircraft.batteries = SolarPower(propulsion, freeStream, solar, mission);
end

% Call the propulsion system
aircraft = Propulsion(aircraft, freeStream, mission.dt_s);

end
