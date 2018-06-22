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
  
function [eomSOC, minSOC, mission] = RunMission(optim, missionType, aircraft, environment)
% This routine runs a given aircraft through a selected mission (climb or
% 24hr worst day). Output quantities are passed back to MDOTopLevel for
% constraints to be applied by the optimizer.

% Check what mission needs to be run
if strcmp(missionType, 'climb')
    % Update avionics power draw based on mission
    aircraft.batteries.avionicPowerDraw_W = aircraft.batteries.avionicPowerDraw_W.climb;
    % Run Mission
   [aircraft, mission] = RunClimbMission(optim, aircraft, environment); 
elseif strcmp(missionType, 'maxWind')
    % Update avionics power draw based on mission
    aircraft.batteries.avionicPowerDraw_W = aircraft.batteries.avionicPowerDraw_W.maxWind;
    % Run Mission
   [aircraft, mission] = RunMaxWindMission(optim, aircraft, environment); 
end

% Find optimization quantities
SOC    = GetLoggedVariable(mission, 'mission', 'SOC');
eomSOC = SOC(end);
minSOC = min(SOC(floor(length(SOC)/2):end));

clear aircraft environment

end

