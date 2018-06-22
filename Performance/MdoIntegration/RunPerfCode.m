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
  
function [minimumSOC, optim, endOfMissionSOC] = RunPerfCode(optim, varargin)
% This function runs the current airfracft described by the optim data 
% structure through the missions perscribed in the same structure.
% It returns 2 vetors: endOfMissionStateofCharge and
% minimumSOC to be used by the optimizer. The order of these outputs in
% each vector is [climbMission, minPower Mission, maxWindSpeed Mission].

if isempty(varargin)
    missionsToRun = {'climb', 'maxWind'};
else
    missionsToRun = varargin;
end

endOfMissionSOC = [];
minimumSOC =[];

% load aircraft
[aircraft, environment] = GenerateAircraft(optim);

% Only run mission #2 i.e high wind at winter solstice
[eomSOC, minSOC, mission]= RunMission(optim, missionsToRun{2}, aircraft, environment);
endOfMissionSOC = eomSOC;
minimumSOC = minSOC;
optim.performance.mission = mission;
optim.performance.aircraft = aircraft;
optim.performance.environment = environment;
optim.performance.missionType = missionsToRun{2};
end


