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
  
function starttime = FindMinSOCTime(mission, aircraft, freeStream, environment)
% This function finds the time at which the SOC time history reaches a
% minimum. This value is then used as the starting point for the
% simulation.

% Assumption is that min SOC is reached at min altitude while trying to
% hold it constant.
startingHours = 1:1:7;

% Initialize leg position values
mission.dt_s    = 1;
mission.t_s = 0;
mission.h_m = mission.minh_m;

% Set a time criteria for the leg exit
mission.leg.mode     = 'stateMachine'; 
freeStream.gamma_rad = 0;
legExit.variable     = 'mission.leg.t_s(end)';
legExit.value        = 2; 
legExit.criterion    = @(x) gt(x, legExit.value);
legExit.tolerance    = 1;
mission.leg.legExit  = legExit;

% Find the hour
for h = 1:length(startingHours)
    mission.starttime = datetime(mission.starttime.Year, mission.starttime.Month, mission.starttime.Day ,startingHours(h),00,0);
    [mission1, aircraft, freeStream] = RunMissionLeg(mission, aircraft, freeStream, environment);
    Pe1_W = GetLoggedVariable(mission1, 'leg', 'PeBatt_W');
    Pe_W(h) = Pe1_W(end);
end

% Remove the non-unique values (night values will be the same)
[Pe_W I] = unique(Pe_W);
startingHours = startingHours(I);
startingHour    = floor(interp1(Pe_W, startingHours, 0));
startingMinute0 = floor( (interp1(Pe_W, startingHours, 0) - startingHour) * 60 );

% Check how far from arrived we are
%mission.starttime = datetime(mission.starttime.Year, mission.starttime.Month, mission.starttime.Day,startingHour,startingMinute0,0);
[mission1, aircraft, freeStream] = RunMissionLeg(mission, aircraft, freeStream, environment);
     
% Find the minute
Pe1_W   = GetLoggedVariable(mission1, 'leg', 'PeBatt_W');
Pe0_W   = Pe1_W(end); 
dMinute = 2;
iter    = 0;

while abs(dMinute) > 1 && iter < 20
    startingMinute1   = startingMinute0 + floor(dMinute);
    [missionStartYear, missionStartMonth, MissionStartDay] = ymd(mission.starttime);
    mission.starttime = datetime(missionStartYear,missionStartMonth,MissionStartDay,startingHour,startingMinute1,0);
    [mission1, aircraft, freeStream] = RunMissionLeg(mission, aircraft, freeStream, environment);
    Pe1_W = GetLoggedVariable(mission1, 'leg', 'PeBatt_W');
    Pe1_W = Pe1_W(end);
    dPedMin = (Pe1_W - Pe0_W)/(startingMinute1 - startingMinute0);
    dMinute = - Pe0_W / dPedMin / 2;
    Pe0_W   = Pe1_W;
    startingMinute0 = startingMinute1;
    iter = iter + 1;
end

starttime = datetime(mission.starttime.Year, mission.starttime.Month, mission.starttime.Day,startingHour,startingMinute0,0);

end

