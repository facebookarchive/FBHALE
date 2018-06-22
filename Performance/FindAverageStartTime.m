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
  
function [starttime, s_m] = FindAverageStartTime(mission, aircraft, freeStream, environment)
% The start time i.e the time at which the required power equals the
% incoming power depends on the aircraft heading/starting location on the 
% ground trajectory. Finds the average time across headings/locations.

N   = 10;
s_m = linspace(0, max(mission.trajectory.s_m), N+1);
s_m = s_m(1:end-1); % The loop closes so last point = first point
startTimes = zeros(1, N);

% for each position find the break-even time
for s = 1:N
  mission.s_m  = s_m(s);
  minstarttime = FindMinSOCTime(mission, aircraft, freeStream, environment);
  startTimes(s)= minstarttime.Hour + minstarttime.Minute/60 + minstarttime.Second/60/60;
end

% Remove potential NaNs for robustness
s_m(isnan(startTimes))          = [];
startTimes(isnan(startTimes))	= [];

% find start time Hour, Minute, Second
avestarttime = mean(startTimes);
startHour = floor(avestarttime);
startMinute = floor((avestarttime - startHour)*60);
startSecond = floor((avestarttime - startHour - startMinute/60)*60*60);

% get start date
[startYear, startMonth, startDay] = ymd(mission.starttime);

starttime = datetime(startYear,startMonth,startDay,startHour,...
                     startMinute,startSecond);

% find start location
[~, iMax] = max(startTimes);
[~, iMin] = min(startTimes);
[startTimes, I] = unique(startTimes(min(iMin, iMax):max(iMin, iMax)));
s_m             = s_m(min(iMin, iMax):max(iMin, iMax));
s_m             = s_m(I);

s_m = interp1(startTimes, s_m, avestarttime, 'pchip');

end

