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
  
function [ location, curTimeStruct ] = GetTimeLocation( mission, aircraft )
% Returns time and location for solar intensity lookup.

% convert variables for internal use
location.altitude   = aircraft.states.h_m(end);
location.latitude   = mission.lat; % in the future this should be a growing array. currently it is just a scalar
location.longitude  = mission.lon;
location.timezone   = mission.timezone;

curTime = mission.starttime + seconds(mission.t_s(end) + mission.leg.t_s(end));
curTimeStruct.year  = year(curTime);  % need to create this second struct because of the timezone
curTimeStruct.month = month(curTime);
curTimeStruct.day   = day(curTime);
curTimeStruct.hour  = hour(curTime);
curTimeStruct.min   = minute(curTime);
curTimeStruct.sec   = second(curTime);
curTimeStruct.UTC   = location.timezone;

end

