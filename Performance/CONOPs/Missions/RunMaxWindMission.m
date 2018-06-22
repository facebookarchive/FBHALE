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
  
function [aircraft, mission] = RunMaxWindMission( optim, aircraft, environment )
% This mission simulates the aircraft performance for 24h during at a user
% specified time of the year and location. A minimum airspeed is specified
% to enforce the aircraft to fly faster than the wind. If no wind heading 
% is specified, the aircraft will fly the specified ground track figure
% with no wind consideration but through airspeed.
% Power management follows the standard power management scheme.

% Start the clock and the mission integration parameters
mission.t_s         = 0;
mission.defaultDt_s = optim.mission.maxWind.defaultDt_s; 
mission.h_m         = optim.mission.maxWind.initialh_m;
mission.maxh_m      = optim.mission.maxWind.maxh_m; %25908 m = 85,000 ft
mission.minh_m      = optim.mission.maxWind.minh_m; %18288 m = 60,000 ft 
mission.lat         = optim.mission.lat_deg;
mission.lon         = optim.mission.long_deg;
mission.timezone    = optim.mission.timezone;
mission.g_ms2       = optim.constants.g_ms2;

% Generate trajectory
mission.groundTrackPath = optim.mission.maxWind.groundTrackPath;
mission.groundTrackFigure = optim.mission.maxWind.groundTrackFigure;
switch optim.mission.maxWind.groundTrackFigure
    case 'circle'
        mission.trajectory  = MakeCircle(optim.mission.maxWind.radius_m);
    case 'figure8'
        mission.trajectory  = MakeFigureEight(optim.mission.maxWind.radiusX_m, optim.mission.maxWind.radiusY_m);
end
mission.s_m         = 0;  % Initial position on the ground trajectory

% Set wind speed, no wind heading
freeStream.windSpeed_ms = optim.mission.maxWind.windSpeed_ms;

% Set mission leg type and stopping criteria
mission.leg.mode     = 'stateMachine'; 
legExit.variable     = optim.mission.maxWind.exitVariable;
legExit.value        = optim.mission.maxWind.exitValue; %mission runs one full day
legExit.criterion    = @(x) gt(x, legExit.value);
legExit.tolerance    = .1;
mission.leg.legExit  = legExit;
mission.starttime    = optim.mission.maxWind.startTime;

% Start time is found as the time at which power is neutral. It depends on
% the initial position along the trajectory. Find the position that yields
% the average start time
[mission.starttime, mission.s_m] = FindAverageStartTime(mission, aircraft, freeStream, environment);

% Run leg
[mission, aircraft, freeStream] = RunMissionLeg(mission, aircraft, freeStream, environment);

end
