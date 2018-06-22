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
  
function [aircraft, mission] = RunClimbMission( optim, aircraft, environment )
% This routine simulates climb from sea-level to the minimum cruise
% altitude. This mission assumes no wind. The aircraft follows a given constant
% ground track and climbs at a given climb rate.
% No solar power is assumed to be collected to be conservative (solar
% panel efficiency set to zero).

environment.solar.panels.efficiency = 0; % de-activate solar panels for climb

% Start the clock and the mission integration parameters
mission.t_s         = 0;
mission.defaultDt_s = optim.mission.climb.defaultDt_s; 
mission.h_m         = optim.mission.climb.initialh_m;
mission.maxh_m      = optim.mission.climb.maxh_m; %25908 m = 85,000 ft
mission.minh_m      = 0; 
mission.starttime   = optim.mission.climb.startTime; % 20 Dec 2016, 12:00:00 PM
mission.lat         = optim.mission.lat_deg;
mission.lon         = optim.mission.long_deg;
mission.timezone    = optim.mission.timezone;
mission.g_ms2       = optim.constants.g_ms2;

% Generate trajectory
mission.groundTrackPath = optim.mission.climb.groundTrackPath;
mission.groundTrackFigure = optim.mission.maxWind.groundTrackFigure;
switch optim.mission.climb.groundTrackFigure
    case 'circle'
        mission.trajectory  = MakeCircle(optim.mission.climb.radius_m);
    case 'figure8'
        mission.trajectory  = MakeFigureEight(optim.mission.climb.radiusX_m, optim.mission.climb.radiusY_m);
end
mission.s_m         = 0;  % Initial position on the ground trajectory

% Set wind speed, no wind heading
freeStream.windSpeed_ms = optim.mission.climb.windSpeed_ms;

% Ensure Battery if fully charged
aircraft.batteries.energyLeft_J = aircraft.batteries.capacity_J; 

% Set mission leg type and stopping criteria
mission.leg.mode     = 'climbMaxPower'; 
%aircraft.states.hdotRequested_ms = optim.mission.climb.hdotRequested_ms;
aircraft.aero.mode   = 'minPower';
legExit.variable     = optim.mission.climb.exitVariable;
legExit.value        = optim.mission.climb.exitValue;
legExit.criterion    = @(x) gt(x, legExit.value);
legExit.tolerance    = .01;
mission.leg.legExit  = legExit;
[mission, aircraft, freeStream] = RunMissionLeg(mission, aircraft, freeStream, environment);

end

