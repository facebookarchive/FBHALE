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
  
% This routine assigns all the design inputs used to describe the various
% mission phases.
function optim = CONOPs(optim)

%% General Mission Quantities
%optim.mission.lat_deg    = 30; % Input variable
optim.mission.long_deg   = -110;
optim.mission.timezone   = -10; 
optim.mission.maxh_m     = 25908; %25,908 m = 85,000 ft
winterSolsticeSunZenith = load('winterSolsticeSunZenith.mat');

% Bullet-proof lat interp
lat_deg = min(max(winterSolsticeSunZenith.latitude_deg), optim.mission.lat_deg);
optim.environment.winterSolstaceSunZenith = interp1f(winterSolsticeSunZenith.latitude_deg, ...
    winterSolsticeSunZenith.sunZenith_deg, lat_deg); %used for solar panel selection 90-elevation angle at noon on winter solstice - should be updated to be latitude dependent

%% Climb Mission
optim.mission.climb.tclimb_min       = 400;
optim.mission.climb.defaultDt_s      = 10;
optim.mission.climb.initialh_m       = 0;
optim.mission.climb.maxh_m           = optim.mission.maxh_m;
optim.mission.climb.minh_m           = optim.mission.minh_m;
optim.mission.climb.hdotRequested_ms = optim.mission.minh_m / (60 * optim.mission.climb.tclimb_min);
optim.mission.climb.startTime        = datetime(2016,12,20,0,00,0);
optim.mission.climb.exitVariable     = 'aircraft.states.h_m';
optim.mission.climb.exitValue        = optim.mission.minh_m;
optim.mission.climb.groundTrackPath  = 'orbit';
optim.mission.climb.groundTrackFigure = 'circle';
optim.mission.climb.radius_m         = 3000;
optim.mission.climb.windSpeed_ms     = 0;

%% MaxWind Mission
optim.mission.maxWind.defaultDt_s    = 60;
optim.mission.maxWind.initialh_m     = optim.mission.minh_m;
optim.mission.maxWind.maxh_m         = optim.mission.maxh_m;
optim.mission.maxWind.minh_m         = optim.mission.minh_m;
optim.mission.minimumAllowableAltitudeForStationKeeping_m = 18440; 
optim.mission.maxWind.startTime      = datetime(2016,12,21,8,00,0); % 21 Dec 2016, 8:00:00 AM;
optim.mission.maxWind.exitVariable   = 'mission.leg.t_s';
optim.mission.maxWind.exitValue      = 24*3600;
optim.mission.maxWind.groundTrackPath = 'orbit';
optim.mission.maxWind.groundTrackFigure = 'circle';
optim.mission.maxWind.radius_m       = 3000;
%optim.mission.maxWind.windHeading_deg= 0;   	% If no heading specified will assume all directions
optim.mission.maxWind.windSpeed_ms   = optim.mission.windSpeed_ms;

end
