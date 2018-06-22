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

function batteries = IntegratedSolarPower(batteries, freeStream, environment, mission, aircraft)
% Calculates solar power collected at specified time step. Using solar
% module normal vectors and areas, vehicle attitude, and current location
% and time calculates total solar power collected.

if ~isfield(batteries, 'i')
    batteries.i = 1;
else
    batteries.i = batteries.i+1;
end
if batteries.i == 200
    x = 1;
end
batteries.i = batteries.i+1;

% Get location and sun position to find solar intensity.
DEG2RAD   = 2*pi/360;
RAD2DEG   = 1/DEG2RAD;
[ location, curTimeStruct ] = GetTimeLocation( mission, aircraft);

sunangle = SunPosition(curTimeStruct, location);

dim1 = round(location.altitude/500)+1;
dim2 = round(sunangle.zenith/0.5)+1;
[siz1, siz2] = size(environment.solar.intensity_power);
if siz1 >= dim1 && siz2 >= dim2
    solarIntensity = environment.solar.intensity_power(dim1,dim2);
else
    solarIntensity = 0;
end

%rotate normal vectors based on aircraft states
% Flip y component of normal vectors to match aircraft convertion
environment.solar.panels.normal = environment.solar.panels.normal.*[1 -1 1];
% Adjust nomal vectors based on aircraft attitude - use yaw, pitch,
% roll convention.
fn = environment.solar.panels.normal * SpinCalc('EA321toDCM',[ aircraft.states.yaw_rad*RAD2DEG -1*aircraft.states.pitch_rad*RAD2DEG -1*aircraft.states.roll_rad*RAD2DEG]);

% Find sun vector - vector pointing to the sun
% sunangle has North at 0 deg going clockwise; sph2cart has x axis in
% xy plane as 0 going counterclockwise; 90 - sunangle.azimuth corrects
% to 0 as North; rotate 90 about z to align with attiude axes
[sunX, sunY, sunZ]  = sph2cart((90-sunangle.azimuth)*DEG2RAD, (90 - sunangle.zenith)*DEG2RAD, 1);
rotate90 = [0, 1, 0; 1, 0, 0; 0, 0, 1];
sunVect             = [sunX sunY sunZ]*rotate90;
%     sunVect             = [sunX sunY sunZ];

[lengthAreaVect, ~] = size(fn);
sunVect             = repmat(sunVect, lengthAreaVect,1);
sunPower            = sunVect.*solarIntensity;

% Calcualte dot product between sun vectos and panel vectors
solarIntersityPerPanel = dot(fn, sunPower,2);

%remove shadowed panels
solarIntersityPerPanel(solarIntersityPerPanel<0) = 0;

%Calculate intensity and power per panel
solarPowerMagnitude = abs(solarIntersityPerPanel);
solarPowerPerPanel  = solarPowerMagnitude.*environment.solar.panels.area_m2 * environment.solar.panels.efficiency * environment.solar.panels.systemEfficency;
Psolar     = sum(solarPowerPerPanel);

% Actual power to the batteries - check if solar power collected
% exceeds mppt sizing power then truncate to sizing power
if Psolar > environment.solar.mppt_sizingPower_W
    batteries.solarPower_W = environment.solar.mppt_sizingPower_W * environment.solar.mppt_efficiency;
else
    batteries.solarPower_W = Psolar * environment.solar.mppt_efficiency;
end

batteries.PSwing = []; batteries.PSvtail = []; batteries.PSout = [];
batteries.azimuth = []; batteries.solarPowerPerPanel = []; batteries.sunVect = [];

end

%% Calculate angle between panel and sun
function [sunfactor, panelAngles] = incidence(states, sunangle, uavpanel, uavMIN_SOLAR_ANGLE)

DEG2RAD   = 2*pi/360;
RAD2DEG   = 1/DEG2RAD;

xyz2ned = [ 0  1  0 ; ...
    1  0  0 ; ...
    0  0 -1 ];
ned2xyz = xyz2ned;         % it actually happens that xyz2ned is its own inverse

[sunX, sunY, sunZ] = sph2cart((90 - sunangle.azimuth)*DEG2RAD, (90 - sunangle.zenith)*DEG2RAD, 1);
sunNED = xyz2ned * [sunX sunY sunZ]';

panelAngles = zeros(1,numel(uavpanel)); % init
sunfactor   = panelAngles;

for i=1:length(uavpanel)
    panelFRD = uavpanel(i).normal;  % in UAV def file, panels are oriented fore, right, down
    panelNED = rotrpy(states.roll_rad, states.pitch, states.yaw_rad) * panelFRD;
    
    panelAngles(i) = (atan2(norm(cross(sunNED,panelNED)),dot(sunNED,panelNED)))*RAD2DEG;
    if panelAngles(i) < (90 - uavMIN_SOLAR_ANGLE)
        sunfactor(i) = cosd(panelAngles(i)) * uavpanel(i).area_m * uavpanel(i).efficiency;
        % replace with a more sophisticated function than cosine in the future
    else
        sunfactor(i) = 0;
    end
end % end of loop through panels
end

%% Rotation matrix based on Euler angles
function R = rotrpy(alpha, beta, gamma)
% gamma=roll  beta=pitch  alpha=yaw
% there's probably a better way of doing this using vrrotvec2mat
R = [cosd(alpha)*cosd(beta)  cosd(alpha)*sind(beta)*sind(gamma) - sind(alpha)*cosd(gamma)  cosd(alpha)*sind(beta)*cosd(gamma) + sind(alpha)*sind(gamma); ...
    sind(alpha)*cosd(beta)  sind(alpha)*sind(beta)*sind(gamma) + cosd(alpha)*cosd(gamma)  sind(alpha)*sind(beta)*cosd(gamma) - cosd(alpha)*sind(gamma); ...
    -sind(beta)              cosd(beta)*sind(gamma)                                        cosd(beta)*cosd(gamma)                          ];
end
