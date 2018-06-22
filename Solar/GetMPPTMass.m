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
  
function [ mpptMass_kg, maxSolarPowerCollected_W ] = GetMPPTMass( optim, fn , area_m2 )
% This function rotates the aircraft through 360 degrees of heading at solar noon
% on the winter solstice and sizes the mppt based on the max solar power
% collected at any heading. Mppt power is capped at this value.

totalSolarPowerCollected_W = []; 

% Load in solar intensity table
solarIntensityTable = load('Solar/absorbed_intensity.mat');

% Loop through headings and record collected solar power
for i = 0:10:350
    sunangle.azimuth = i;
    sunangle.zenith  = optim.environment.winterSolstaceSunZenith; % sun zenith at solar noon on winter solstice
    DEG2RAD   = 2*pi/360;
    [sunX, sunY, sunZ]  = sph2cart((90 - sunangle.azimuth)*DEG2RAD, (90 - sunangle.zenith)*DEG2RAD, 1);
    sunVect             = [sunX sunY sunZ]; 
    dim1 = round(optim.mission.maxh_m/500)+1; %use max altitude because solar intensity gets higher with altitude
    dim2 = round(sunangle.zenith/0.5)+1;
    solarIntensity_W_m2 = solarIntensityTable.intensity_power(dim1,dim2);
    [lengthAreaVect, ~] = size(fn);
    sunVect             = repmat(sunVect, lengthAreaVect,1);
    sunPower_W_m2            = sunVect.*solarIntensity_W_m2;
    
    % Calculate dot product between sun vectors and panel vectors
    solarIntensityPerPanel = dot(fn, sunPower_W_m2,2);
    
    % Remove panels with negative solar flux
    solarIntensityPerPanel(solarIntensityPerPanel<0) = 0;
    
    % Calculate intensity and power per panel
    solarPowerMagnitude = abs(solarIntensityPerPanel);
    solarPowerPerPanel  = solarPowerMagnitude.*area_m2 * optim.solar.panelEfficiency * optim.solar.systemEfficiency;
    totalSolarPowerCollected_W(end+1) = sum(solarPowerPerPanel);
     
end

% Find max observed power
maxSolarPowerCollected_W = max(totalSolarPowerCollected_W);

% Size mppt based on max observed power
mpptMass_kg = optim.solar.mppt.kg_kW*maxSolarPowerCollected_W/1000;

end

