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
  
function optim = DiscretizeSolarPanels(optim)
%This function tessellates the upper surface of horizontal surfaces and 
% both sides of vertical surfaces then stack ranks the tessellated patches 
% and selects the best patches until the desired relative solar area is 
% reached. 
%
% Outputs:
% fn   : [n,3] matrix containing each panels normal vector
% area : colum vector containing area of each panel
% TR   : [n,3] matrix describing the connectivity of each triangle - not
% needed for performance calcs, need for visualization
% XR   : [m, 3] matrix of x,y and z locations of each point refrenced in TR

%%%%%%%%%%%%% Horizontal Panel Sizing and Selection %%%%%%%%%%%%%%%%%%%%%%%
%% Get wing panel geometric properties
% If battery mass has changed then repanel wing, ensures only repanel when
% needed
if ~isfield(optim.solar, 'resizingSwitch') || ~isfield(optim.wing.solar, 'triangulation')
    optim.solar.resizingSwitch.previousBatteryMass_kg = optim.batteries.mass_kg(1);
    repanelWing = logical(1);
else 
    if optim.solar.resizingSwitch.previousBatteryMass_kg == optim.batteries.mass_kg(1)
        repanelWing = logical(0);
    else
        repanelWing = logical(1);
        optim.solar.resizingSwitch.previousBatteryMass_kg = optim.batteries.mass_kg(1);
    end
end

% Repanel wing if needed
if repanelWing
    [fnwall, wingSolarArea_m2, XbWing, PWing, optim ] = GetPanelProperties(optim, 'wing');
    optim.wing.solar.triangulation.fnwall             = fnwall;
    optim.wing.solar.triangulation.wingSolarArea_m2   = wingSolarArea_m2;
    optim.wing.solar.triangulation.XbWing             = XbWing;
    optim.wing.solar.triangulation.PWing              = PWing;
end

fnWing              = optim.wing.solar.triangulation.fnwall;
wingSolarArea_m2    = optim.wing.solar.triangulation.wingSolarArea_m2; 
XbWing              = optim.wing.solar.triangulation.XbWing; 
PWing               = optim.wing.solar.triangulation.PWing;

%% Get panels for htail
if isfield(optim, 'htail')
    [fnHtail, htailSolarArea, XbHtal, PHtail ] = GetPanelProperties(optim, 'htail');
    fnHorizontal = [fnWing; fnHtail];
    AreaHorizontal_m2 = [wingSolarArea_m2; htailSolarArea];
    XbHorizontal = [XbWing ; XbHtal];
    PHorizontal = [PWing; PHtail];
    [winglen, ~] = size(fnWing);
    [htlen, ~]   = size(fnHtail);
    wingIds     = [];
    htailIds    = [];
else
    fnHorizontal = [fnWing];
    AreaHorizontal_m2 = [wingSolarArea_m2];
    XbHorizontal = [XbWing ];
    PHorizontal = [PWing];
    [winglen, ~] = size(fnWing);
    wingIds     = [];
    
end

% rank solar cells according to collection effectiveness
[~, rankedHorizontalIndicies] = HorizontalSolarCellRanking(optim, fnHorizontal);

% Half desired area to work with a half span
horizontalSolarAreaDesired_m2_2 = sum(AreaHorizontal_m2)*optim.solar.relativeHorizontalArea;
horizontalSolarAreaActual = 0;
i = 0;

% Loop through stack ranked modules selecting the best remaining untill
% desired soalr area is met
while(horizontalSolarAreaActual < horizontalSolarAreaDesired_m2_2 &&  ...
        i < length(rankedHorizontalIndicies))
    i = i+1;
    horizontalSolarAreaActual = horizontalSolarAreaActual + ...
        AreaHorizontal_m2(rankedHorizontalIndicies(i));
    
     %assign which surface this panel belongs to
     if rankedHorizontalIndicies(i) <= winglen
         wingIds(end+1) = rankedHorizontalIndicies(i);
     elseif isfield(optim, 'htail')
         if rankedHorizontalIndicies(i) <= htlen + winglen
             htailIds(end+1) = rankedHorizontalIndicies(i);
         end
     end
end

% find which modules are from the wing and reflect for left wing
optim.wing.solar.fn         = [fnHorizontal(wingIds,:); fnHorizontal(wingIds,:).*[1 -1 1]];
optim.wing.solar.area_m2    = [AreaHorizontal_m2(wingIds); AreaHorizontal_m2(wingIds)];
optim.wing.solar.Xb         = [XbHorizontal(wingIds,:); XbHorizontal(wingIds,:).*[1 -1 1]];
optim.wing.solar.P          = [PHorizontal(wingIds,:,:); PHorizontal(wingIds,:,:).*[1 -1 1]];

% find which are from htail and reflect
if isfield(optim, 'htail')
optim.htail.solar.fn = [fnHorizontal(htailIds,:); fnHorizontal(htailIds,:).*[1 -1 1]];
optim.htail.solar.area_m2 = [AreaHorizontal_m2(htailIds); AreaHorizontal_m2(htailIds)];
optim.htail.solar.Xb = [XbHorizontal(htailIds,:); XbHorizontal(htailIds,:).*[1 -1 1]];
end

if isfield(optim, 'vtail')
%%%%%%%%% Vertical Area %%%%%%%%%%
%% Get panels for vtail
[ fnVtail, areaVtail_m2, XbVtail, PVtail ] = GetPanelProperties(optim, 'vtail');

% reflect across span and across x axis
rvv = [1, 1, 1; 1, 1, -1; 1, -1, 1; 1, -1, -1];
fnVtail = [fnVtail; fnVtail.*rvv(2,:); fnVtail.*rvv(3,:); fnVtail.*rvv(4,:)];
areaVtail_m2 = [areaVtail_m2;areaVtail_m2;areaVtail_m2;areaVtail_m2];
XbVtail = [XbVtail; XbVtail.*rvv(2,:); XbVtail.*rvv(3,:); XbVtail.*rvv(4,:)];
PVtail = [PVtail; PVtail.*rvv(2,:); PVtail.*rvv(3,:); PVtail.*rvv(4,:)];

% Get Sorted best panesl for vertical surfaces
[rankedVerticalIndicies] = GetVerticalSolarCellRankning(optim, fnVtail, PVtail);

verticalSolarAreaDesired_m2 = sum(areaVtail_m2)*optim.solar.relativeVerticalArea;
verticalSolarAreaActual = 0;
selectedVtailIds = [];
i = 0;

% Vertical surfaces modules are ranked based on height. It is assumeed all 
% have equal effectiveness since normal vectors are parallel/anitparallel.
% Higher modules are assumed to be less effected by self shading so they
% are choosen first. 
while(verticalSolarAreaActual < verticalSolarAreaDesired_m2 && i < length(rankedVerticalIndicies))
    i = i+1;
    verticalSolarAreaActual = verticalSolarAreaActual + areaVtail_m2(rankedVerticalIndicies(i));  
    selectedVtailIds(end+1) = rankedVerticalIndicies(i);
end

optim.vtail.solar.fn = fnVtail(selectedVtailIds,:);
optim.vtail.solar.area_m2 = areaVtail_m2(selectedVtailIds);
optim.vtail.solar.Xb = XbVtail(selectedVtailIds,:);
optim.vtail.solar.P = PVtail(selectedVtailIds,:,:);

end

if isfield(optim, 'htail') 
% Create total aircraft solar properties
optim.solar.totalSolarArea_m2       = sum(optim.wing.solar.area_m2) + sum(optim.htail.solar.area_m2) + sum(optim.vtail.solar.area_m2);
optim.solar.totalSolarfn            = [optim.wing.solar.fn; optim.htail.solar.fn; optim.vtail.solar.fn];
optim.solar.totalSolarAreaVect_m2   = [optim.wing.solar.area_m2; optim.htail.solar.area_m2; optim.vtail.solar.area_m2];
optim.solar.totalSolarXb            = [optim.wing.solar.Xb; optim.htail.solar.Xb; optim.vtail.solar.Xb];
%optim.solar.totalSolarP             = [optim.wing.solar.P; optim.htail.solar.P; optim.vtail.solar.P];
else
    optim.solar.totalSolarArea_m2       = sum(optim.wing.solar.area_m2) ;
optim.solar.totalSolarfn            = [optim.wing.solar.fn];
optim.solar.totalSolarAreaVect_m2   = [optim.wing.solar.area_m2];
optim.solar.totalSolarXb            = [optim.wing.solar.Xb];
end
end

function [totalHorizontalSolar_m2, horizontalIndicies] = HorizontalSolarCellRanking(optim, fn)
% Rank panels based on equivalent intensity at mid day on dec 21. Ranking 
% is done based on cumulative solar power gathered during a 360 degree turn.

% Run aircraft through 360 degree turn around Z axis

for az = [optim.environment.winterSolstaceSunZenith]
    sunangle.zenith = az;
    totalHorizontalSolar_m2 = zeros(length(fn),1);
    for i = 0:10:350
        sunangle.azimuth = i;
        sunangle.zenith  = optim.environment.winterSolstaceSunZenith;
        
        DEG2RAD   = 2*pi/360;
        [sunX, sunY, sunZ]  = sph2cart((90 - sunangle.azimuth)*DEG2RAD, (90 - sunangle.zenith)*DEG2RAD, 1);
        %sph2cart((sunangle.azimuth)*DEG2RAD, (90 - sunangle.zenith)*DEG2RAD, 1);
        sunVect             = [sunX sunY sunZ]; % <------- -1*Z
        solarPower          = 1300; %dummy value, actual value really doesn't matter here since comparision is relative
        [lengthAreaVect, ~] = size(fn);
        sunVect             = repmat(sunVect, lengthAreaVect,1);
        sunPower            = sunVect.*solarPower;

        % Calculate dot product between sun vectors and panel vectors
        solarIntensityPerPanel = dot(fn, sunPower,2);

        % Remove panels with negative solar flux (ie the sun shines on the
        %back of the panel
        solarIntensityPerPanel(solarIntensityPerPanel<0) = 0;

        % Calculate intensity and power per panel
        solarPowerMagnitude = abs(solarIntensityPerPanel);
        totalHorizontalSolar_m2 = totalHorizontalSolar_m2 + solarPowerMagnitude;
    end
end

% Now sort panels based on effectiveness
[totalHorizontalSolar_m2, horizontalIndicies] = sort(totalHorizontalSolar_m2);

horizontalIndicies = flipud(horizontalIndicies);

end

function [verticalIndicies] = GetVerticalSolarCellRankning(optim, fnvt, Pvt)
% This function ranks vertical solar cells based on the amount of power
% they generate when the sun if 5 degrees above the horizon off the right
% and left wing tip. This is done to establish which vertical cells are the
% most effective at gathering solar energy at dawn and dusk to move the
% power neutral point and shorten the length of the night. - Scheme
% simplified to just take panels from top of tail to bottom to avoid
% shadowing impact.

[~,verticalIndicies]=sort(Pvt(:,3));
verticalIndicies = flipud(verticalIndicies);

end
