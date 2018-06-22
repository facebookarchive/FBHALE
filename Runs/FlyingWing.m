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
  
clear all
close all

% If Mac or Unix system add to path xfoil and aswing
if strfind(computer, 'MAC')
    PATH = getenv('PATH');
    setenv('PATH', [PATH ':~/Documents/AeroCodes/XfoilDP/bin:~/Documents/AeroCodes/AswingMDOfaero/bin:~/Documents/AeroCodes/Xrotor/bin']);
end

% Check for required codes to be in the path
isReady = CheckSystem();

% Check local directory
fileList = dir('.');
if ~contains([fileList.name], 'MDOTopLevel.m')
    error(['Please run the code from the top level directory of the ' ...
        'framework i.e where MDOTopLevel.m exists. Also make sure that ' ...
        'the entire code is in your MATLAB path']);
end

% constants
aircraftName = 'FlyingWing';
requiredWindAvailability = .997;
techType = 'shortTerm';

% Optimization variables - No AFS
designID                    = 3;
twists_deg                  = [-4.3087 -9.5907];
ybatteriesobo2              = 5.479E-1;
pitchAxis                   = 2.661E-1;
CL                          = 1.350E0;
tipBoxSkin_nPly             = 0.000E0;
rootUD_nPly                 = 2.000E0;
sparCapWidth                = 2.022E-1;
sparCapCore_nPly            = 3.000E0;
webCore_nPly                = 3.000E0;
AR                          = 2.836E1;
relativeBoomLength          = 0;
tipUD_nPly                  = 2.000E0;
yWingBreakobo2              = 4.973E-1;
MGTOW_kg                    = 3.196E2;
rootBoxSkin_nPly            = 0.000E0;
taperRatios                 = [1.0000 0.3275];
sparCapWidthTaperRatio      = 9.591E-1;
minh_m                      = 2.057E4;
wingToCIndices              = [0.8857 0.2877 0.2933];
VH                          = 0;
percentVerticalSolarPanel   = 4.730E-1;
percentHorizontalSolarPanel = 8.544E-1;
wingLoading_kgm2            = 4.116E0;
propellerCLroot             = 7.493E-1;
propellerCLtip              = 4.314E-1;
propellerRPMmargin          = 1.095E1;
sweep_deg                   = 1.774E1;
latitude_deg                = 2.019E1;
batteryIndex                = 1.000E0;
solarIndex                  = 1.000E0;
techTypeIndex               = 1.000E0;

% Optimization variables - With AFS
designID                    = 3;
twists_deg                  = [-5.1081 -10.6545];
ybatteriesobo2              = 5.457E-1;
pitchAxis                   = 2.542E-1;
CL                          = 1.150E0;
tipBoxSkin_nPly             = 0.000E0;
rootUD_nPly                 = 2.000E0;
sparCapWidth                = 1.772E-1;
sparCapCore_nPly            = 3.000E0;
webCore_nPly                = 4.000E0;
AR                          = 3.159E1;
relativeBoomLength          = 0;
tipUD_nPly                  = 2.000E0;
yWingBreakobo2              = 3.750E-1;
MGTOW_kg                    = 2.898E2;
rootBoxSkin_nPly            = 0.000E0;
taperRatios                 = [1.0000 0.3962];
sparCapWidthTaperRatio      = 8.357E-1;
minh_m                      = 2.057E4;
wingToCIndices              = [0.8285 0.3724 0.3341];
VH                          = 0;
percentVerticalSolarPanel   = 8.510E-1;
percentHorizontalSolarPanel = 8.907E-1;
wingLoading_kgm2            = 3.856E0;
propellerCLroot             = 6.632E-1;
propellerCLtip              = 5.040E-1;
propellerRPMmargin          = 4.590E0;
sweep_deg                   = 2.036E1;
latitude_deg                = 1.957E1;
batteryIndex                = 1.000E0;
solarIndex                  = 1.000E0;
techTypeIndex               = 1.000E0;

tic
[climbMinSOC, maxWindMissionWorstSOC, worstModeDampingRatio, SM, reversalConsistency, tipStallMargin, maxNormalStress, maxShearStress, maxRelBucklingLoad, maxTipDeflection, maxTwist_deg, batteryWeight_kg, relativeSeperationPropCenterBody, outputBatteryEnergyDensity_Wh_kg, outputSolarEff, wingSpeed_ms, divergenceLevel, deltaRollRate, isConverged, optim] = ...
    MDOTopLevel(aircraftName, designID, MGTOW_kg, CL, sweep_deg, VH, latitude_deg, wingLoading_kgm2, minh_m, AR, taperRatios, twists_deg, wingToCIndices, yWingBreakobo2, ybatteriesobo2, relativeBoomLength, ... 
    rootUD_nPly, tipUD_nPly, rootBoxSkin_nPly, tipBoxSkin_nPly, sparCapCore_nPly, webCore_nPly, sparCapWidth, sparCapWidthTaperRatio, pitchAxis, ...
    propellerCLroot, propellerCLtip, propellerRPMmargin, ...
    percentHorizontalSolarPanel, percentVerticalSolarPanel, batteryIndex, solarIndex, techType);
toc

% For debugging
worstModeDampingRatio
SM
reversalConsistency
divergenceLevel

