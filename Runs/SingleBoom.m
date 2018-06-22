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

% Check for required codes to be in the path
isReady = CheckSystem();
if ~isReady
    error('Please add required codes to your path');
end

% Check local directory
fileList = dir('.');
if ~contains([fileList.name], 'MDOTopLevel.m')
    error(['Please run the code from the top level directory of the ' ...
        'framework i.e where MDOTopLevel.m exists. Also make sure that ' ...
        'the entire code is in your MATLAB path']);
end

% Constants
aircraftName = 'SingleBoom';
requiredWindAvailability = .997;
techType = 'shortTerm';

% Optimization variables
designID					=   1;
ybatteriesobo2    	 		=   5.072E-1;
rootUD_nPly  				=	3.000E0;
twists_deg					=	[-0.9400 -1.9300];
rootBoxSkin_nPly			=	0.000E0;
taperRatios					=	[1.0000 0.3266];
pitchAxis					=	2.523E-1;
latitude_deg				=	1.982E1;
relativeBoomLength			=	2.197E-1;
tipUD_nPly					=   1.000E0;
CL							=	1.357E0;
percentHorizontalSolarPanel	=	9.016E-1;
sparCapWidth				=	1.738E-1;
sparCapCore_nPly            =   4.000E0;
webCore_nPly                =   4.000E0;
percentVerticalSolarPanel	=	3.106E-1;
AR							=	2.902E1;
tipBoxSkin_nPly 			=	0.000E0;
wingToCIndices				=	[0.6980 0.2404 0.0675];
yWingBreakobo2				=	5.665E-1;
MGTOW_kg					=	3.211E2;
MGTOW_kg					=	3.20E2;
wingLoading_kgm2			=	4.460E0;
sparCapWidthTaperRatio		=	6.134E-1;
VH							=	2.248E-1;
minh_m						=	2.057E4;
batteryIndex         		=	1.000E0;
solarIndex					=   1.000E0;
techTypeIndex			    =   1.000E0;
propellerCLroot             =   5.407E-1;
propellerCLtip              =   7.202E-1;
propellerRPMmargin          =   6.170E0;
sweep_deg                   =   0.000E0;

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

