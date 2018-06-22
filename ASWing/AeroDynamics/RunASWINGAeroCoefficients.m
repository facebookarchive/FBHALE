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
  
function [ optim ] = RunASWINGAeroCoefficients( optim)
%Runs wing and fuselage ASWing to get cnbeta, Cmbeta ect. and checks for
%aileron reversal.

%% Write .ASW Aircraft input files
% Write two files one using aerocoeficient adb and one using loads ADB. 
optim = WriteASWingFile(optim, [optim.ASWINGIODir 'CnBetaAircraft' num2str(optim.designID) '.asw' ], 'AeroCoefficients', [1 1 1 1], 'AeroCoefficients');
optim = WriteASWingFile(optim, [optim.ASWINGIODir 'CnBetaAircraftWithTails' num2str(optim.designID) '.asw' ], 'LoadsADB', [1 1 1 1], 'LoadsADB');

%% Write ASWing input string 
% find alpha vectors
optim.wing.aero.alpha_deg_VC    = 0;
diveCL                          = optim.MGTOW_kg*optim.constants.g_ms2 *2 / (optim.constants.rhoSL_kgm3 *optim.wing.vn.vEasTopRightCorner_ms^2 * optim.wing.Sref_m2);
optim.wing.aero.alpha_deg_VD    = optim.wing.aero.alpha0_deg+(diveCL/optim.wing.aero.CLalpha)*180/pi;

WriteAeroCoefficientsInputString(optim);

%% Run ASWING
system(['aswing<' optim.ASWINGIODir 'CnBetaInputString' num2str(optim.designID) '.txt>' GetNullDevice() ' 2>&1']);

%% Parse Outputs
[optim.wing.aero.CnBeta_VC, optim.wing.aero.CYBeta_VC, optim.wing.aero.CXa_VC, optim.wing.aero.CZa_VC, optim.wing.aero.CMa_VC, ...
    optim.wing.aero.CX_VC, optim.wing.aero.Cm_VC, optim.wing.aero.CZ_VC ] = ParseCnBeta([optim.ASWINGIODir 'ForceDerivativesMatrix_Vc' num2str(optim.designID) '.txt']);
[optim.wing.aero.CnBeta_VD, optim.wing.aero.CYBeta_VD, optim.wing.aero.CXa_VD, optim.wing.aero.CZa_VD, optim.wing.aero.CMa_VD, ...
     optim.wing.aero.CX_VD, optim.wing.aero.Cm_VD, optim.wing.aero.CZ_VD] = ParseCnBeta([optim.ASWINGIODir 'ForceDerivativesMatrix_Vd' num2str(optim.designID) '.txt']);
 
 % Adverse yaw 
[optim.wing.aero.CnBetaDeltaMax_VS, optim.wing.aero.CnDeltaMax_VS, ...
    optim.wing.aero.CYBetaDeltaMax_VS, optim.wing.aero.CYDeltaMax_VS] = ...
    ParseDeltaMaxCn([optim.ASWINGIODir 'ForceDerivativesMatrix_VS_AileronDeflected' num2str(optim.designID) '.txt']);
[optim.wing.aero.CnBetaDeltaMax_VD, optim.wing.aero.CnDeltaMax_VD, ...
    optim.wing.aero.CYBetaDeltaMax_VD, optim.wing.aero.CYDeltaMax_VD] = ...
    ParseDeltaMaxCn([optim.ASWINGIODir 'ForceDerivativesMatrix_VD_AileronDeflected' num2str(optim.designID) '.txt']);

% Record reference location 
optim.wing.aero.Yref_m = 0;
optim.wing.aero.Zref_m = 0;
optim.wing.aero.CnBetaxCGLocation_m = optim.xCG_m;
optim.wing.aero.CnBetaInitialWingXLocatiom_m = optim.wing.x_m;

% Check for aileron reversal
% returns positive number if both ailerons reversal state is consistent at
% Vs and Vd
OmegaX_VS = ParseOmegaX([optim.ASWINGIODir 'ForceDerivativesMatrix_VS_AileronDeflected' num2str(optim.designID) '.txt']);
OmegaX_VD = ParseOmegaX([optim.ASWINGIODir 'ForceDerivativesMatrix_VD_AileronDeflected' num2str(optim.designID) '.txt']);

optim.aircraft.controlSurfaces.roll.isConsistent = OmegaX_VS*OmegaX_VD; 

% Find deflected CG position
[optim.xCGDef_m, optim.zCGDef_m] = GetMomentReference([optim.ASWINGIODir 'ForceDerivativesMatrix_Vc' num2str(optim.designID) '.txt']);

end

