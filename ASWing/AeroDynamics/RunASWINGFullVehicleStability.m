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
  
function [ optim ] = RunASWINGFullVehicleStability( optim )
% This function adds optim.wing.aero.vehicleCnBetaVc, ...
% optim.wing.aero.vehicleCXa_VC, optim.wing.aero.vehicleCZa_VC , ..
% and optim.wing.aero.vehicleCmAlphaVc. 
% Quantiies around deflected Xcg and Zcg.

% Set to fully flexible aircraft structure
useStructure = [1 1 1 1];

% Generate ASWing File
optim = WriteASWingFile(optim, [optim.ASWINGIODir 'PolarAircraft' num2str(optim.designID) '.asw'] , 'polarNoSensors', useStructure);

% Write Input String
optim = WriteFullAircraftStabilityInputString( optim ) ;

% Run
system(['aswing<' optim.ASWINGIODir 'FullVehicleStabilityInputString' num2str(optim.designID) '.txt>' GetNullDevice() ' 2>&1']);

% Get Reference Location and Wing Location
[optim.wing.aero.fullvehicleXref_m, optim.wing.aero.fullvehicleZref_m] ...
    = GetMomentReference([optim.ASWINGIODir 'ForceDerivativesMatrixFullAircraft_Vc' num2str(optim.designID) '.txt']);
optim.wing.aero.fullVehicleWingx_m  = optim.wing.x_m;

% Populate Output Quantities
[optim.wing.aero.vehicleCnBetaVc, ~, optim.wing.aero.vehicleCXa_VC, ...
    optim.wing.aero.vehicleCZa_VC , optim.wing.aero.vehicleCmAlphaVc,~, ~, ~ ] = ...
    ParseCnBeta([optim.ASWINGIODir 'ForceDerivativesMatrixFullAircraft_Vc' num2str(optim.designID) '.txt']);

end

