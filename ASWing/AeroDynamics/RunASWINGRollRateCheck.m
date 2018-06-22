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
  
function [ optim ] = RunASWINGRollRateCheck( optim )
% Find aircraft roll rate if ailerons are used for roll. If htail is used
% for roll then find deflection required to meet required roll rate.

%If using the htail for roll set the aero adb to performance to remove
%clmax to ensure we always are able to meet the desired roll rate. 
if strcmp(optim.aircraft.controlSurfaces.roll.surfaceName, 'htail')
    optim.htail.aero.aswingLoadsAirfoilADB.CLmaxActual = optim.htail.aero.aswingLoadsAirfoilADB.CLmax;
	optim.htail.aero.aswingLoadsAirfoilADB.CLminActual = optim.htail.aero.aswingLoadsAirfoilADB.CLmin;
    optim.htail.aero.aswingLoadsAirfoilADB.CLmax = optim.htail.aero.aswingPerfAirfoilADB.CLmax;
	optim.htail.aero.aswingLoadsAirfoilADB.CLmin = optim.htail.aero.aswingPerfAirfoilADB.CLmin;

end
    
% Write .ASW File
WriteASWingFile(optim, [optim.ASWINGIODir 'PolarAircraft' num2str(optim.designID) '.asw'] , 'polarNoSensors', [1 1 1 1], 'LoadsADB'); % using the loads adb for aileron sizing 

% Write ASWING Input String
WriteASWingRollRateCheckInputString( optim )

% Run ASWING
system(['aswing<' optim.ASWINGIODir 'CheckRollRateInputString' num2str(optim.designID) '.txt>' optim.ASWINGIODir 'RollRateRawOutput' num2str(optim.designID) '.txt' ' 2>&1']); 

% Parse roll rates
OmegaXVStall = ParseOmegaX([optim.ASWINGIODir 'RollRateDataVSTALL' num2str(optim.designID) '.txt']);
OmegaXVD     = ParseOmegaX([optim.ASWINGIODir 'RollRateDataVD' num2str(optim.designID) '.txt']);
[F1, F2]     = ParseControlSurfaceDef([optim.ASWINGIODir 'RollRateRawOutput' num2str(optim.designID) '.txt']);

% return minimum normalized roll rate
VNE_ms   = optim.wing.vn.vEasTopRightCorner_ms;
p_VNE    = abs(OmegaXVD)*pi/180*(optim.wing.bref_m/2)/VNE_ms;
Vstall_ms= sqrt(optim.MGTOW_kg * optim.constants.g_ms2 * 2 /(optim.wing.aero.CLmax/1.1  * optim.constants.rhoSL_kgm3 * optim.aircraft.Sref_m2));
p_Vstall = abs(OmegaXVStall)*pi/180*(optim.wing.bref_m/2)/Vstall_ms;

optim.aircraft.controlSurfaces.roll.minRollRate = min(p_VNE, p_Vstall);

% Populate required surface deflection if htails are used for roll
if strcmp(optim.aircraft.controlSurfaces.roll.surfaceName, 'htail')
   optim.htail.controlSurface(1).reqDeflection_deg = max(abs(F1)+abs(F2));
   optim.htail.controlSurface(2).reqDeflection_deg = optim.htail.controlSurface(1).reqDeflection_deg;
   optim.htail.aero.aswingPerfAirfoilADB = optim.htail.aero.aswingLoadsAirfoilADB;
end

% If ADB was changed reset
if strcmp(optim.aircraft.controlSurfaces.roll.surfaceName, 'htail') && isfield(optim.htail.aero.aswingPerfAirfoilADB,'CLmaxActual')
    optim.htail.aero.aswingLoadsAirfoilADB.CLmax = optim.htail.aero.aswingLoadsAirfoilADB.CLmaxActual;
	optim.htail.aero.aswingLoadsAirfoilADB.CLmin = optim.htail.aero.aswingLoadsAirfoilADB.CLminActual;
    optim.htail.aero.aswingPerfAirfoilADB = rmfield(optim.htail.aero.aswingPerfAirfoilADB, {'CLmaxActual', 'CLminActual'});
end

end

