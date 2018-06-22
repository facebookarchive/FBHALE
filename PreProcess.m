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
  
function optim = PreProcess(params)
% This function prepares the optim structure for optimization. First, 
% design variables are stored. Then basic geometric quantities are derived. 
% Subsequently, configuration-depdendent inputs parameters are populated. 
% Finally, initial guesses and mass properties at the loop onset are 
% computed.

% Create optim structure with optimization variable
optim = AssignDesignVariables(params);

% Set Directories & Paths:
if isdeployed
    optim.ASWINGIODir  = []; 
    optim.CoBladeIODir = []; 
    optim.BoxShapeDir  = '.'; 
    optim.rootDir      = '.';
    optim.propDir      = '.';
    optim.isDeployed   = true;
else     
    optim.ASWINGIODir  = ['ASWing' filesep 'InOutFiles' filesep]; 
    optim.CoBladeIODir = ['StructuralSizing' filesep]; 
    optim.BoxShapeDir  = ['StructuralSizing' filesep 'Airfoil_Data']; 
    optim.propDir      = ['Aerodynamics' filesep 'Propulsion'];
    optim.rootDir      = pwd;
    optim.isDeployed   = false;
    addpath(genpath('Runs')); rmpath(genpath('Runs')); addpath('Runs');
    aircraftInputfolder = ['Runs' filesep optim.aircraftName];
    airfoilADBsfolder = ['Runs' filesep 'airfoilADBs'];
    addpath(genpath(aircraftInputfolder));
    addpath(genpath(airfoilADBsfolder));
end

% Clean up any temp files from prior runs
CleanUp (optim)

% Define universal constants and frequently used quantities
optim.constants.g_ms2       = 9.81;
[rhoSL_kgm3,     muSL_Nsm2]	= GetAtmosphereProperties(0);
optim.constants.rhoSL_kgm3  = rhoSL_kgm3;
optim.constants.muSL_Nsm2   = muSL_Nsm2;
[rhoCruise_kgm3, muCruise_Nsm2]	= GetAtmosphereProperties(optim.mission.minh_m);
optim.constants.rhoCruise_kgm3  = rhoCruise_kgm3;
optim.constants.muCruise_Nsm2   = muCruise_Nsm2;

% Wing area and airspeed
Sref_m2      = optim.MGTOW_kg / optim.wing.wingLoading_kgm2;
windSpeed_ms = sqrt(2 * optim.wing.wingLoading_kgm2 * optim.constants.g_ms2 / optim.constants.rhoCruise_kgm3 / optim.wing.aero.CLsizing);
optim.mission.windSpeed_ms = windSpeed_ms;

% Figure out root chord based on target CL and TAS
bref_m     = sqrt(Sref_m2 * optim.wing.AR);
rootChord_m = fzero(@(rc) optim.wing.AR * trapz(optim.wing.yobo2, rc * optim.wing.taperRatios)/cosd(optim.wing.sweep_deg) - bref_m, 2);

% Wing geometry
optim.wing.c_m        = rootChord_m * optim.wing.taperRatios;
optim.wing.bref_m     = bref_m;
optim.wing.Sref_m2    = Sref_m2;
optim.wing.cref_m     = 2 * trapz(optim.wing.yobo2 * optim.wing.bref_m/2, optim.wing.c_m .^2) / optim.wing.Sref_m2;

% Positions
optim.batteries.y_m  = optim.batteries.yobo2 * optim.wing.bref_m/2;

% Assign airplane-dependent design parameters and initial guesses
designFunctionHandle = str2func('DesignInputs');
optim = designFunctionHandle(optim);
initialGuessFunctionHandle = str2func('InitialGuess');
optim = initialGuessFunctionHandle(optim);
cd (optim.rootDir);

% Aircraft reference quantities
optim.aircraft.Sref_m2 = optim.wing.Sref_m2 * sum(optim.wing.N);
optim.aircraft.cref_m  = optim.wing.cref_m;
optim.aircraft.bref_m  = optim.wing.bref_m;

% Preliminary shaping of fuselage
optim = ShapeFuselage(optim);

% Pre-compute various subsystem weights and provide estimates for battery
% weight
optim 	 = UpdateMassProperties(optim);
Wbatt_kg = (optim.batteries.weighResidual_kg + sum(optim.batteries.mass_kg.*optim.batteries.N));
optim.batteries.mass_kg = Wbatt_kg .* optim.batteries.massDistribution;
optim 	 = UpdateMassProperties(optim);

% Generate ASW dbs for load & performance calculationss
for b = 1:length(optim.aircraft.beamNames)
    if ~max(strcmp(optim.aircraft.beamNames{b}, {'boom', 'pod'}))
        optim = GenerateASWINGADBs(optim, optim.aircraft.beamNames{b});
    end
end

% Find control surface and actuator index used for roll
optim = FindRollControlSurface(optim);
end
