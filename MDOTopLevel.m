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
function [deltaClimbMinSOC, deltaMaxWindMissionWorstSOC, worstModeDampingRatio, ...
         SM, reversalConsistency, tipStallMargin, maxRelNormalStress, maxRelShearStress, maxRelBucklingLoad, ...
         maxRelTipDeflection, maxTwist_deg, Wbatt_kg, relativeSeperationPropCenterBody,...
         outputBatteriesEnergyDensity_Wh_kg, outputSolarEff, windSpeed_ms, divergenceLevel,...
         maxRollCSDeltaDeflection_deg, isConverged, optim] = ...
            MDOTopLevel(aircraftName, designID, MGTOW_kg, CL, sweep_deg, VH, latitude_deg, wingLoading_kgm2, minh_m, AR, taperRatios, twists_deg, wingToCIndices, yWingBreakobo2, ybatteriesobo2, relativeBoomLength, ...
           	rootUD_nPly, tipUD_nPly, rootBoxSkin_nPly, tipBoxSkin_nPly, sparCapCore_nPly, webCore_nPly, sparCapWidth, sparCapWidthTaperRatio, pitchAxis, ...
            propellerCLroot, propellerCLtip, propellerRPMmargin, ...
            percentHorizontalSolarPanel, percentVerticalSolarPanel, batterySelectionIndex, solarSelectionIndex, techType)
% Top level function that runs the design loop. 
% Making use of the combined input parameters and optimization
% variables, it assembles the aircraft design point according to design
% rules for each subsystem. Design parameters are aircraft dependent and
% set in a user created folder in 'AircraftInputs'. The folder name is 
% passed in the 'aircraftName' variable. 
% Once weight and CG are converged, mission performance is computed, 
% stresses are calculated as well as aero-structural modes. 
% This function would be the one interfacing an optimization routine.
%
% 'optim' is the global structure that is used to record the state of the
% airplane as it is being sized.

% Pack params
params = v2struct;

% Prepare optim structure by 1) storing design variables and
% parameters, and 2) initial guesses
disp(['  Assigning variables and computing initial geometrical quantities']);
optim = PreProcess(params);

% Initialize aerodynamic data and generate Vn diagram
optim = InitialWingAerodynamicPerformance(optim);
optim = GenerateVn(optim);
    
% Setup loop
tolW     = 5e-3;
tolCG    = 3e-2;
Wbatt_kg = sum(optim.batteries.mass_kg.*optim.batteries.N);
iter     = 0;
isWeightConverged = false;
isCGConverged     = false;

while ~isWeightConverged || ~isCGConverged || iter <= 2
    
    % Protect against too many iterations
    if iter == 6
        warning('Could not converge within allocated number of iterations');
        break;
    end
    
    iter = iter + 1;
    disp(' ');
    disp(['Iteration ' num2str(iter)]);
    disp('------------------------------------------------------------------');
    optim.batteries.mass_kg = Wbatt_kg;
    xCG0_m = optim.xCG_m;
    
    % Compute new battery weight - relax
    if iter > 2
        signdWdb = sign(dWdB);
        dWdB = max(.5,  abs(dWdB));
        dWdB = min(1.5, abs(dWdB));
        dWdB = signdWdb * dWdB;
        Wbatt_kg = Wbatt_kg - optim.batteries.weighResidual_kg / dWdB;
    elseif iter == 2
        Wbatt_kg = Wbatt_kg + optim.batteries.weighResidual_kg;
    end
    optim.batteries.mass_kg = Wbatt_kg .* optim.batteries.massDistribution;
    
    % Update tail sizes and wing position to balance the aircraft
    optim = SizeAndBalance(optim, iter);
    
	% Evaluate gradient
    if iter > 1
        % If the past iteration was converged weight-wise, dWdB will not be
        % evaluated correctly
        dWdB = (optim.batteries.weighResidual_kg - weighResidualOld_kg)/(Wbatt_kg - WbattOld_kg);
    end
    
    % Convergence criteria
    isWeightConverged   = abs(optim.batteries.weighResidual_kg/optim.MGTOW_kg)	< tolW;
    isCGConverged       = abs(optim.xCG_m - xCG0_m)/optim.wing.cref_m           < tolCG;
    weighResidualOld_kg = optim.batteries.weighResidual_kg;
 	WbattOld_kg         = Wbatt_kg;
    
    disp([' ']);
    disp(['     Weight residual  : ' num2str(100 * optim.batteries.weighResidual_kg/optim.MGTOW_kg) '% of MGTOW vs ' num2str(100 * tolW) '% criterion']);
    disp(['     CG movement      : ' num2str(100 * abs(optim.xCG_m - xCG0_m)/optim.wing.cref_m) '% chord vs ' num2str(100 * tolCG) '% criterion']);
end
isConverged = isWeightConverged && isCGConverged;
disp('------------------------------------------------------------------');
disp(['Converged in ' num2str(iter) ' iterations']);
disp(['']);

% Ensure weight and polar convergence
optim.batteries.mass_kg = sum(optim.batteries.mass_kg.*optim.batteries.N) + optim.batteries.weighResidual_kg;
optim.batteries.mass_kg = optim.batteries.mass_kg .* optim.batteries.massDistribution;
optim.batteries.weighResidual_kg = 0;
Wbatt_kg                         = sum(optim.batteries.mass_kg.*optim.batteries.N);
disp('  Running polar sweep...');
optim = RunASWINGPolar(optim, [1,1,1,1]);
optim = AerodynamicPerformance(optim);

% Calculate Wing Stresses And Deflections
disp('  Running loads & checking stresses/deflections...');
[maxRelNormalStress, maxRelTransverseStress, maxRelShearStress, maxRelBucklingLoad, maxRelTipDeflection, maxTwist_deg, optim]  = ...
    StressesAndDeflections(optim);

% Aero performance report
disp('Aerodynamic performance');
disp('------------------------------------------------------------------');
disp(['Endurance parameter  : ' num2str(optim.aircraft.aero.CL3halfoCDcruise)            ]);
disp(['Angle of attack      : ' num2str(optim.aircraft.aero.alphaCruise_deg) ' deg' ]);
disp(['Cruise Re            : ' num2str(optim.aircraft.aero.Recruise)               ]);
disp('------------------------------------------------------------------');

% Output data
reversalConsistency = optim.aircraft.controlSurfaces.roll.isConsistent; % Positive if no-reversal
SM                  = optim.aircraft.SM;
tipStallMargin      = optim.wing.aero.tipStallMargin;

% Run dynamic modes 
disp('Checking modes...');
optim = RunASWINGDynamicModes(optim, [1,1,1,1]);
worstModeDampingRatio = optim.wing.eigenModes.minDampingRatio;

% Run divergence check
disp('Checking torsional divergence...');
optim = RunASWINGDivergence( optim, [1 1 1 1]);
divergenceLevel = optim.wing.divergenceLevel;

% If battery weight is negative, don't run the performance code
disp('------------------------------------------------------------------');
disp('Mission performance');
disp('------------------------------------------------------------------');
if optim.batteries.mass_kg <= 0
    minimumSOC      = -1 * [1; 1; 1];
else
    % Run aircraft through performance code worst case wind at winter
    % solstice
    [minimumSOC, optim] = RunPerfCode(optim);
end
deltaMaxWindMissionWorstSOC  = minimumSOC-optim.batteries.minSOC;
deltaClimbMinSOC             = 1 - optim.mission.climb.totalClimbEnergy_J/(optim.batteries.specificEnergyDensity_kJkg*1e3*Wbatt_kg) - optim.batteries.minSOC;
windSpeed_ms                 = optim.mission.windSpeed_ms;
disp(['Delta end of climb SOC    : ' num2str(deltaClimbMinSOC*100) ' %']);
disp(['Delta end of solstice SOC : ' num2str(deltaMaxWindMissionWorstSOC*100) ' %']);

% Calculate center body prop separation
if optim.propulsion.N == 1
    relativeSeperationPropCenterBody = 2;
else 
    relativeSeperationPropCenterBody = optim.propulsion.y_m(1)/(optim.propulsion.propeller.r_m*2);
end

% Save ASWing file for user check
if ~isdeployed % If running in compiled mode, don't record the airplane
    if ~exist(['.' filesep 'ASWing' filesep 'Aircraft'])
        mkdir(['.' filesep 'ASWing' filesep 'Aircraft'])
    end
    WriteASWingFile(optim, ['ASWing' filesep 'Aircraft' filesep num2str(optim.designID) '.asw'] , 'polarNoSensors', [1 1 1 1]);
end

% Output tech used
outputBatteriesEnergyDensity_Wh_kg = optim.batteries.specificEnergyDensity_Wh_kg;
outputSolarEff = optim.solar.panelEfficiency;

% If roll power is provided by another surface but the wing, check that the
% required deflection does not exceed the maximum one
if isfield(optim.(optim.aircraft.controlSurfaces.roll.surfaceName).controlSurface, 'reqDeflection_deg')
    maxRollCSDeltaDeflection_deg = max([optim.(optim.aircraft.controlSurfaces.roll.surfaceName).controlSurface.reqDeflection_deg] - ...
                                       [optim.(optim.aircraft.controlSurfaces.roll.surfaceName).controlSurface.maxDeflection_deg]);
else
    maxRollCSDeltaDeflection_deg = 0;
end

% Clean up temp files (ASWING, Structures)
CleanUp(optim);
end
