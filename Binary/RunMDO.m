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
  
function RunMDO (inputFilename)
% a file with objective and constraint data. These input/output files may then be used in an
% optimization framework such as ModeFrontier, ISIGHT, etc. 


%% Set I/O Filenames
[~, designName] = fileparts(inputFilename);
outputFilename  = [designName '.out'];

%% Assign Inputs:
fid = fopen(inputFilename);
C = textscan(fid, '%[^= ]%*[= ]%f %f %f', 'CommentStyle', '%');
fclose(fid);
varNames   = C{1};
varValues1 = C{2};
varValues2 = [C{3}; nan(length(C{1}) - length(C{3}),1)];
varValues3 = [C{4}; nan(length(C{1}) - length(C{4}),1)];

for i = 1:length(varNames)
    eval([varNames{i} '= [' num2str(varValues1(i), '%20.20f') ' ' num2str(varValues2(i), '%20.20f') ' ' num2str(varValues3(i), '%20.20f') '];']);
    eval([varNames{i} '(isnan(' varNames{i} '))= [];']);
end

aircraftName = 'SingleBoom';
techTypes = {'shortTerm', 'longTermHighCostHighPerf', 'longTermHighCostLowPerf', 'longTermLowCostHighPerf','longTermLowCostLowPerf'};
techType  = techTypes{techTypeIndex};

%% Run Framework::

 [deltaClimbMinSOC, deltaMaxWindMissionWorstSOC, worstModeDampingRatio, ...
         SM, reversalConsistency, tipStallMargin, maxRelNormalStress, maxRelShearStress, maxRelBucklingLoad, ...
         maxRelTipDeflection, maxTwist_deg, Wbatt_kg, relativeSeperationPropCenterBody,...
         outputBatteriesEnergyDensity_Wh_kg, outputSolarEff, windSpeed_ms, divergenceLevel,...
         maxRollCSDeltaDeflection_deg, isConverged, optim] = ...
            MDOTopLevel(aircraftName, designID, MGTOW_kg, CL, sweep_deg, VH, latitude_deg, wingLoading_kgm2, minh_m, AR, taperRatios, twists_deg, wingToCIndices, yWingBreakobo2, ybatteriesobo2, relativeBoomLength, ...
           	rootUD_nPly, tipUD_nPly, rootBoxSkin_nPly, tipBoxSkin_nPly, sparCapCore_nPly, webCore_nPly, sparCapWidth, sparCapWidthTaperRatio, pitchAxis, ...
            propellerCLroot, propellerCLtip, propellerRPMmargin, ...
            percentHorizontalSolarPanel, percentVerticalSolarPanel, batteryIndex, solarIndex, techType);


% Check Airspeed is high enough for specified latitude
deltaAirspeed_ms = getDeltaAirspeed(optim, latitude_deg);
        
%% Write Outputs::

outVarsStruct = v2struct(deltaClimbMinSOC, deltaMaxWindMissionWorstSOC, worstModeDampingRatio, SM, reversalConsistency, tipStallMargin, maxRelNormalStress, ... 
         maxRelShearStress, maxRelBucklingLoad, maxRelTipDeflection, maxTwist_deg, Wbatt_kg, relativeSeperationPropCenterBody,...
         outputBatteriesEnergyDensity_Wh_kg, outputSolarEff, windSpeed_ms, divergenceLevel, maxRollCSDeltaDeflection_deg, deltaAirspeed_ms, isConverged);
outputVarNames = fieldnames(outVarsStruct);

fid = fopen(outputFilename,'w');
for i = 1:length(outputVarNames)
C = fprintf(fid, '%-35s = %f\n',outputVarNames{i},outVarsStruct.(outputVarNames{i}));
end
fclose(fid);















