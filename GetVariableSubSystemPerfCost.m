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
  
function [ optim ] = GetVariableSubSystemPerfCost(optim, inputBatteryTechIndex, inputSolarTechIndex)

[batteries, solar, costModel] = VariableTechInputs();

if strcmp(optim.techOperatingMode, 'shortTerm')
   baseTechMatrix.solar       = solar.shortTerm;
   baseTechMatrix.batteries   = batteries.shortTerm;
   baseTechMatrix.costModel   = costModel.shortTerm;
   
elseif strcmp(optim.techOperatingMode, 'longTermHighCostHighPerf')
    baseTechMatrix.solar        = solar.longTerm.highCostHighPerf;
    desiredBatteryTechIndex     = find(strcmp(batteries.longTerm.techName, 'longTermHighCostHighPerf'));
    baseTechMatrix.costModel    = costModel.longTerm;
    
elseif strcmp(optim.techOperatingMode, 'longTermHighCostLowPerf')
    baseTechMatrix.solar        = solar.longTerm.highCostLowPerf;
    desiredBatteryTechIndex     = find(strcmp(batteries.longTerm.techName, 'longTermHighCostLowPerf')); 
    baseTechMatrix.costModel    = costModel.longTerm;
     
elseif strcmp(optim.techOperatingMode, 'longTermLowCostHighPerf')
    baseTechMatrix.solar        = solar.longTerm.lowCostHighPerf;
    desiredBatteryTechIndex     = find(strcmp(batteries.longTerm.techName, 'longTermLowCostHighPerf'));
    baseTechMatrix.costModel    = costModel.longTerm;
    
elseif strcmp(optim.techOperatingMode, 'longTermLowCostLowPerf') 
    baseTechMatrix.solar        = solar.longTerm.lowCostLowPerf;
    desiredBatteryTechIndex     = find(strcmp(batteries.longTerm.techName, 'longTermLowCostLowPerf'));
    baseTechMatrix.costModel    = costModel.longTerm;

elseif strcmp(optim.techOperatingMode, 'test') 
    baseTechMatrix.solar        = solar.test;
    baseTechMatrix.batteries    = batteries.test;
    baseTechMatrix.costModel    = costModel.test;
else
    error('Unknown tech input set specified')
end
  
% If needed set baseTech battery matrix
if exist('desiredBatteryTechIndex')
   baseTechMatrix.batteries.techName{1}                      = batteries.longTerm.techName{desiredBatteryTechIndex};
   baseTechMatrix.batteries.endOfLifeCapacityLost            = batteries.longTerm.endOfLifeCapacityLost(desiredBatteryTechIndex);  
   baseTechMatrix.batteries.packagingWeightCapacityLoss      = batteries.longTerm.packagingWeightCapacityLoss(desiredBatteryTechIndex);
   baseTechMatrix.batteries.specificEnergyDensity_Wh_kg      = batteries.longTerm.specificEnergyDensity_Wh_kg(desiredBatteryTechIndex);
   baseTechMatrix.batteries.roundTripEfficiency              = batteries.longTerm.roundTripEfficiency(desiredBatteryTechIndex);
   baseTechMatrix.batteries.controllerEfficiency             = batteries.longTerm.controllerEfficiency(desiredBatteryTechIndex);
   baseTechMatrix.batteries.minSOC                           = batteries.longTerm.minSOC(desiredBatteryTechIndex);
   baseTechMatrix.batteries.batteryCost_kWh                  = batteries.longTerm.batteryCost_kWh(desiredBatteryTechIndex);       
end

%clear battery and solar to avoid potential confusion
batteries = [];
solar = [];
costModel = [];

% Now sort output baseTech base on described order
[~, batterySortVect] =  sort((baseTechMatrix.batteries.specificEnergyDensity_Wh_kg .* baseTechMatrix.batteries.packagingWeightCapacityLoss .* ...
    (1-baseTechMatrix.batteries.minSOC)) ./ baseTechMatrix.batteries.batteryCost_kWh);
[~, solarSortVect]   =  sort(baseTechMatrix.solar.panelEfficiency .* baseTechMatrix.solar.systemEfficiency ./ baseTechMatrix.solar.panelDensity_kg_m2 ./ baseTechMatrix.solar.cost_m2); 

batteryIndex  = find(batterySortVect == optim.batteries.inputSelectionIndex );
solarIndex = find(solarSortVect == optim.solar.inputSelectionIndex );

% Now grab the desired tech and set in inputs
optim.batteries.techName{1}                      = baseTechMatrix.batteries.techName{batteryIndex}; 
optim.batteries.endOfLifeCapacityLost            = baseTechMatrix.batteries.endOfLifeCapacityLost(batteryIndex);  
optim.batteries.packagingWeightCapacityLoss      = baseTechMatrix.batteries.packagingWeightCapacityLoss(batteryIndex);
optim.batteries.specificEnergyDensity_Wh_kg      = baseTechMatrix.batteries.specificEnergyDensity_Wh_kg(batteryIndex);
optim.batteries.roundTripEfficiency              = baseTechMatrix.batteries.roundTripEfficiency(batteryIndex);
optim.batteries.controllerEfficiency             = baseTechMatrix.batteries.controllerEfficiency(batteryIndex);   
optim.batteries.minSOC                           = baseTechMatrix.batteries.minSOC(batteryIndex);   
optim.costModel.batteryCost_kWh                  = baseTechMatrix.batteries.batteryCost_kWh (batteryIndex);

optim.solar.techName{1}          = baseTechMatrix.solar.techName{solarIndex}; 
optim.solar.systemEfficiency     = baseTechMatrix.solar.systemEfficiency(solarIndex); 
optim.solar.panelEfficiency      = baseTechMatrix.solar.panelEfficiency(solarIndex);
optim.solar.panelDensity_kg_m2   = baseTechMatrix.solar.panelDensity_kg_m2(solarIndex); 
optim.costModel.solarCellCost_m2 = baseTechMatrix.solar.cost_m2(solarIndex);
   
%optim.costModel.batteryMultiplier   = baseTechMatrix.costModel.batteryMultiplier;
optim.costModel.structuralCost_kg   = baseTechMatrix.costModel.structuralCost_kg;
optim.costModel.avionicsFixedCost   = baseTechMatrix.costModel.avionicsFixedCost;
optim.costModel.payloadCost         = baseTechMatrix.costModel.payloadCost;
optim.costModel.landingsPerCrash    = baseTechMatrix.costModel.landingsPerCrash;
optim.costModel.studyDuration_years = baseTechMatrix.costModel.studyDuration_years;
optim.costModel.batteriesPerYear    = baseTechMatrix.costModel.batteriesPerYear;

end
