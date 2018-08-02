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

function [ optim ] = GetVariableSubSystemPerf(optim)

[batteries, solar] = VariableTechInputs();

baseTechMatrix.solar       = solar.shortTerm;
baseTechMatrix.batteries   = batteries.shortTerm;

%clear battery and solar to avoid potential confusion
batteries = [];
solar = [];

batteryIndex  = optim.batteries.inputSelectionIndex;
solarIndex = optim.solar.inputSelectionIndex;

% Now grab the desired tech and set in inputs
optim.batteries.techName{1}                      = baseTechMatrix.batteries.techName{batteryIndex};
optim.batteries.endOfLifeCapacityLost            = baseTechMatrix.batteries.endOfLifeCapacityLost(batteryIndex);
optim.batteries.packagingWeightCapacityLoss      = baseTechMatrix.batteries.packagingWeightCapacityLoss(batteryIndex);
optim.batteries.specificEnergyDensity_Wh_kg      = baseTechMatrix.batteries.specificEnergyDensity_Wh_kg(batteryIndex);
optim.batteries.roundTripEfficiency              = baseTechMatrix.batteries.roundTripEfficiency(batteryIndex);
optim.batteries.controllerEfficiency             = baseTechMatrix.batteries.controllerEfficiency(batteryIndex);
optim.batteries.minSOC                           = baseTechMatrix.batteries.minSOC(batteryIndex);

optim.solar.techName{1}          = baseTechMatrix.solar.techName{solarIndex};
optim.solar.systemEfficiency     = baseTechMatrix.solar.systemEfficiency(solarIndex);
optim.solar.panelEfficiency      = baseTechMatrix.solar.panelEfficiency(solarIndex);
optim.solar.panelDensity_kg_m2   = baseTechMatrix.solar.panelDensity_kg_m2(solarIndex);

end
