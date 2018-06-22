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
  
function optim = GetHarnessMass(optim)

% Find distances need for calculating harness mass;
fuseBattToWingBatt_m        = abs(optim.batteries.y_m(2));
fuseBattToInboardMotor_m    = abs(optim.propulsion.y_m(1));
inboardMotorToWingBatt_m    = abs(optim.batteries.y_m(2) - optim.propulsion.y_m(1));
outboardMotorToWingBatt_m   = abs(optim.batteries.y_m(2) - optim.propulsion.y_m(2));


%% Wire Masses 
% inboard (Size 14 Pwr and Rtn, 2 STP for data bus, 5 DIO)
inboardWireMass_kg_m    = 2*optim.harness.size14Wire.mass_kg_m * (1 - optim.harness.AlWireWeigthSavings_pct) ...
                        + 2*optim.harness.STPWire.mass_kg_m ...
                        + 5*optim.harness.DIOWire.mass_kg_m ;
% outboard (Size 14 Pwr and Rtn, 2 STP for data bus)                        
outboardWireMass_kg_m   = 2*optim.harness.size14Wire.mass_kg_m * (1 - optim.harness.AlWireWeigthSavings_pct) ...
                        + 2*optim.harness.STPWire.mass_kg_m ;

%% Define Harnesses
% Power & Data -- Outboard Motor Controller to Wing Battery Pack
optim.harness.outboardMotorControllerToWingBatteryPack.length_m     = outboardMotorToWingBatt_m + optim.harness.powerCableRoutingAllowance_m;
optim.harness.outboardMotorControllerToWingBatteryPack.mass_kg      = optim.harness.outboardMotorControllerToWingBatteryPack.length_m * outboardWireMass_kg_m + (optim.harness.plug.mass_kg * 2);
 
% Power & Data -- Wing Battery Pack to Inboard Motor 
optim.harness.inboardMotorControllerToWingBatteryPack.length_m      = inboardMotorToWingBatt_m + optim.harness.powerCableRoutingAllowance_m;
optim.harness.inboardMotorControllerToWingBatteryPack.mass_kg       = optim.harness.outboardMotorControllerToWingBatteryPack.length_m * inboardWireMass_kg_m + (optim.harness.plug.mass_kg * 2);

% POWER & Data - Inboard Motor to Fuselage Battery Packs
optim.harness.inboardMotorControllerToFuselageBatteryPack.length_m  = (fuseBattToInboardMotor_m + optim.harness.powerCableRoutingAllowance_m);
optim.harness.inboardMotorControllerToFuselageBatteryPack.mass_kg   = optim.harness.inboardMotorControllerToFuselageBatteryPack.length_m * inboardWireMass_kg_m + optim.harness.plug.mass_kg * 2;

% Motor Controller to Motor
optim.harness.motorControllerToMotor.length_m    = 0.0 + optim.harness.powerCableRoutingAllowance_m;
optim.harness.motorControllerToMotor.mass_kg     = optim.harness.motorControllerToMotor.length_m * (optim.harness.size8Wire.mass_kg_m * 3 + optim.harness.STPWire.mass_kg_m) + optim.harness.plug.mass_kg * 2;  %% three phase power, one STP tach.

optim.harness.totalPropulsionHarnessMass_kg = (optim.harness.outboardMotorControllerToWingBatteryPack.mass_kg  + optim.harness.inboardMotorControllerToWingBatteryPack.mass_kg  + ...
    optim.harness.inboardMotorControllerToFuselageBatteryPack.mass_kg ) * 2 + optim.harness.motorControllerToMotor.mass_kg * 4;  % two sides

%% Summarize into harness mass and CG
fuselageHarnessMass_kg = optim.harness.inboardMotorControllerToFuselageBatteryPack.mass_kg / 2 * 2; %2x to account for wring to both sides of aircraft

innerMotorHarnessMass_kg = optim.harness.motorControllerToMotor.mass_kg + ...
                         + optim.harness.inboardMotorControllerToFuselageBatteryPack.mass_kg /2 ...
                         + optim.harness.inboardMotorControllerToWingBatteryPack.mass_kg /2;

podHarnessMass_kg = optim.harness.outboardMotorControllerToWingBatteryPack.mass_kg / 2 ...
                  + optim.harness.inboardMotorControllerToWingBatteryPack.mass_kg / 2 ;

outerMotorHarnessMass = optim.harness.outboardMotorControllerToWingBatteryPack.mass_kg / 2 ...
                      + optim.harness.motorControllerToMotor.mass_kg;

optim.harness.mass_kg = [fuselageHarnessMass_kg innerMotorHarnessMass_kg podHarnessMass_kg outerMotorHarnessMass]; 
optim.harness.N       = [1 2 2 2];

end


