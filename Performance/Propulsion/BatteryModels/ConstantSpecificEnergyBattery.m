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
  
function batteries = ConstantSpecificEnergyBattery(aircraft)
% This function represents the battery as an energy reservoir set by the
% mass and specific energy density. The state of charge SOC is tracked by
% the state integrator.

batteries  = aircraft.batteries;
propulsion = aircraft.propulsion;
harness    = aircraft.harness;

% Figure out how much power is drawn and received
Pe_W = 0;

% Loop through propulsors to find total power draw
for p = 1:length(propulsion.propulsor)
    Pe_W = Pe_W + propulsion.propulsor(p).motor.electricalPower_W / harness.mpptToMotorsEff;
end
batteries.PeProp_W    = Pe_W;

% Add in extra power draw e.g. avionics and payload
Pe_W = Pe_W + batteries.payloadPowerDraw_W / harness.payloadHarnessEff + ...
   batteries.avionicPowerDraw_W / batteries.avionicsPowerConversionEff / harness.avionicsHarnessEff;
Pe_W 			  	  = Pe_W * (1 + batteries.powerMargin);  % adding power margin
batteries.powerDraw_W = Pe_W; 
batteries.Pextra_W    = (batteries.payloadPowerDraw_W + batteries.avionicPowerDraw_W) * (1 + batteries.powerMargin);

% If the net power is negative (i.e receiving), charge batteries. Book-keep
% battery RTE on the charging leg
if Pe_W - batteries.solarPower_W <= 0 
    PeBatt_W = batteries.roundTripEfficiency * batteries.controllerEfficiency * (Pe_W - batteries.solarPower_W * harness.solarToMpptEff);
else
    PeBatt_W = 1/batteries.controllerEfficiency * (Pe_W - batteries.solarPower_W * harness.solarToMpptEff);
end

batteries.SOC = batteries.energyLeft_J/batteries.capacity_J;

% Pull the resulting power
batteries.Pe_W      = PeBatt_W;

% Store
batteries.PeBatt_W	= PeBatt_W;
batteries.PeSolar_W  = batteries.solarPower_W;

end
