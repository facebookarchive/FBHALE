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
  
function [optim] = SubsystemInputs(optim)
%==========================================================================
% Power collection and storage
%==========================================================================

% MPPT
optim.solar.mppt.kg_kW             	= 0.45; 
optim.solar.mpptEfficiency          = 0.97;

% Batteries
optim.batteries.N                            = [1 2];
optim.batteries.specificEnergyDensity_kJkg   = optim.batteries.specificEnergyDensity_Wh_kg * 3.6 * (1-optim.batteries.endOfLifeCapacityLost) * optim.batteries.packagingWeightCapacityLoss; 
optim.batteries.SOCstartClimb                = 1; 

% Solar Cell Properties
optim.solar.surfacesWithSolar                = {'wing','htail','vtail'};

% Specify cell size                                                      
optim.solar.width_m   = 0.017;
optim.solar.length_m  = 0.05;
optim.solar.module.chordwiseMaxPanels 	= 3;
optim.solar.module.chordwiseMinPanels 	= 5;
optim.solar.module.maxNPanels 		 	= 125;
optim.solar.module.minNPanels 			= 75;

% Set battery mass distribution based on configuration
optim.batteries.massDistribution = [1/3 1/3];
optim.batteries.xCG_m = [0 .5];

% Set number of solar point masses for aswing files - specified number is
% per half span. 
optim.wing.solar.nRepresentativePointMasses  = 2;

% Discretization of solar masses along lifting surfaces
optim.wing.solar.N 	 = [2 2];
optim.htail.solar.N  = 1;
optim.vtail.solar.N  = 1;

% htail and vtail solar mass are assumed to be at z = 0, wing solar
% location is calcualted in the loop.
optim.htail.solar.z_m = 0;
optim.vtail.solar.z_m = 0;

%==========================================================================
% Propulsion
%==========================================================================
optim.propulsion.propeller.cruiseDiskLoading_Nm2 = 4;
optim.propulsion.propeller.Nb                    = 2;
optim.propulsion.propeller.airfoilName           = 'sa7024';
optim.propulsion.propeller.hubRelativeRadius     = .2;
optim.propulsion.propeller.tipMach             = 0.5;
optim.propulsion.motor.efficiency              = 0.96;
optim.propulsion.controller.efficiency         = 0.975;
optim.propulsion.propulsionNonMotorComponentsRelativeMass    = 1.5116;
optim.propulsion.kgW                           = 0.00173155;
optim.propulsion.relativeClimbRateMotorOut      = .70; % achieve X% of the nominal climb rate when one motor out

% Alternative sizing: use a known relationship for torque vs weight of the
% motor, then add controller weight plus the remaining
optim.propulsion.torqueWeightConstants = [2.35 0.29];

%==========================================================================
% Landing gear
%==========================================================================
optim.landingGear.N         = [1 2];
optim.landingGear.y_m       = 0;
optim.landingGear.mass_kg	= .02*optim.MGTOW_kg/sum(optim.landingGear.N) * ones(1, length(optim.landingGear.N));

%==========================================================================
% Avionics & Payload
%==========================================================================

% Avionics
% Find configuration dependent avionics mass
optim.avionics.N	= 1;
optim.avionics.mass_kg = 5.1*optim.MGTOW_kg/550 + 12.1*(optim.wing.bref_m/70) + 10.7; %servo, wiring, computers
optim.avionics.powerDraw_W.climb           = 235.5;
optim.avionics.powerDraw_W.maxWind         = 211;
optim.avionics.y_m                         = 0;
optim.avionics.z_m                         = 0;
optim.avionics.powerConversionEfficiency   = .90;

% Payload
optim.payload.N                            = 1;
optim.payload.mass_kg                      = 10;  
optim.payload.powerDraw_W 	               = 150; 
optim.payload.y_m                          = 0;
optim.payload.z_m                          = 0;

%==========================================================================
% Power Transmission
%==========================================================================
optim.harness.N                             = 1;
optim.harness.efficiency.solarToMpptEff     = 0.995;
optim.harness.efficiency.avionicsHarnessEff = 0.995;
optim.harness.efficiency.payloadHarnessEff  = 0.995;
optim.harness.efficiency.mpptToMotorsEff    = 0.995;

optim.harness.size16Wire.mass_kg_m          = 13.0/1000;   	% M22759/43-16
optim.harness.size16Wire.ampicity_amps      = 12.0;         % Derated for 80kft altitude and 30 degC temp rise
optim.harness.size16Wire.impedance_Ohm_m    = 14.8/1000;    

optim.harness.size14Wire.mass_kg_m          = 20.5/1000;
optim.harness.size14Wire.ampicity_amps      = 15.8;         % Derated for 80kft altitude and 30 degC temp rise
optim.harness.size14Wire.impedance_Ohm_m    = 9.45/1000;

optim.harness.size8Wire.mass_kg_m           = 93.0/1000;

optim.harness.DIOWire.mass_kg_m             = 2.5/1000;
optim.harness.STPWire.mass_kg_m             = 27.7/1000;

optim.harness.powerCableRoutingAllowance_m  = 0.5;
optim.harness.AlWireWeigthSavings_pct       = 0.0;          % set to 0.15 if using Al wires.
optim.harness.plug.mass_kg                  = 0.03/1000;    % 16 grams plug for a size 13 composite 38999 connector.  For estimate purpose only

end

