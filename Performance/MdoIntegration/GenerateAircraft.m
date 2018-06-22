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
  
function [ aircraft, environment] = GenerateAircraft( optim )
% This function translates the optim data structure into the set of inputs
% required for the performance code to be run.

%% Airframe Aero
aero.DOF         = 2;

%this is deprecated, this should refrence populated quantities in optim not
%look up tables

aero.CLcruise        =      optim.aircraft.aero.CLcruise;
aero.polar.CD        =      optim.aircraft.aero.polar.CD;
aero.polar.CL        =      optim.aircraft.aero.polar.CL;
aero.polar.alpha_rad =      optim.aircraft.aero.polar.AoA_deg .* pi/180;
aero.polar.Re        =      optim.aircraft.aero.polar.Re;
aero.description     =      optim.wing.aero.description;
aero.Sref_m2         =      optim.aircraft.Sref_m2;
aero.Cref_m          =      optim.aircraft.cref_m; 

aero.minPower.Re        = optim.aircraft.aero.minPower.Re;
aero.minPower.CL        = optim.aircraft.aero.minPower.CL;
aero.minPower.alpha_rad = optim.aircraft.aero.minPower.AoA_deg*pi/180;
aero.minPower.CD        = optim.aircraft.aero.minPower.CD;

aircraft.aero    = aero;

%% Propulsors

    % Number of occurences
    propulsor.N             = sum(optim.propulsion.N);

    % Propeller
    propeller.d_m           = optim.propulsion.propeller.r_m*2;
    propeller.description   = 'variableEfficiency';
    
    propeller.map           = optim.propulsion.propeller.map;
    propeller.invertedMaps  = optim.propulsion.propeller.invertedMaps;
   
    propeller.RPMmax        = optim.propulsion.propeller.RPMmax;
    propeller.Pmax_W        = optim.propulsion.propeller.PMax_W;
    propeller.Qmax_Nm       = optim.propulsion.propeller.Qmax_Nm;
    
    % Motor
    motor.description       = 'constantEfficiency';
    motor.eta               = optim.propulsion.motor.efficiency;
    motor.controller_eta    = optim.propulsion.controller.efficiency;
    
    % Storing
    propulsor.propeller     = propeller;
    propulsor.motor         = motor;
    aircraft.propulsion.propulsor(1) = propulsor;
    aircraft.propulsion.PpropMax_W   = optim.propulsion.PpropMax_W;
    
% Solar cells

    % Solar model setup
    solar.description       = 'perf3cmodel';
    solar.min_solar_angle   = 10;                    % below this angle, clip power to zero 
    solar.mppt_efficiency   = optim.solar.mpptEfficiency;
    solar.mppt_sizingPower_W = optim.solar.mppt.mpptSizingPower_W;
    
    solar.panels.efficiency      = optim.solar.panelEfficiency;
    solar.panels.systemEfficency = optim.solar.systemEfficiency; 
    if optim.solar.length_m > 0.5 || isfield(optim.wing, 'dihedral')
      optim    = DiscretizeSolarPanels(optim);
    end
    solar.panels.normal        = optim.solar.totalSolarfn;
    solar.panels.area_m2       = optim.solar.totalSolarAreaVect_m2; 
    solar.panels.Xb            = optim.solar.totalSolarXb;

    load('absorbed_intensity.mat');
    solar.intensity_power       = intensity_power;
    solar.intensity_altitudes   = intensity_altitudes;
    solar.intensity_zeniths     = intensity_zeniths;
    
    environment.solar           = solar;

    
%% Batteries
batteries.powerMargin         = optim.margin.power_percent;
batteries.roundTripEfficiency = optim.batteries.roundTripEfficiency;
batteries.controllerEfficiency= optim.batteries.controllerEfficiency;
batteries.description         = 'constantSpecificEnergy';
batteries.specificEnergy_kJkg = optim.batteries.specificEnergyDensity_kJkg; % 250 Wh/kg converted to kJ/kg
massProperties.batteries_kg   = sum(optim.batteries.mass_kg.*optim.batteries.N);
batteries.SOCstartClimb       = optim.batteries.SOCstartClimb;
batteries.initialSOC          = optim.batteries.minSOC;
batteries.energyLeft_J        = batteries.initialSOC*batteries.specificEnergy_kJkg * 1e3 * massProperties.batteries_kg;
batteries.capacity_J          = batteries.specificEnergy_kJkg * 1e3 * massProperties.batteries_kg;

% Power draw
batteries.payloadPowerDraw_W    = optim.payload.powerDraw_W;
batteries.avionicPowerDraw_W.climb    = optim.avionics.powerDraw_W.climb;
batteries.avionicPowerDraw_W.maxWind  = optim.avionics.powerDraw_W.maxWind;
batteries.avionicsPowerConversionEff = optim.avionics.powerConversionEfficiency;

aircraft.batteries            = batteries;

%% Harnness
harness.solarToMpptEff     = optim.harness.efficiency.solarToMpptEff;
harness.avionicsHarnessEff = optim.harness.efficiency.avionicsHarnessEff;
harness.payloadHarnessEff  = optim.harness.efficiency.payloadHarnessEff;
harness.mpptToMotorsEff    = optim.harness.efficiency.mpptToMotorsEff;

aircraft.harness           = harness;

%% Mass properties

massProperties.MGTOW_kg       = optim.MGTOW_kg;
massProperties.g_ms2          = optim.constants.g_ms2;
massProperties.MGTOW_N        = massProperties.MGTOW_kg * massProperties.g_ms2;
aircraft.massProperties       = massProperties; 

%delete all the struct that are now sub structs inside of aircraft
clear aero propeller solar batteries massProperties motor propulsor harness

end

