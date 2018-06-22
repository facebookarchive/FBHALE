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
  
function optim = AerodynamicPerformance(optim)
% Compute aerodynamic performance by first correcting ASWING output with
% XFOIL-based airfoil adbs. The drag polar is then built-up considering
% other drag sources. The minimum power point is compared with the
% wind speed and a target cruise condition is identified. Tip
% stall is checked. Finally, the aswing sectional properties are updated
% to reflect cruise cReynolds number.


% Correct profile drag based on airfoil DB and local cls along span for
% each lifting beam
for b = 1:length(optim.aircraft.beamNames)
    if ~max(strcmp(optim.aircraft.beamNames{b}, {'boom', 'pod'}))
        optim = LiftAndProfileDragCorrection(optim, optim.aircraft.beamNames{b});
    end
end

% Drag build-up for performance calculation
optim = DragBuildUp(optim);

% Find min power CL at various Re and therefore altitude for input to 
% performance code
optim  = MinPowerCL(optim);

% Identify cruise conditions: max TAS between wind and min power conditions
CLwind = 2*optim.MGTOW_kg*optim.constants.g_ms2/(optim.constants.rhoCruise_kgm3*optim.mission.windSpeed_ms^2*optim.aircraft.Sref_m2);
CLminPower = interp1f(optim.aircraft.aero.minPower.h_m, optim.aircraft.aero.minPower.CL, optim.mission.minh_m);

if CLwind < CLminPower
    optim.aircraft.aero.CLcruise = CLwind;
    optim.aircraft.aero.Recruise = optim.aircraft.cref_m * optim.mission.windSpeed_ms / (optim.constants.muCruise_Nsm2/optim.constants.rhoCruise_kgm3);
else
    optim.aircraft.aero.CLcruise = CLminPower;
    optim.aircraft.aero.Recruise = interp1f(optim.aircraft.aero.minPower.h_m, optim.aircraft.aero.minPower.Re, optim.mission.minh_m);
end
optim.aircraft.aero.VcruiseEAS_ms = sqrt(2*optim.MGTOW_kg*optim.constants.g_ms2/optim.aircraft.Sref_m2/optim.constants.rhoSL_kgm3/optim.aircraft.aero.CLcruise);
CLvsAoA = ScaleTimeBulletProof(optim.aircraft.aero.polar.CL, optim.aircraft.aero.polar.Re, optim.aircraft.aero.Recruise);
alfa_deg= optim.aircraft.aero.polar.AoA_deg;
alfa_deg(CLvsAoA<0) = [];
CLvsAoA(CLvsAoA<0)  = [];
optim.aircraft.aero.alphaCruise_deg	= interp1f(CLvsAoA,  alfa_deg, optim.aircraft.aero.CLcruise);
optim.aircraft.aero.CDcruise      	= interp2f(optim.aircraft.aero.polar.AoA_deg, optim.aircraft.aero.polar.Re, optim.aircraft.aero.polar.CD, ...
                                                        optim.aircraft.aero.alphaCruise_deg, optim.aircraft.aero.Recruise); 
optim.aircraft.aero.CL3halfoCDcruise= optim.aircraft.aero.CLcruise^(3/2)/optim.aircraft.aero.CDcruise;
CDi                              	= interp2f(optim.aircraft.aero.polar.AoA_deg, optim.aircraft.aero.polar.Re, optim.aircraft.aero.polar.CDi, ...
                                                     	optim.aircraft.aero.alphaCruise_deg, optim.aircraft.aero.Recruise);
optim.aircraft.aero.ecruise       	= optim.aircraft.aero.CLcruise ^2  / (CDi * pi * optim.wing.AR);

% Update pitching moment of lifting surfaces for aero-elastic effects to be
% captured in aircraft balancing
for b = 1:length(optim.aircraft.beamNames)
    if ~max(strcmp(optim.aircraft.beamNames{b}, {'boom', 'pod'}))
        surface = optim.aircraft.beamNames{b};
        for i = 1:length(optim.(surface).N)
            optim.(surface).aero.CMcruise(i) = interp2f(optim.aircraft.aero.polar.AoA_deg, optim.aircraft.aero.polar.Re, optim.(surface).aero.polar.Cm{i}, ...
                                                        optim.aircraft.aero.alphaCruise_deg, optim.aircraft.aero.Recruise);
            optim.(surface).aero.CDcruise(i) = interp2f(optim.aircraft.aero.polar.AoA_deg, optim.aircraft.aero.polar.Re, optim.(surface).aero.polar.CDp{i}, ...
                                                        optim.aircraft.aero.alphaCruise_deg, optim.aircraft.aero.Recruise);
            if strcmp(optim.aircraft.beamNames{b}, 'wing')
                optim.(surface).aero.CLcruise(i) = interp2f(optim.aircraft.aero.polar.AoA_deg, optim.aircraft.aero.polar.Re, optim.(surface).aero.polar.CL{i}, ...
                                                        optim.aircraft.aero.alphaCruise_deg, optim.aircraft.aero.Recruise);
            end
        end
    end
end

% Assimilate induced drag to wing
optim.wing.aero.CDcruise = optim.wing.aero.CDcruise + interp2f(optim.aircraft.aero.polar.AoA_deg, optim.aircraft.aero.polar.Re, optim.aircraft.aero.polar.CDi, optim.aircraft.aero.alphaCruise_deg, optim.aircraft.aero.Recruise);

% Check for tip stall near cruise
cl           = ScaleTimeBulletProof(optim.wing.aero.ASWINGpolar.clCorrected, optim.aircraft.aero.polar.AoA_deg, optim.aircraft.aero.alphaCruise_deg);
cl           = ScaleTimeBulletProof(reshape(cl, length(optim.aircraft.aero.polar.Re), length(optim.wing.aero.ASWINGpolar.t{1})), optim.aircraft.aero.polar.Re, optim.aircraft.aero.Recruise);
tocs         = interp1f(optim.wing.aero.sectionPolars.t, optim.wing.aero.sectionPolars.toc, optim.wing.aero.ASWINGpolar.t{1});
clMaxDistrib = interp1f(optim.wing.aero.airfoilADB.dim1.vector, optim.wing.aero.aswingPerfAirfoilADB.CLmaxActual, tocs);
clMargin     = cl - clMaxDistrib;
clMarginRoot = clMargin(find(optim.wing.aero.ASWINGpolar.t{1} == 0, 1, 'first'));
clMargin     = clMarginRoot - clMargin ;
optim.wing.aero.tipStallMargin = min(clMargin(abs(optim.wing.aero.ASWINGpolar.t{1}) >= .8)); 

% Update wing twist for cruise conditions - only at end of first iteration.
if ~isfield( optim.wing, 'wasReTwisted')
    
    optim.wing.twists_deg = optim.wing.twists_deg + optim.aircraft.aero.alphaCruise_deg;
    optim.wing.aero.sectionPolars.twists_deg = optim.wing.aero.sectionPolars.twists_deg + optim.aircraft.aero.alphaCruise_deg;
    
    if isfield(optim.wing, 'winglet')
        if optim.wing.winglet.zoc > 0
            optim.wing.aero.sectionPolars.twists_deg(1:2) = 0;
            optim.wing.aero.sectionPolars.twists_deg(end-1:end) = 0;
        end
    end
    
    optim.aircraft.aero.polar.AoA_deg = optim.aircraft.aero.polar.AoA_deg - optim.aircraft.aero.alphaCruise_deg;
    optim.aircraft.aero.alphaCruise_deg = 0;
    optim.wing.wasReTwisted = true;
    
   	% Since wing twist has changed, re-compute normal vectors for solar
    % panelling
    optim.wing.solar = rmfield(optim.wing.solar, 'triangulation');
end

% Update asw db
for b = 1:length(optim.aircraft.beamNames)
    if ~max(strcmp(optim.aircraft.beamNames{b}, {'boom', 'pod'}))
        optim = GenerateASWINGADBs(optim, optim.aircraft.beamNames{b});
    end
end
end
