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
  
function optim = GenerateASWINGADBs(optim, surface)
% This routine computes the sectional aerodynamic performance quantities
% required to run ASWING at a cruise conditions (performance) and sea-level
% (loads conditions).
% 
% ASWING requires the following quantities at each section:
% Cdf   : section profile friction drag coefficient
% Cdp   : section profile pressure drag coefficient
% alpha : angle of zero-lift line above c-axis
% Cm    : section pitching moment coefficient about chord/4
% CLmax : section maximum lift coefficient
% CLmin : section minimum lift coefficient
% dCLda : section lift-curve slope
% dCLdF1 : dCL/dFlap1 derivative ("flap" is in any units)
% dCMdF1 : dCM/dFlap1 derivative
% dCDdF1 : dCDp/dFlap1 derivative

% Based on which surface is selected, use the appropriate control surface,
% chord, and span
chords          = {'cref_m',    'c_m',      'c_m'       };

switch surface
    case 'wing'
        s = 1;
    case 'htail'
        s = 2;
    case 'vtail'
        s = 3;
end

% Find Re for surface of interest at SL and cruise altitudes
if ~isfield(optim.aircraft.aero, 'CLcruise')
    VTAS_ms = sqrt(2*optim.MGTOW_kg * optim.constants.g_ms2 / (optim.constants.rhoCruise_kgm3 * optim.aircraft.Sref_m2 * optim.wing.aero.CLcruise)) * [sqrt(optim.constants.rhoCruise_kgm3/optim.constants.rhoSL_kgm3); 1];
else
      VTAS_ms = sqrt(2*optim.MGTOW_kg * optim.constants.g_ms2 / (optim.constants.rhoCruise_kgm3 * optim.aircraft.Sref_m2 * optim.aircraft.aero.CLcruise)) * [sqrt(optim.constants.rhoCruise_kgm3/optim.constants.rhoSL_kgm3); 1];
end
Re      = VTAS_ms .* optim.(surface).(chords{s})(1) ./ ([optim.constants.muSL_Nsm2; optim.constants.muCruise_Nsm2]./[optim.constants.rhoSL_kgm3; optim.constants.rhoCruise_kgm3]);

% If the wing is the surface of interest store Re as it will be used again
if s == 1
    optim.wing.aero.ReSL = Re(1);
    optim.wing.aero.ReCruise = Re(2);
end

if ~isfield(optim.(surface), 'controlSurface')
    Nc = 1;
    csOcs = 1;
else
    Nc = length(optim.(surface).controlSurface);
    csOcs = [optim.(surface).controlSurface(:).csoc];
end


% If section polars haven't been populated yet, do so now
if ~isfield(optim.(surface).aero, 'sectionPolars')
    optim = PopulateSectionPolars(optim, surface, {'cl', 'cd', 'cdp', 'cm'});
end

% Spanwise Reynolds number
ReSpan = Re * optim.(surface).aero.sectionPolars.c_m/optim.(surface).(chords{s})(1);

% Compute loads (SL) and perf (cruise) airfoil adbs
aswingLoadsAirfoilADB = GenerateASWINGAirfoilADB(optim.(surface).aero.airfoilADB, csOcs(1), optim.(surface).aero.sectionPolars.toc, ReSpan(1, :));
aswingPerfAirfoilADB  = GenerateASWINGAirfoilADB(optim.(surface).aero.airfoilADB, csOcs(1), optim.(surface).aero.sectionPolars.toc, ReSpan(2, :));
optim.(surface).aero.aswingLoadsAirfoilADB = aswingLoadsAirfoilADB;
optim.(surface).aero.aswingLoadsAirfoilADB.t = optim.(surface).aero.sectionPolars.t;
optim.(surface).aero.aswingPerfAirfoilADB  = aswingPerfAirfoilADB;
optim.(surface).aero.aswingPerfAirfoilADB.t = optim.(surface).aero.sectionPolars.t;
    
% Loop through control surfaces and assigns aero derivatives accordingly 
for c = 1:Nc

    if csOcs(c) ~= csOcs(1) % If the additional control surfaces don't have the same chord, compute their derivative
        % Compute additional dbs loads (SL) and perf (cruise) airfoil adbs
        aswingLoadsAirfoilADB = GenerateASWINGAirfoilADB(optim.(surface).aero.airfoilADB, csOcs(c), optim.(surface).aero.sectionPolars.toc, ReSpan(1, :)); 
        aswingPerfAirfoilADB  = GenerateASWINGAirfoilADB(optim.(surface).aero.airfoilADB, csOcs(c), optim.(surface).aero.sectionPolars.toc, ReSpan(2, :));
    end
    
    % Append to dbs
    optim.(surface).aero.aswingLoadsAirfoilADB.dCLdF{c} = aswingLoadsAirfoilADB.dCLdF{1};
    optim.(surface).aero.aswingLoadsAirfoilADB.dCDdF{c} = aswingLoadsAirfoilADB.dCDdF{1};
    optim.(surface).aero.aswingLoadsAirfoilADB.dCMdF{c} = aswingLoadsAirfoilADB.dCMdF{1};
    optim.(surface).aero.aswingPerfAirfoilADB.dCLdF{c} = aswingPerfAirfoilADB.dCLdF{1};
    optim.(surface).aero.aswingPerfAirfoilADB.dCDdF{c} = aswingPerfAirfoilADB.dCDdF{1};
    optim.(surface).aero.aswingPerfAirfoilADB.dCMdF{c} = aswingPerfAirfoilADB.dCMdF{1};
    
    if ~isfield(optim.(surface), 'controlSurface')
        optim.(surface).aero.aswingLoadsAirfoilADB = rmfield(optim.(surface).aero.aswingLoadsAirfoilADB, {'dCLdF', 'dCDdF', 'dCMdF'});
    end

    % For performance, remove the sectional cl limits as it is enforced through
    % section polar look-ups
    optim.(surface).aero.aswingPerfAirfoilADB.CLmaxActual = optim.(surface).aero.aswingPerfAirfoilADB.CLmax;
    optim.(surface).aero.aswingPerfAirfoilADB.CLminActual = optim.(surface).aero.aswingPerfAirfoilADB.CLmin;
    optim.(surface).aero.aswingPerfAirfoilADB.CLmax = 100 * ones(length(optim.(surface).aero.aswingPerfAirfoilADB.CLmax), 1);
    optim.(surface).aero.aswingPerfAirfoilADB.CLmin = -100 * ones(length(optim.(surface).aero.aswingPerfAirfoilADB.CLmin), 1);
end
