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
  
function [ optim ] = UpdateMassLocations( optim )
% This function sets configuration dependent point mass locations.

optim.wing.y_m =0;

%% Transform structural data to global reference frame
optim = BuildAircraftGeometry (optim);


%% Batteries
optim.batteries.y_m    = optim.batteries.y_m;
optim.batteries.zCG_m    = [0 0];

%% Propulsion
%optim.propulsion.xCG_m = optim.propulsion.xCG_m  - populated elsewhere for
% now
optim.propulsion.y_m = [optim.batteries.y_m(2)-optim.propulsion.propeller.r_m*2 optim.batteries.y_m(2)+optim.propulsion.propeller.r_m*2];

saxis   = interp1(optim.wing.globaly_m{1}, optim.wing.globalx_m{1} , optim.propulsion.y_m-optim.propulsion.propeller.r_m);
chord   = interp1(optim.wing.globaly_m{1}, optim.wing.structure.chord_m{1} , optim.propulsion.y_m);
xax     = interp1(optim.wing.globaly_m{1}, optim.wing.structure.Xax{1}, optim.propulsion.y_m);
Le      = saxis-chord.*xax;
optim.propulsion.xCG_m = Le - optim.propulsion.propeller.r_m./2;

optim.propulsion.zCG_m = optim.wing.z_m+ -1 * optim.propulsion.propeller.r_m/2;
optim.propulsion.zCG_m = [optim.propulsion.zCG_m optim.propulsion.zCG_m]; 

%% Pod locations
optim.pod.x_m = [0 optim.batteries.xCG_m(2)-optim.pod.structure.length_m(2)/2];
optim.pod.y_m = [0 optim.batteries.y_m(2)];
optim.pod.z_m = [0 optim.batteries.zCG_m(2)];

%% Landing gear y locations follow the battery packs
optim.landingGear.xCG_m = [optim.pod.x_m(1)+optim.pod.structure.length_m(1)/2 optim.batteries.xCG_m(2)];
optim.landingGear.y_m = [0 optim.batteries.y_m(2)];
optim.landingGear.zCG_m = [max(optim.pod.omlSectionsR_m{1}) max(optim.pod.omlSectionsR_m{2})];

%% Harness
optim.harness.xCG_m   = [optim.pod.x_m(1)+optim.pod.structure.length_m(1)/2 optim.propulsion.xCG_m(1) optim.batteries.xCG_m(2) optim.propulsion.xCG_m(2)];
optim.harness.y_m   = [0 optim.propulsion.y_m(1) optim.pod.y_m(2) optim.propulsion.y_m(2)];
optim.harness.zCG_m   = [0 optim.propulsion.zCG_m(1) optim.pod.z_m optim.propulsion.zCG_m(2)];

%% Margin
optim.margin.xCG_m = 1i;
optim.margin.y_m = 0;
optim.margin.zCG_m = 0;

%% Avionics
optim.avionics.xCG_m = optim.avionics.xCG_m; % Set wlsewhere for now
optim.avionics.y_m   = 0;
optim.avionics.zCG_m   = 0;

%% Payload
optim.payload.xCG_m = optim.payload.xCG_m; % Set wlsewhere for now
optim.payload.y_m   = 0;
optim.payload.zCG_m   = 0;

%% Solar
% Wing
optim.wing.solar.xCG_m  = optim.wing.solar.xRef_m + interp1(optim.wing.globaly_m{1}, optim.wing.globalx_m{1}, optim.wing.solar.y_m);
optim.wing.solar.y_m  = optim.wing.solar.y_m; % output of GetSolarMassDistribution
optim.wing.solar.zCG_m  = optim.wing.z_m*ones(1, optim.wing.solar.nRepresentativePointMasses);

end

