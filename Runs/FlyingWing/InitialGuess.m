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
  
% This routine assigns all the design inputs for the design closure of a
% given configuration.
function [optim] = InitialGuess(optim)

% Lengths and positions
optim.wing.controlSurface(1).yobo2   	= [0.6 optim.wing.controlSurface(1).maxyobo2];  	% Updated during aileron sizing
optim.wing.x_m                          = 1;                                                   	% Updated during tail sizing
optim.batteries.sparOffsetX_m           = 0;

optim.pod.x_m = [0 1];
optim.pod.y_m = [0 1];
optim.pod.z_m = [0 0];
optim.avionics.xCG_m	= 2; % updated to wing location in 2 boom case
optim.payload.xCG_m 	= 2; % updated to wing location in 2 boom case

% Initial winglet length
optim.wing.winglet.zoc = 1.3;

optim.batteries.xCG_m(2) =  2+optim.wing.x_m + optim.batteries.yobo2(2) * optim.wing.bref_m/2 * sind(optim.wing.sweep_deg);

% Weights
optim.propulsion.mass_kg    	= 30 * [1/4 1/4]; % Updated in propulsion sizing
optim.wing.structure.mass_kg	= 0.35 * optim.MGTOW_kg;
optim.pod.structure.mass_kg     = [3 3];
optim.batteries.mass_kg     	= 0.35 * optim.MGTOW_kg .* optim.batteries.massDistribution;

% Misc
optim.propulsion.propeller.r_m     	= 2; % Will be overwritten during propulsion sizing
optim.wing.aero.CLcruise            = optim.wing.aero.CLsizing;
end
