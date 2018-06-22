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
optim.wing.controlSurface(1).yobo2           = [0.6 optim.wing.controlSurface(1).maxyobo2];           % Updated during aileron sizing
optim.vtail.c_m                   	= 1.5 * [1 optim.vtail.taperRatio];             % Updated during tail sizing
optim.vtail.structure.mass_kg       = 5;                                            % Updated during tail sizing
optim.boom.structure.mass_kg        = 15;                                           % Updated during tail sizing
optim.wing.x_m                      = 2;                                            % Updated during pitch trimming
optim.wing.z_m                      = 0;

optim.htail.Sref_m2                	= optim.htail.VH * optim.wing.Sref_m2 * optim.wing.cref_m / (optim.htail.x_m - optim.wing.x_m);
optim.htail.bref_m                  = sqrt(optim.htail.AR * optim.htail.Sref_m2);
optim.htail.c_m                     = optim.htail.Sref_m2 / optim.htail.bref_m * [1 1];
optim.htail.structure.mass_kg       = optim.htail.Sref_m2 * .5;                                   	% Updated during tail sizing
optim.vtail.x_m                 	= optim.htail.x_m + optim.htail.c_m(1) + optim.vtail.c_m(1)/4; 	% Updated during tail sizing
optim.vtail.Sref_m2                	= optim.vtail.AR * trapz(optim.vtail.yobo2, optim.vtail.c_m)^2; % Updated during tail sizing
optim.vtail.bref_m                  = sqrt(optim.vtail.Sref_m2*optim.vtail.AR);
optim.avionics.xCG_m             	= 2; % updated to wing location in 2 boom case
optim.payload.xCG_m                 = 2; % updated to wing location in 2 boom case
optim.boom.x_m                      = 0;
optim.pod.x_m                       = 0;
optim.boom.structure.length_m       = max(optim.htail.x_m, optim.vtail.x_m);

% Weights
optim.propulsion.mass_kg    	= 30 * [1/4 1/4]; % Updated in propulsion sizing
optim.wing.structure.mass_kg	= 0.30 * optim.MGTOW_kg;
optim.batteries.mass_kg     	= 0.35 * optim.MGTOW_kg .* optim.batteries.massDistribution;

% Misc
optim.propulsion.propeller.r_m     	= 2; % Will be overwritten during propulsion sizing
optim.wing.aero.CLcruise            = optim.wing.aero.CLsizing;
end
