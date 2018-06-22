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
  
function optim = MiscParameters(optim)

% ASWING execution related parameters
optim.aircraft.aero.ASWINGpolar.minCL = 0.8;
optim.aircraft.aero.ASWINGpolar.maxCL = 1.6;
optim.wing.aero.ASWINGpolar.nNodes      = 100;

% Terminal output quantities for polar
optim.aircraft.aero.ASWINGpolar.defaultOutputNumbers	= [2 3 5 7 8 12 22 27];
optim.aircraft.aero.ASWINGpolar.outputColumNames    	= {'deltax_m', 'y_m', 'deltaz_m', 'phi_deg', 'theta_deg', 'psi_deg', 'c_m', 'cl', 'f_lift_N_m'}; 
optim.aircraft.aero.ASWINGpolar.outputColumNumbers  	= [1 2 3 4 5 6 21 22 24];

% Dynamic stability check inputs
optim.wing.dynamicModes.numEigenModesPerPoint   = 30;
optim.wing.dynamicModes.altitude_m              = [optim.mission.minh_m optim.mission.maxh_m ]; % [0 30,000 60,000 80,000] ft


end
