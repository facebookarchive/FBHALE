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
  

function optim = AerodynamicsAndOML(optim)
% This routine assigns design parameters associated with aerodynamics
% (airfoils, tail AR, controls, stability, etc..) for a given aircraft.

%==========================================================================
% Aerodynamics & Basic OML dimensions
%==========================================================================

% Wing
optim.wing.zoc                	= [0 0 0]; % Non dimentionalized by root chord
optim.wing.aero.description 	= 'polar'; 
optim.wing.aero.airfoilADB   	= load('AAAFamily.mat');
optim.wing.yAirfoilsobo2    	= [0 optim.wing.yobo2(2) (1+optim.wing.yobo2(2))/2 1]; % root, break, half taper, tip

% Add geometry break points to the wing t/c distribution
yAirfoilsobo2            = sort(unique([optim.wing.yAirfoilsobo2 optim.wing.yobo2]));
optim.wing.toc           = interp1(optim.wing.yAirfoilsobo2, optim.wing.toc, yAirfoilsobo2);
optim.wing.toc           = optim.wing.toc * (max(optim.wing.aero.airfoilADB.dim1.vector)-min(optim.wing.aero.airfoilADB.dim1.vector)) + min(optim.wing.aero.airfoilADB.dim1.vector);
optim.wing.yAirfoilsobo2 = yAirfoilsobo2;

% Pod shape
optim.pod.attachmentLocation   	= .55;
optim.pod.shape                 = load('Fairing.mat');  
optim.pod.CDFairing          	= 2e-2;
optim.pod.maxRadius_m          	= [0.35 0.35];
optim.pod.minRadius_m          	= [0.35 0.35];

%==========================================================================
% Additional Drag sources
%==========================================================================

optim.payload.CD        = 0.00; 
optim.payload.Sref_m2   = 0.00;
optim.propulsion.CD     = 0.016; 
optim.propulsion.Sref_m2= 1;

%==========================================================================
% Controls
%==========================================================================

optim.aircraft.aero.CnBetaMin = 0.001; % Raymer
optim.aircraft.controlSurfaces.roll.requiredRollRate             = 0.05;

% Control Surfaces
% Surface scheduling maps to [roll, pitch, yaw] or [F1 F2 F3]

% Wing 
% Control surfaces act in a ganged way for pitch, assymetric for roll
% Control surfaces size is solved for in the loop so max yobo2 is specified 
optim.wing.controlSurface(1).name       = {'aileron'};
optim.wing.controlSurface(1).schedule   = [1 1 1];
optim.wing.controlSurface(1).csoc       = 0.25; % Relative control surface chord
optim.wing.controlSurface(1).maxDeflection_deg   = 12.5;
optim.wing.controlSurface(1).maxyobo2   = 0.99;
optim.wing.controlSurface(1).gearRatio 	= 3; 
end
