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
optim.wing.zoc                      	= [0 0 0]; % Non dimentionalized by root chord
optim.wing.aero.description 			= 'polar'; 
optim.wing.aero.airfoilADB              = load('AAAFamily.mat');
optim.wing.yAirfoilsobo2              	= [0 optim.wing.yobo2(2) (1+optim.wing.yobo2(2))/2 1]; % root, break, half taper, tip
optim.wing.winglet.zoc               	= 0;

% Htail
optim.htail.aero.CLcruise           	= 0;
optim.htail.sweep_deg               	= 0;
optim.htail.AR                       	= 8;
optim.htail.aero.airfoilADB         	= load('HT12.mat');
optim.htail.toc                         = optim.htail.aero.airfoilADB.dim1.vector(1)*[1 1];
optim.htail.yAirfoilsobo2             	= [0 1];
optim.htail.taperRatio               	= 1;
optim.htail.twists_deg               	= [0 0];
optim.htail.yobo2                    	= [0 1];
optim.htail.zoc                      	= [0 0];
optim.htail.x_m                       	= optim.boom.lob * optim.wing.bref_m;

% Vtail
optim.vtail.sweep_deg                 	= 0;
optim.vtail.maxVy_ms                  	= 2;
optim.vtail.CnBetaMin                	= 0.001; % Raymer
optim.vtail.AR                          = 4;
optim.vtail.taperRatio                	= 1;
optim.vtail.aero.airfoilADB            	= load('S9033.mat');
optim.vtail.toc                      	= optim.vtail.aero.airfoilADB.dim1.vector(1)*[1 1];
optim.vtail.yAirfoilsobo2             	= [0 1];
optim.vtail.twists_deg               	= [0 0];
optim.vtail.yobo2                     	= [0 1];
optim.vtail.zoc                        	= [0 0];

% Boom
optim.boom.toc                         	= [1 1 1 1]; % dummy / not used
optim.boom.shape                        = load('Fairing.mat');
optim.boom.CDFairing                    = 2e-2;
optim.boom.attachmentLocation           = .55;
optim.boom.minRadius_m               	= 0.35;
optim.boom.maxRadius_m               	= 0.35;
optim.boom.z_m                          = 0;

% Pod
optim.pod.shape                     	= optim.boom.shape;
optim.pod.maxRadius_m                   = .35;
optim.pod.CDFairing                     = 2e-2;

% Add geometry break points to the wing t/c distribution
yAirfoilsobo2            = sort(unique([optim.wing.yAirfoilsobo2 optim.wing.yobo2]));
optim.wing.toc           = interp1(optim.wing.yAirfoilsobo2, optim.wing.toc, yAirfoilsobo2);
optim.wing.toc           = optim.wing.toc * (max(optim.wing.aero.airfoilADB.dim1.vector)-min(optim.wing.aero.airfoilADB.dim1.vector)) + min(optim.wing.aero.airfoilADB.dim1.vector);
optim.wing.yAirfoilsobo2 = yAirfoilsobo2;

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

optim.aircraft.controlSurfaces.roll.requiredRollRate             = 0.05;

% Control Surfaces
% Surface scheduling maps to [roll, pitch, yaw] or [F1 F2 F3]

% Wing
% Control surface size is solved for in the loop so max yobo2 is specified 
optim.wing.controlSurface(1).name       = {'aileron'};
optim.wing.controlSurface(1).schedule   = [1 0 0];
optim.wing.controlSurface(1).csoc       = 0.25; % Relative control surface chord
optim.wing.controlSurface(1).maxDeflection_deg   = 12.5;
optim.wing.controlSurface(1).maxDeflection0_deg   = 12.5;
optim.wing.controlSurface(1).maxyobo2   = 0.99;
optim.wing.controlSurface(1).gearRatio 	= 3; 
optim.wing.controlSurface(1).mapping    = [1 0 0];

% Htail
optim.htail.controlSurface(1).name      = {'elevator'};
optim.htail.controlSurface(1).schedule  = [0 1 0];
optim.htail.controlSurface(1).yobo2     = [0 1];
optim.htail.controlSurface(1).csoc      = 1;
optim.htail.controlSurface(1).maxDeflection_deg  = 9.1475; % CLmax + 2 degrees rad2deg(.7362/5.9015) + 2 = 9.1475

% Vtail
optim.vtail.controlSurface(1).name      = {'rudder'};
optim.vtail.controlSurface(1).schedule  = [0 0 1];
optim.vtail.controlSurface(1).yobo2     = [0 1];
optim.vtail.controlSurface(1).csoc      = 1;
optim.vtail.controlSurface(1).maxDeflection_deg   = 9.6135; % CLmax + 2 deg (.9013/6.7828) + 2 = 9.6135

end
