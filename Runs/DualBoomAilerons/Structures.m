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
  
function optim                                     = Structures(optim)

%==========================================================================
% Wing V-n & Loads inputs
%==========================================================================

optim.wing.vn.VzCruiseGustEAS_ms                   = 3.4;
optim.wing.vn.VzDiveGustEAS_ms                     = 3.4; 
optim.wing.vn.nmax                                 = 2;
optim.wing.vn.nmin                                 = -0.5;

optim.environment.gustLoadMargin                   = 1.1;

optim.environment.DARPAGust.wavelength_2_m         = optim.wing.bref_m/1.2/2;
optim.environment.DARPAGust.width_m                = optim.wing.cref_m*8; 
optim.environment.DARPAGust.magnitude_m_s          = 2; 
optim.environment.DARPAGust.dt_s                   = .1;
optim.environment.DARPAGust.n_steps                = 60;

optim.environment.FARVerticalGust.V_ms             = 3.4;
optim.environment.FARVerticalGust.Width_Chord      = 1;
optim.environment.FARVerticalGust.dt_s             = 0.1;
optim.environment.FARVerticalGust.n_steps          = 60;

optim.environment.ControlSurfaceDeflection.dt_s    = 0.1;
optim.environment.ControlSurfaceDeflection.n_steps = 30;
optim.environment.FactorOfSafety                   = 1.5;

%==========================================================================
% Aircraft Modes
%==========================================================================

optim.wing.dynamicModes.activeFlutterControlVNEReductionFactor = 1;

%==========================================================================
% Structures:
%==========================================================================

% wing properties:
optim.wing.structure.numInstances                  = 2;                         %no. of instances of structure
optim.wing.structure.DivsPerSection                = 4;                         %no. of divisions per section
optim.wing.structure.length_m                      = optim.wing.bref_m*0.5/cosd(optim.wing.sweep_deg);     %beam length
optim.wing.structure.twist_deg                     = optim.wing.twists_deg;
optim.wing.structure.toc                           = optim.wing.toc;
optim.wing.structure.chord_m                       = optim.wing.c_m;            %chord distribution
optim.wing.structure.airfoilBreakPoints            = optim.wing.yAirfoilsobo2; 
optim.wing.structure.sizingLocations               = optim.wing.yobo2; 
optim.wing.structure.skinWeightPerArea_kgm2        = 0.30375;
optim.wing.structure.NribsPerSpan_m                = 1.189;
optim.wing.structure.ribMassPerChord_kgm           = 0.084;
optim.wing.structure.numStiffeners                 = 2;
optim.wing.structure.sparCap_core_nPly(2)          = 2;
optim.wing.structure.TopUDScale                    = [2.5 2.5];
optim.wing.structure.UDScaleSpanExtent             = [0 0.8];

% horizontal tail properties:
optim.htail.structure.skinWeightPerArea_kgm2       = 0.351;
optim.htail.structure.NribsPerSpan_m               = 2.5;
optim.htail.structure.ribMassPerChord_kgm          = 0.0312;
optim.htail.structure.DivsPerSection               = 4;                        %no. of spanwise stations in structural grid
optim.htail.structure.toc                          = optim.htail.toc;     
optim.htail.structure.sparCap_core_nPly            = [2 2];
optim.htail.structure.web_core_nPly                = [1 1];
optim.htail.structure.sparCapWidth                 = 0.20;
optim.htail.structure.pitchAxis                    = 0.15;
optim.htail.structure.UD_nPly                      = [2 1]; 
optim.htail.structure.numStiffeners                = 2;
optim.htail.structure.TopUDScale                   = [2 2];

% vertical tail properties:
optim.vtail.structure.skinWeightPerArea_kgm2       = 0.3038;
optim.vtail.structure.NribsPerSpan_m               = 2.083;
optim.vtail.structure.ribMassPerChord_kgm          = 0.0375;
optim.vtail.structure.DivsPerSection               = 4 ;                        %no. of spanwise stations in structural grid              
optim.vtail.structure.toc                          = optim.vtail.toc;           %thickness distribution
optim.vtail.structure.sparCap_core_nPly            = [2 2];
optim.vtail.structure.web_core_nPly                = [1 1];
optim.vtail.structure.sparCapWidth                 = 0.10;
optim.vtail.structure.pitchAxis                    = 0.22;
optim.vtail.structure.UD_nPly                      = [1 1]; 
optim.vtail.structure.numStiffeners                 = 2;

% boom properties:
optim.boom.structure.fairingWeightPerArea_kgm2     = 0.33;
optim.boom.structure.DivsPerSection                = 10;                        %no. of spanwise stations in structural grid
optim.boom.structure.sparCap_core_nPly             = [3 3];
optim.boom.structure.web_core_nPly                 = [2 2];
optim.boom.structure.pitchAxis                     = 0.5;
optim.boom.structure.numStiffeners                  = 2;
optim.boom.structure.chord_m                       = [0.3 0.3];                 % Updated during ShapeFuselage
optim.boom.structure.UD_nPly                       = [3 3]; 
optim.boom.structure.UDScaleSpanExtent             = [0.3 0.6];
optim.boom.structure.sparCapWidthInboard           = 0.80;
optim.boom.structure.sparCapWidthOutboard          = 0.70;   

% wing pod structure:
optim.pod.structure.numInstances                   = 1;
optim.pod.structure.fairingWeightPerArea_kgm2      = 0.33;
optim.pod.structure.DivsPerSection                 = 10;                   
optim.pod.structure.sparCap_core_nPly              = [3 3];
optim.pod.structure.web_core_nPly                  = [2 2];
optim.pod.structure.pitchAxis                      = 0.5;
optim.pod.structure.numStiffeners                  = 2;
optim.pod.structure.chord_m                        = [0.3 0.3];          
optim.pod.structure.UD_nPly                        = [3 3]; 
optim.pod.structure.UDScaleSpanExtent              = [0 0];
optim.pod.structure.sparCapWidthInboard            = 0.80;
optim.pod.structure.sparCapWidthOutboard           = 0.80;  

% Set number of instances for each surface. No reflection of the boom to
% get its full mass
optim.boom.structure.numInstances                  = 1;
optim.htail.structure.numInstances                 = 2;
optim.vtail.structure.numInstances                 = 2;

optim                                              = PreprocessStructures(optim); 

end
