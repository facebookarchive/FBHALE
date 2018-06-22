%{
Copyright (c) 2012, Danny Sale 
Copyright (c) 2016, H.J. Sommer 
Copyright (c) 2009, John D'Errico 
Copyright (c) 2017, Yair Altman 
Copyright (c) 2016, S. Samuel Chen 
All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are 
met:

* Redistributions of source code must retain the above copyright 
notice, this list of conditions and the following disclaimer. 
* Redistributions in binary form must reproduce the above copyright 
notice, this list of conditions and the following disclaimer in 
the documentation and/or other materials provided with the distribution 
* Neither the name of the Penn State University nor the names 
of its contributors may be used to endorse or promote products derived 
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
POSSIBILITY OF SUCH DAMAGE.
%}
  
function ResLoads = loadsResultant(zSec, ...
                                   NUM_SEC, ...
                                   AppLoads,...
                                   x_cm, ...
                                   y_cm, ...
                                   x_tc, ...
                                   y_tc, ...
                                   x_sc, ...
                                   y_sc)

N    = NUM_SEC;
px_a = AppLoads.px_a;
py_a = AppLoads.py_a;
qz_a = AppLoads.qz_a;
px_w = AppLoads.px_w;
py_w = AppLoads.py_w;
pz_w = AppLoads.pz_w;
px_c = AppLoads.px_c;
py_c = AppLoads.py_c;
pz_c = AppLoads.pz_c;

% aerodynamic center (point where the aerodynamic loads are applied)
% for now, assume that the aero loads are applied at the pitch axis, which
% is the same as (x_ac, y_ac) = (0, 0) w.r.t. the blade coordinate system
x_ac = zeros(N,1);
y_ac = zeros(N,1);

%% integrate the differential equations of equilibrium (derived for a cantilever beam)

% resultant shear forces and axial force
dVx_dz = -px_a - px_w - px_c;
dVy_dz = -py_a - py_w - py_c;
dVz_dz = -pz_w - pz_c;
% numerically compute the indefinite integrals
Vx = cumtrapzf(zSec, dVx_dz);
Vy = cumtrapzf(zSec, dVy_dz);
Vz = cumtrapzf(zSec, dVz_dz);
% add the constant of integration, from the appropriate boundary condition
Vx = Vx - Vx(end);
Vy = Vy - Vy(end);
Vz = Vz - Vz(end);

% resultant bending moments and torsional moment
dMx_dz =  Vy - (pz_w + pz_c).*(y_cm - y_tc);
dMy_dz = -Vx + (pz_w + pz_c).*(x_cm - x_tc);
dMz_dz = -qz_a - py_a.*(x_ac - x_sc) ...
               + px_a.*(y_ac - y_sc) ...
               - (py_w + py_c).*(x_cm - x_sc) ...
               + (px_w + px_c).*(y_cm - y_sc); 
% numerically compute the indefinite integrals
Mx = cumtrapzf(zSec, dMx_dz);
My = cumtrapzf(zSec, dMy_dz);
Mz = cumtrapzf(zSec, dMz_dz);
% add the constant of integration, from the appropriate boundary condition
Mx = Mx - Mx(end);
My = My - My(end);
Mz = Mz - Mz(end);

%% Collect the output
ResLoads.Vx     = Vx;     % resultant force in the x-direction w.r.t. the blade coordinate system
ResLoads.Vy     = Vy;     % resultant force in the y-direction w.r.t. the blade coordinate system
ResLoads.Vz     = Vz;     % resultant force in the z-direction w.r.t. the blade coordinate system
ResLoads.Mx     = Mx;     % resultant moment about the centroidal (tension center) x-axis w.r.t the blade coordinate system
ResLoads.My     = My;     % resultant moment about the centroidal (tension center) y-axis w.r.t the blade coordinate system
ResLoads.Mz     = Mz;     % resultant moment about the centroidal (tension center) z-axis w.r.t the blade coordinate system
ResLoads.dVx_dz = dVx_dz; % 1st derivative of Vx
ResLoads.dVy_dz = dVy_dz; % 1st derivative of Vy
ResLoads.dVz_dz = dVz_dz; % 1st derivative of Vz
ResLoads.dMx_dz = dMx_dz; % 1st derivative of Mx
ResLoads.dMy_dz = dMy_dz; % 1st derivative of My
ResLoads.dMz_dz = dMz_dz; % 1st derivative of Mz

end % function loadsResultant

