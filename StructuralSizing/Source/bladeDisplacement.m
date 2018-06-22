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

Modified from the original coblade repository. License for the original 
file appears below:

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
  
function Disp = bladeDisplacement(zSec, ...
                                  zFrac, ...
                                  NUM_SEC, ...
                                  ResLoads, ...
                                  flapI_mc, ...
                                  edgeI_mc, ...
                                  EIx, ...
                                  EIy, ...
                                  EIxy, ...
                                  axial_stff, ...
                                  tor_stff, ...
                                  Tran, ...
                                  DISP_CF)

Vz     = ResLoads.Vz;
Mx     = ResLoads.Mx;
My     = ResLoads.My;
dMz_dz = ResLoads.dMz_dz;
                              
%% Compute correction factors for the displacements (correction factors for displacement of tapered cantilever beams)     
if DISP_CF                              
    cf = dispCorrectionFactor(zFrac, NUM_SEC, flapI_mc, edgeI_mc);
else
    cf = ones(NUM_SEC, 1);
end

%% Compute the displacement of the centroidal (tension center) axis by integration of the 2nd order ODE's

d2uo_dz2 = cf .* ( My.*EIx + Mx.*EIxy)./(EIx.*EIy - EIxy.^2);
d2vo_dz2 = cf .* (-Mx.*EIy - My.*EIxy)./(EIx.*EIy - EIxy.^2);
dwo_dz   = Vz ./ axial_stff;

% numerically compute the indefinite integrals
duo_dz = cumtrapzf(zSec, d2uo_dz2);
dvo_dz = cumtrapzf(zSec, d2vo_dz2);
dtz_dz = cumtrapzf(zSec, dMz_dz);
% add the constant of integration, from the appropriate boundary condition
duo_dz = duo_dz - duo_dz(1);
dvo_dz = dvo_dz - dvo_dz(1);
dtz_dz = dtz_dz - dtz_dz(end);

% numerically compute the indefinite integrals
uo = cumtrapzf(zSec, duo_dz);
vo = cumtrapzf(zSec, dvo_dz);
wo = cumtrapzf(zSec, dwo_dz);
tz = cumtrapzf(zSec, dtz_dz ./ tor_stff);
% add the constant of integration, from the appropriate boundary condition
uo = uo - uo(1);
vo = vo - vo(1);
wo = wo - wo(1);
tz = tz - tz(1);

%% Compute the rigid body rotations of the cross sections
N    = NUM_SEC;
angY = zeros(N,1);
angX = zeros(N,1);
for i = 2:N
    dx      = uo(i) - uo(i-1);
    dy      = vo(i) - vo(i-1);
    dz      = (zSec(i) + wo(i)) - (zSec(i-1) + wo(i-1));
    angY(i) = atan2(dx, dz);
    angX(i) = atan2(dy, sqrt(dx^2 + dz^2)); 
end
Rz  = cell(N,1);
Rxy = cell(N,1);
for i = 1:N
    % rotation matrices
    RotZ = [  cos(tz(i)),    -sin(tz(i)),              0;
              sin(tz(i)),     cos(tz(i)),              0;
                       0,              0,              1]; 
    RotY = [ cos(angY(i)),             0,   sin(angY(i));
                        0,             1,              0;
            -sin(angY(i)),             0,   cos(angY(i))];
    RotX = [            1,             0,              0;
                        0, cos(-angX(i)), -sin(-angX(i));
                        0, sin(-angX(i)),  cos(-angX(i))];
    % this rotation matrix rotates the cross section so it is orthogonal 
    % to the centriodal (tension center) axis in accordance with the Euler-Bernoulli assumption
    Rz{i}  = RotZ;
    Rxy{i} = RotY * RotX;
end

%% Compute the component of displacment in the direction of the tower
disp_G     = Tran.BG * [uo'; vo'; wo']; % displacement vector w.r.t. the global coordinate system
tipDeflect = disp_G(2,:)';              % in the global coordinate system the x-component is in the direction of the tower

%% Collect the output
Disp.uo         = uo;
Disp.vo         = vo; 
Disp.wo         = wo;
Disp.tz         = tz;
Disp.Rz         = Rz;
Disp.Rxy        = Rxy;
uoMax = max(abs(uo));
voMax = max(abs(vo));
woMax = max(abs(wo));
Disp.tipDeflect = max([uoMax, voMax, woMax]);  % only take the displacment at the tip of the blade, in the direction of the tower
Disp.maxTwist = max(abs(Disp.tz/pi*180));
end % function bladeDisplacement



