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
  
function s = CoBladeLoadsTail (s, entity)
% Routine to transform ASWING loads to coblade format.

% Transform limit loads from ASWING reference frame to coblade:
s.(entity).structure = transformLoads (s.(entity).structure);

% Update resultant loads:
s.(entity).structure.ResLoads.Vx = (s.(entity).structure.limitLoads.Vx(:));
s.(entity).structure.ResLoads.Vy = (s.(entity).structure.limitLoads.Vy(:));
s.(entity).structure.ResLoads.Vz = (s.(entity).structure.limitLoads.Vz(:));
s.(entity).structure.ResLoads.Mx = (s.(entity).structure.limitLoads.Mx(:));
s.(entity).structure.ResLoads.My = (s.(entity).structure.limitLoads.My(:));
s.(entity).structure.ResLoads.Mz = smooth(s.(entity).structure.limitLoads.Mz(:));

% Compute derivatives:
s.(entity).structure.ResLoads = diffLoads (s.(entity).structure.ResLoads, s.(entity).structure.zSec);
end

function s = transformLoads (s)

% Transform loads from ASWING axis (c-s-n) to coblade's reference axis (x-y-z):
% Mx, My are w.r.t section centroid.
% Mz is w.r.t to section shear center. 
% Compare aswing and coblade manuals for sign conventions. 

% Correct z-axis sign:
    s.limitLoads.Vx = s.limitLoads.Fc (:);
    s.limitLoads.Vy = s.limitLoads.Fn(:);
    s.limitLoads.Vz = s.limitLoads.Fs(:);
    s.limitLoads.Mx = -s.limitLoads.Mc(:);
    s.limitLoads.My = -s.limitLoads.Mn(:);
    s.limitLoads.Mz = s.limitLoads.Ms(:);

end

function loads = diffLoads (loads, locs_m)
dlocs_m = diff(locs_m);
loads.dVx_dz = [diff(loads.Vx(:))./dlocs_m; 0];
loads.dVy_dz = [diff(loads.Vy(:))./dlocs_m; 0];
loads.dVz_dz = [diff(loads.Vz(:))./dlocs_m;0];
loads.dMx_dz = [diff(loads.Mx(:))./dlocs_m;0];
loads.dMy_dz = [diff(loads.My(:))./dlocs_m;0];
loads.dMz_dz = smooth([diff(loads.Mz(:))./dlocs_m;0]);
end

 
