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
  
function optim = PitchTrim(optim, iterCnt)
% This function ensures CM=0 at cruise.
% For empennage-stabilized aircraft, this is enforced by displacing the
% wing. The vtail is also resized in the loop.
% For flying-wings, batteries are displaced towards the same goal.

% Capture sensitivity of CG position to wing assembly position and deduce
% effective displacement due to deflections
optim0 = optim;
optim0.wing.x_m = optim0.wing.x_m*1.05;
optim0 = UpdateMassProperties(optim0, 'analytical');
dxCGdw = (optim.xCG_m - optim0.xCG_m)/(optim.wing.x_m - optim0.wing.x_m);
optim.wing.dxCGdw = dxCGdw;
optim.wing.dxDue2Deflections_m = (optim.xCGDef_m - optim.xCG_m)/dxCGdw;

% Pitch trim based on config
if sum(optim.boom.N) > 0
    optim = PitchTrimEmpennage(optim);
else
    optim = PitchTrimFlyingWing(optim, iterCnt);
end
end
