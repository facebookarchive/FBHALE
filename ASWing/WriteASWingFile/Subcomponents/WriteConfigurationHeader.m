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
  
function [ optim] = WriteConfigurationHeader( optim, aswingIn )

% Aircraft name
fprintf(aswingIn,'#============\n');
fprintf(aswingIn,'Name\n');
fprintf(aswingIn,'Aquila 2\n');
fprintf(aswingIn,'End\n');

% Set units - always SI and constants at sea level
fprintf(aswingIn,'#============\n');
fprintf(aswingIn,'Units\n');
fprintf(aswingIn,'L 1.0 m\n');
fprintf(aswingIn,'T 1.0 s\n');
fprintf(aswingIn,'F 1.0 N\n');
fprintf(aswingIn,'End\n');

fprintf(aswingIn,'#============\n');
fprintf(aswingIn,'Constant\n');
fprintf(aswingIn,'#   g    rho_SL     V_sound\n');
fprintf(aswingIn,'   9.81  1.225   340.3\n');
fprintf(aswingIn,'End\n');

% Set reference values and point to reference moments about
%reference locations are not specified and default to 0 but should
%probably be placed on the aircraft CG for stability calcs
fprintf(aswingIn,'#============\n');
fprintf(aswingIn,'Reference\n');
fprintf(aswingIn,'# Sref  Cref  Bref\n');
fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\n', optim.wing.Sref_m2, optim.wing.cref_m , optim.wing.bref_m);
fprintf(aswingIn,'End\n');

% Joints
% Join each surface to the boom at its' quarter cord.
fprintf(aswingIn,'#============\n');
fprintf(aswingIn,' Joint\n');
fprintf(aswingIn,'#  Nbeam1  Nbeam2    t1     t2\n');

% Call configuration specific joint specification
JointSpecification(optim, aswingIn)

fprintf(aswingIn,'End\n');

end

