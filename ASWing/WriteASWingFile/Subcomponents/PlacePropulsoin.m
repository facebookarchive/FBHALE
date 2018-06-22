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
  
function [ optim ] = PlacePropulsoin( optim,aswingIn )
% Adds propulsion to aswing file

% Header
fprintf(aswingIn,'#======================\n');
fprintf(aswingIn,'Engine\n');
fprintf(aswingIn,'# KPeng   IEtyp   Nbeam   t     Xp        Yp      Zp      Tx      Ty  Tz  dFdPe dMdPe  Rdisk Omega \n');

if optim.boom.N == 1 && mod(sum(optim.propulsion.N),2)==1
    beamNumber = 4;
else
    beamNumber = 1;
end

if optim.propulsion.y_m(1) == 0
    fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', ...
        1, 0, beamNumber, 0, optim.propulsion.xCG_m(1), optim.propulsion.y_m(1), optim.propulsion.zCG_m, -1.0, 0, 0, 1, 0, optim.propulsion.propeller.r_m, 12.5);
end

% Rest of motors shold be attached to the wing 
beamNumber = optim.wing.beamNumber;

% Write reflected propulsion
reflectedIndicies = find(optim.propulsion.y_m);
for i = min(reflectedIndicies):max(reflectedIndicies)
    fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', ...
        1, 0, beamNumber, optim.propulsion.y_m(i)/(optim.wing.bref_m/2), ...
        optim.propulsion.xCG_m(i), optim.propulsion.y_m(i), optim.propulsion.zCG_m(i), ...
        -1.0, 0, 0, 1, 0, optim.propulsion.propeller.r_m, 12.5);
    fprintf(aswingIn,'\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\n', ...
        1, 0, beamNumber, -1*optim.propulsion.y_m(i)/(optim.wing.bref_m/2), ...
        optim.propulsion.xCG_m(i), -1*optim.propulsion.y_m(i), optim.propulsion.zCG_m(i), ...
        -1.0, 0, 0, 1, 0, optim.propulsion.propeller.r_m, 12.5);
end

fprintf(aswingIn,'End\n');
end



