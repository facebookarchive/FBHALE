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
  
function BlendAirfoil (airfoil1, airfoil2, fraction, airfoilName)

% Temp coommand file
BlendCommandFile = 'BlendAirfoilComs.txt';

% Command input
G{1} = 'INTE';
G{2} = 'F';
G{3} = airfoil1;
G{4} = 'F';
G{5} = airfoil2;
G{6} = [num2str(fraction)];
G{7} = 'blend';
G{8} = 'PANE';
G{9} = 'SAVE';
G{10} = airfoilName;
G{11} = ' ';
G{12} = ' ';
G{13} = 'QUIT';

fid = fopen(BlendCommandFile, 'w');
fprintf(fid, '%s\n', G{:});
fclose(fid);

% Run xfoil geo routine
system(['xfoil <' BlendCommandFile '>' GetNullDevice ' 2>&1']);

% delete temp file
delete(BlendCommandFile);