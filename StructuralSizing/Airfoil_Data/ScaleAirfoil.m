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
  
function ScaleAirfoil (airfoilFile, tOc)

% Temp command file 
ScaleCommandFile = 'ScaleAirfoilComs.txt';

% Command inputs
G{1} = 'PLOP';
G{2} = 'G';
G{3} = ' ';
G{4} = 'load';
G{5} = airfoilFile;
G{6} = 'GDES';
G{7} = 'TSET';
G{8} = [num2str(tOc)];
G{9} = ' ';
G{10} = 'X';
G{11} = ' ';
G{12} = 'SAVE';
G{13} = airfoilFile;
G{14} = ' ';
G{15} = ' ';
G{16} = 'QUIT';

fid = fopen(ScaleCommandFile, 'w');
fprintf(fid, '%s\n', G{:});
fclose(fid);

% Run xfoil geo routines 
system(['xfoil <' ScaleCommandFile '>' GetNullDevice ' 2>&1']);

% delete temp files
delete(ScaleCommandFile);
