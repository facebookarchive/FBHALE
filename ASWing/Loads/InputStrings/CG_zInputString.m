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
  
function [  ] = CG_zInputString( optim )
% Input string to find undeflected ASWING calculated aircraft CG.

fid3 = fopen([optim.ASWINGIODir 'CGz' num2str(optim.designID) '.txt'],'wt');

% turn off graphics
fprintf(fid3,'plpa\n');
fprintf(fid3,'G F\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

% load aircraft file
fprintf(fid3,'load %sLoadsAircraft%i.asw\n', optim.ASWINGIODir, optim.designID);
SetNodes(optim, fid3 );

fprintf(fid3,'\n');
fprintf(fid3,'\n');

fprintf(fid3,'\n');
fprintf(fid3,'\n');

% exit
fprintf(fid3,'quit\n');
fclose(fid3);


end

