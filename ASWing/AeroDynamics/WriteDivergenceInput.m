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
  
function [] = WriteDivergenceInput( optim )
% Write input string for RunASWINGDicergence\
%create file
fid3 = fopen([optim.ASWINGIODir 'DivergenceInputString' num2str(optim.designID) '.txt'],'wt');

% turn off graphics
fprintf(fid3,'plpa\n');
fprintf(fid3,'G F\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

% load aircraft file
fprintf(fid3,'load %sDivergenceAircraft%i.asw\n', optim.ASWINGIODir, optim.designID);
SetNodes(optim, fid3 );

% set iterations limit
fprintf(fid3,'oper\n');
fprintf(fid3,'K\n');
fprintf(fid3,'I\n');
fprintf(fid3,'40\n'); % set number of iterations allowable
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

%% Run Cruise 

% Establish Steady Level flight
% load pnt file
fprintf(fid3,'pget\n');
fprintf(fid3,'%sSLF.pnt\n', optim.ASWINGIODir);

%Set up trimmed slf
fprintf(fid3,'oper\n');
fprintf(fid3,'init\n');
fprintf(fid3,'!L\n');
fprintf(fid3,'\t%3.3f\n', optim.MGTOW_kg*optim.constants.g_ms2);
fprintf(fid3,'!V\n');
fprintf(fid3,'\t%3.3f\n', optim.wing.vn.vEasTopRightCorner_ms);
fprintf(fid3,'X\n');
fprintf(fid3,'R\n');

% Anchor plane
fprintf(fid3,'%%\n');
fprintf(fid3,'A\n');
fprintf(fid3,'19 19\n');
fprintf(fid3,'20 20\n');
fprintf(fid3,'21 21\n');
fprintf(fid3,'22 22\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

% Run Dynamic modes
fprintf(fid3,'mode\n');
fprintf(fid3,'K\n');
fprintf(fid3,'L\n');
fprintf(fid3,'\n');
fprintf(fid3,'N\n');
fprintf(fid3,'50\n');
fprintf(fid3,'\n');
fprintf(fid3,'W\n');
fprintf(fid3,'%sDivergenceEVs%i.e00\n', optim.ASWINGIODir, optim.designID);
fprintf(fid3,'Y\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

%% exit
fprintf(fid3,'quit\n');
fclose(fid3);

end

