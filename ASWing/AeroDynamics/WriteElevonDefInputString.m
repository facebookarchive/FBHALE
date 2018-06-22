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
  
function [] = WriteElevonDefInputString(optim)
% Write input string for RunASWINGElevonDef

% create file
fid3 = fopen([optim.ASWINGIODir 'ElevonDefInputString' num2str(optim.designID) '.txt'],'wt');
 
% turn off graphics
fprintf(fid3,'plpa\n');
fprintf(fid3,'G F\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
 
% load aircraft file
fprintf(fid3, 'load %sGetElevlonDefAircraft%i.asw\n', optim.ASWINGIODir, optim.designID);

for i = 1:optim.wing.N
    fprintf(fid3,'node\n');
    fprintf(fid3,['B ' num2str(i) '\n']);
    fprintf(fid3,'I\n');
    fprintf(fid3,[num2str(optim.wing.aero.ASWINGpolar.nNodes) '\n']);
end
 
% set iterations limit
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'oper\n');
fprintf(fid3,'K\n');
fprintf(fid3,'I\n');
fprintf(fid3,'40\n'); % set number of iteration allowable
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
 
% set to body axis
fprintf(fid3,'oper\n'); 
fprintf(fid3,'K\n');
fprintf(fid3,'C\n');
fprintf(fid3,'0\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

% Set Constraints to SLF
fprintf(fid3,'pget %sSLF.pnt\n', optim.ASWINGIODir);
fprintf(fid3,'\n'); 
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

% turn off graphics
 fprintf(fid3,'plpa\n');
 fprintf(fid3,'G T\n');
 fprintf(fid3,'\n');
 fprintf(fid3,'\n');

fprintf(fid3,'oper\n');
fprintf(fid3,'%%\n');
fprintf(fid3,'19 19\n');
fprintf(fid3,'21 21\n');

% turn off graphics
 fprintf(fid3,'plpa\n');
 fprintf(fid3,'G F\n');
 fprintf(fid3,'\n');
 fprintf(fid3,'\n');

% Set Operating Point
fprintf(fid3,'oper\n');  
fprintf(fid3,'!\n'); 
fprintf(fid3,'V\n');
fprintf(fid3,'\t%3.3f\n',  sqrt(optim.MGTOW_kg*optim.constants.g_ms2 *2/ ...
                                (optim.constants.rhoSL_kgm3*optim.aircraft.aero.CLcruise * optim.aircraft.Sref_m2)));
fprintf(fid3,'\n'); 
fprintf(fid3,'X\n'); 
 
% write force derivatives matrix to file 
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
 

%% exit
fprintf(fid3,'quit\n');
fclose(fid3);
 
end
