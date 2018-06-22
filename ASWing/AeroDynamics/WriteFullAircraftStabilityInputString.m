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
  
function [ optim ] = WriteFullAircraftStabilityInputString( optim )
% ASWING input string for full vehicle polar.
% Runs a fully flexible aircraft to find stability around Xcg and Zcg
% reference point.

%% Set up trimmed cruise point to find vehicle Cma and Cnb
fid3 = fopen([optim.ASWINGIODir 'FullVehicleStabilityInputString' num2str(optim.designID) '.txt'],'wt');

% turn off graphics
fprintf(fid3,'plpa\n');
fprintf(fid3,'G F\n');

fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'load %sPolarAircraft%i.asw\n', optim.ASWINGIODir, optim.designID);

fprintf(fid3,'pget %sSLF.pnt\n', optim.ASWINGIODir);

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

% Set Operating Point
fprintf(fid3,'oper\n'); 
fprintf(fid3,'!\n'); 
fprintf(fid3,'V\n');
fprintf(fid3,'\t%3.3f\n',  optim.wing.vn.vCruise_ms);
fprintf(fid3,'\n'); 

fprintf(fid3,'X\n');

% write force derivatives matrix to file 
fprintf(fid3,'M\n');
fprintf(fid3,'%sForceDerivativesMatrixFullAircraft_Vc%i.txt\n', optim.ASWINGIODir, optim.designID);
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

% save set file for recall with rigid boom
fprintf(fid3,'ssav\n');
fprintf(fid3,'%sPolarAircraft%i.set\n', optim.ASWINGIODir, optim.designID);
fprintf(fid3,'Y\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

fprintf(fid3,'quit\n');
fclose(fid3);

end

