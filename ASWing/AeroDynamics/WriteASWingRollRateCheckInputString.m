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
  
function [] = WriteASWingRollRateCheckInputString( optim )
% Write input string for RunASWINGRollRateCheck

%create file
fid3 = fopen([optim.ASWINGIODir 'CheckRollRateInputString' num2str(optim.designID) '.txt'],'wt');

%turn off graphics
fprintf(fid3,'plpa\n');
fprintf(fid3,'G\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

% load aircraft file
fprintf(fid3,'load %sPolarAircraft%i.asw\n', optim.ASWINGIODir, optim.designID);
SetNodes(optim, fid3 );

% set iterations limit
fprintf(fid3,'oper\n');
fprintf(fid3,'K\n');
fprintf(fid3,'I\n');
fprintf(fid3,'40\n'); % set number of iterations allowable
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

fprintf(fid3,'pget %sSLF.pnt\n', optim.ASWINGIODir);

% Select sensor outputs
fprintf(fid3,'\n');
fprintf(fid3,'\n');

fprintf(fid3,'oper\n');
fprintf(fid3,'init\n');
vStall_ms = sqrt(optim.MGTOW_kg * optim.constants.g_ms2 * 2 /(optim.wing.aero.CLmax * optim.constants.rhoSL_kgm3 * optim.wing.Sref_m2))*sqrt(1.1); %sqrt(1.05) to scale CL to 10% above CLstall
fprintf(fid3,'!V\n');
fprintf(fid3,'\t%3.4f\n',vStall_ms);
fprintf(fid3,'X\n');
fprintf(fid3,'R\n');

if strcmp(optim.aircraft.controlSurfaces.roll.surfaceName, 'htail')
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'oper\n');
    fprintf(fid3,'%%\n');
    fprintf(fid3,'21 21\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'!Wx\n');
    fprintf(fid3,'\t%3.4f\n',optim.aircraft.controlSurfaces.roll.requiredRollRate*vStall_ms/(optim.wing.bref_m/2*pi/180));

else
    % turn on graphics
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'plpa\n');
    fprintf(fid3,'G\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    
    % set constraints
    fprintf(fid3,'oper\n');
    fprintf(fid3,'%%\n');
    fprintf(fid3,'19 19\n');
    fprintf(fid3,'20 20\n');
    fprintf(fid3,'21 21\n');
    fprintf(fid3,'22 22\n');
    fprintf(fid3,'1 1\n');
    fprintf(fid3,'2 2\n');
    fprintf(fid3,'2 2\n');
    fprintf(fid3,'3 3\n');
    fprintf(fid3,'4 4\n');
    fprintf(fid3,'5 5\n');
    fprintf(fid3,'6 6\n');
    fprintf(fid3,'7 7\n');
    fprintf(fid3,'8 8\n');
    fprintf(fid3,'9 25\n');
    fprintf(fid3,'10 26\n');
    fprintf(fid3,'11 27\n');
    fprintf(fid3,'12 12\n');
    fprintf(fid3,'13 0\n');
    fprintf(fid3,'14 0\n');
    fprintf(fid3,'15 0\n');
    fprintf(fid3,'16 0\n');
    fprintf(fid3,'17 0\n');
    fprintf(fid3,'18 0\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');

    % turn off graphics
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'plpa\n');
    fprintf(fid3,'G\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');

    % set operating point
    fprintf(fid3,'oper\n');
    fprintf(fid3,'init\n');
    fprintf(fid3,'!F1\n');
    fprintf(fid3,'\t%3.4f\n',optim.(optim.aircraft.controlSurfaces.roll.surfaceName).controlSurface(optim.aircraft.controlSurfaces.roll.contolSurfaceIndex).maxDeflection_deg);
end

fprintf(fid3,'X\n');
fprintf(fid3,'M\n');
fprintf(fid3,'%sRollRateDataVSTALL%i.txt\n', optim.ASWINGIODir, optim.designID);
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

% Move to Vne
fprintf(fid3,'pget %sSLF.pnt\n', optim.ASWINGIODir);

fprintf(fid3,'oper\n');
fprintf(fid3,'init\n');
fprintf(fid3,'!V\n');
fprintf(fid3,'\t%3.4f\n',optim.wing.vn.vEasTopRightCorner_ms);
fprintf(fid3,'X\n');
fprintf(fid3,'R\n');

if strcmp(optim.aircraft.controlSurfaces.roll.surfaceName, 'htail')
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
 	fprintf(fid3,'oper\n');
    fprintf(fid3,'%%\n');
    fprintf(fid3,'21 21\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'!Wx\n');
    fprintf(fid3,'\t%3.4f\n',optim.aircraft.controlSurfaces.roll.requiredRollRate*optim.wing.vn.vEasTopRightCorner_ms/(optim.wing.bref_m/2*pi/180));
       
else
     % turn on graphics
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'plpa\n');
    fprintf(fid3,'G\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    
    % set constraints
    fprintf(fid3,'oper\n');
    fprintf(fid3,'%%\n');
    fprintf(fid3,'19 19\n');
    fprintf(fid3,'20 20\n');
    fprintf(fid3,'21 21\n');
    fprintf(fid3,'22 22\n');
    fprintf(fid3,'1 1\n');
    fprintf(fid3,'2 2\n');
    fprintf(fid3,'2 2\n');
    fprintf(fid3,'3 3\n');
    fprintf(fid3,'4 4\n');
    fprintf(fid3,'5 5\n');
    fprintf(fid3,'6 6\n');
    fprintf(fid3,'7 7\n');
    fprintf(fid3,'8 8\n');
    fprintf(fid3,'9 25\n');
    fprintf(fid3,'10 26\n');
    fprintf(fid3,'11 27\n');
    fprintf(fid3,'12 12\n');
    fprintf(fid3,'13 0\n');
    fprintf(fid3,'14 0\n');
    fprintf(fid3,'15 0\n');
    fprintf(fid3,'16 0\n');
    fprintf(fid3,'17 0\n');
    fprintf(fid3,'18 0\n');
    
    % turn off graphics
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'plpa\n');
    fprintf(fid3,'G\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    
    % set operating point
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'oper\n');
    fprintf(fid3,'init\n');
    fprintf(fid3,'!F1\n');
    fprintf(fid3,'\t%3.4f\n',optim.(optim.aircraft.controlSurfaces.roll.surfaceName).controlSurface(optim.aircraft.controlSurfaces.roll.contolSurfaceIndex).maxDeflection_deg);
    
end

fprintf(fid3,'X\n');
fprintf(fid3,'M\n');
fprintf(fid3,'%sRollRateDataVD%i.txt\n', optim.ASWINGIODir, optim.designID);
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

%% exit
fprintf(fid3,'quit\n');
fclose(fid3);
end

