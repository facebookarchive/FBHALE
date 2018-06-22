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
  
function [] = WriteAeroCoefficientsInputString(optim)
% Write input string for RunASWINGAeroCoefficient

% Open file
fid3 = fopen([optim.ASWINGIODir 'CnBetaInputString' num2str(optim.designID) '.txt'],'wt');
 
% load aircraft file
fprintf(fid3, 'load %sCnBetaAircraft%i.asw\n', optim.ASWINGIODir, optim.designID);

% set wing nodes
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

% Set Constraints to anchored
fprintf(fid3,'oper\n'); 
fprintf(fid3,'%%\n');
fprintf(fid3,'A\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

% Set Operating Point
fprintf(fid3,'oper\n'); 
fprintf(fid3,'!\n'); 
fprintf(fid3,'A\n');
fprintf(fid3,'\t%3.3f\n',  optim.aircraft.aero.alphaCruise_deg);
fprintf(fid3,'\n'); 
fprintf(fid3,'!\n'); 
fprintf(fid3,'V\n');
fprintf(fid3,'\t%3.3f\n',  optim.wing.vn.vCruise_ms);
fprintf(fid3,'\n'); 
 
fprintf(fid3,'X\n'); 
 
% write force derivatives matrix to file 
fprintf(fid3,'M\n');
fprintf(fid3, '%sForceDerivativesMatrix_Vc%i.txt\n', optim.ASWINGIODir, optim.designID);
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
 
%%
fprintf(fid3, 'load %sCnBetaAircraft%i.asw\n', optim.ASWINGIODir, optim.designID);
SetNodes(optim, fid3 );
 
% set to body axis
fprintf(fid3,'oper\n'); 
fprintf(fid3,'K\n');
fprintf(fid3,'C\n');
fprintf(fid3,'0\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

% set Constraints to anchored
fprintf(fid3,'oper\n'); 
fprintf(fid3,'%%\n');
fprintf(fid3,'A\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

% set operating point
fprintf(fid3,'oper\n'); 
fprintf(fid3,'!\n'); 
fprintf(fid3,'A\n');
fprintf(fid3,'\t%3.3f\n',  optim.wing.aero.alpha_deg_VD(1));
fprintf(fid3,'\n'); 
fprintf(fid3,'!\n'); 
fprintf(fid3,'V\n');
fprintf(fid3,'\t%3.3f\n',  optim.wing.vn.vEasTopRightCorner_ms);
fprintf(fid3,'\n'); 
 
fprintf(fid3,'X\n'); 
 
% write force derivatives matrix to file 
fprintf(fid3,'M\n');
fprintf(fid3, '%sForceDerivativesMatrix_Vd%i.txt\n', optim.ASWINGIODir, optim.designID);
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
 
 
%% Aileron deflection at Vstall

fprintf(fid3, 'load %sCnBetaAircraftWithTails%i.asw\n', optim.ASWINGIODir, optim.designID);
 
SetNodes(optim, fid3 );
 
% set location to take moments around
% set to body axis
fprintf(fid3,'oper\n'); 
fprintf(fid3,'K\n');
fprintf(fid3,'C\n');
fprintf(fid3,'0\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
 
% set constraints
fprintf(fid3,'\n');
fprintf(fid3,'\n'); 
fprintf(fid3,'oper\n');
fprintf(fid3,'%%\n');
fprintf(fid3,'S\n'); 
fprintf(fid3,'7 7\n'); 
fprintf(fid3,'8 8\n');
fprintf(fid3,'11 11\n');
fprintf(fid3,'12 12\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n'); 
fprintf(fid3,'\n');
fprintf(fid3,'\n'); 
 
% set operating conditions
fprintf(fid3,'oper\n');
fprintf(fid3,'!F1\n');
fprintf(fid3,[num2str(optim.(optim.aircraft.controlSurfaces.roll.surfaceName). ...
    controlSurface(1).maxDeflection_deg) '\n']); 
fprintf(fid3,'!\n'); 
fprintf(fid3,'A\n');
fprintf(fid3,'0\n');
fprintf(fid3,'\n'); 
fprintf(fid3,'!\n'); 
fprintf(fid3,'V\n');
fprintf(fid3,'\t%3.3f\n',  sqrt(optim.MGTOW_kg * optim.constants.g_ms2 * 2 /(optim.wing.aero.CLmax * optim.constants.rhoSL_kgm3 * optim.wing.Sref_m2))*sqrt(1.05));
fprintf(fid3,'\n'); 
 
fprintf(fid3,'X\n'); 
 
% write force derivatives matrix to file 
fprintf(fid3,'M\n');
fprintf(fid3, '%sForceDerivativesMatrix_VS_AileronDeflected%i.txt\n', optim.ASWINGIODir, optim.designID);
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
 
%% Aileron deflection at Vd
fprintf(fid3, 'load %sCnBetaAircraftWithTails%i.asw\n', optim.ASWINGIODir, optim.designID);
 
SetNodes(optim, fid3 );
 
% set to body axis
fprintf(fid3,'oper\n'); 
fprintf(fid3,'K\n');
fprintf(fid3,'C\n');
fprintf(fid3,'0\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
 
% set constraints
fprintf(fid3,'\n');
fprintf(fid3,'\n'); 
fprintf(fid3,'oper\n');
fprintf(fid3,'%%\n');
fprintf(fid3,'S\n'); 
fprintf(fid3,'7 7\n'); 
fprintf(fid3,'8 8\n');
fprintf(fid3,'11 11\n');
fprintf(fid3,'12 12\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n'); 
fprintf(fid3,'\n');
fprintf(fid3,'\n'); 

% set operating conditions
fprintf(fid3,'oper\n');
fprintf(fid3,'!F1\n');
fprintf(fid3,[num2str(optim.(optim.aircraft.controlSurfaces.roll.surfaceName). ...
    controlSurface(1).maxDeflection_deg) '\n']); 
fprintf(fid3,'!\n'); 
fprintf(fid3,'A\n');
fprintf(fid3,'0\n');
fprintf(fid3,'\n'); 
fprintf(fid3,'!\n'); 
fprintf(fid3,'V\n');
fprintf(fid3,'\t%3.3f\n',  optim.wing.vn.vEasTopRightCorner_ms);
fprintf(fid3,'\n'); 
 
fprintf(fid3,'X\n'); 
 
% write force derivatives matrix to file 
fprintf(fid3,'M\n');
fprintf(fid3, '%sForceDerivativesMatrix_VD_AileronDeflected%i.txt\n', optim.ASWINGIODir, optim.designID);
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
 
%% exit
fprintf(fid3,'quit\n');
fclose(fid3);
 
end
