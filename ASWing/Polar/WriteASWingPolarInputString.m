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
  
function [ output_args ] = WriteASWingPolarInputString( optim )
%The function writes the input string used to generate the aircraft polar
%% Write ASWing input string 
fid3 = fopen([optim.ASWINGIODir 'PolarInputString' num2str(optim.designID) '.txt'],'wt');
fprintf(fid3,'plpa\n');
fprintf(fid3,'G F\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

% Load Input File
fprintf(fid3,'load %sPolarAircraft%i.asw\n', optim.ASWINGIODir, optim.designID);
fprintf(fid3,'\n'); 
fprintf(fid3,'\n');
fprintf(fid3,'\n'); 
fprintf(fid3,'\n');

% Set Nodes
SetNodes(optim, fid3 );

% Set Freeze VL matrices to false
fprintf(fid3,'\n'); 
fprintf(fid3,'\n');
fprintf(fid3,'\n'); 
fprintf(fid3,'\n');
fprintf(fid3,'oper\n');
fprintf(fid3,'k\n');
fprintf(fid3,'f\n');
fprintf(fid3,'\n'); 
fprintf(fid3,'\n');
fprintf(fid3,'\n'); 
fprintf(fid3,'\n');

% Set custom terminal outputs
fprintf(fid3,'oper\n');
fprintf(fid3,'k\n');
fprintf(fid3,'v\n');
% Remove all defulat outputs
for i = 1:length(optim.aircraft.aero.ASWINGpolar.defaultOutputNumbers)
   fprintf(fid3,'%3.3f\n',optim.aircraft.aero.ASWINGpolar.defaultOutputNumbers(i));
end
% Turn on all desired outputs
for i = 1:length(optim.aircraft.aero.ASWINGpolar.outputColumNumbers)
   fprintf(fid3,'%3.3f\n',optim.aircraft.aero.ASWINGpolar.outputColumNumbers(i));
end
fprintf(fid3,'\n'); 
fprintf(fid3,'\n');
fprintf(fid3,'\n'); 
fprintf(fid3,'\n');

% Load pnt file
fprintf(fid3,'pget %sSLF.pnt\n', optim.ASWINGIODir);
fprintf(fid3,'\n'); 
fprintf(fid3,'\n');
fprintf(fid3,'\n'); 
fprintf(fid3,'\n');
fprintf(fid3,'oper\n');
fprintf(fid3,'init\n');
fprintf(fid3,'X\n'); 
fprintf(fid3,'R\n');
fprintf(fid3,'\n'); 
fprintf(fid3,'\n');

% Set Constraints 
fprintf(fid3,'\n');
fprintf(fid3,'\n'); 
fprintf(fid3,'\n');
fprintf(fid3,'pget\n');
fprintf(fid3,'%sPolarV.pnt\n', optim.ASWINGIODir);

% Set Iteration Limit
fprintf(fid3,'oper\n');
fprintf(fid3,'K\n');
fprintf(fid3,'I\n');
fprintf(fid3,'40\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

% Set Sensor Output Values
fprintf(fid3,'oper\n');
fprintf(fid3,'P\n');
fprintf(fid3,'S\n');
fprintf(fid3,'68 75 78 19 20 21\n'); %Select to print vehcile CDi and CL
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

fprintf(fid3,'oper\n');
fprintf(fid3,'init\n');

% Now set file loaded. Default used for polat.
V_ms = optim.aircraft.aero.ASWINGpolar.V_ms;
fprintf(fid3,'!V %3.3f\n',V_ms(1));

% Loop through remaining alphas
j = 2;
while j <= length(V_ms) 
    fprintf(fid3,'+\n',V_ms(j));
    fprintf(fid3,'!V %3.3f\n',V_ms(j));
    
    j = j+1;
end 
fprintf(fid3,'XX\n');
fprintf(fid3,'P\n');

% Write output file
fprintf(fid3,'W\n');
fprintf(fid3,'%sPolarOutputs%i.txt\n', optim.ASWINGIODir, optim.designID);
fprintf(fid3,'overwrite\n');


%%
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'quit\n');
fclose(fid3);


end

