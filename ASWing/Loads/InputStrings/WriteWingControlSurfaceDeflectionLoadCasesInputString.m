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

function [ output_args ] = WriteWingControlSurfaceDeflectionLoadCasesInputString( optim)
% This function loops through the control surfaces on the wing and writes
% the input string to run the deflected load case for each one.

% create file
fid3 = fopen([optim.ASWINGIODir 'WingControlSurfaceImpulseInputString' num2str(optim.designID) '.txt'],'wt');

% turn off graphics
fprintf(fid3,'plpa\n');
fprintf(fid3,'G F\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

% load aircraft file
fprintf(fid3,'load %sLoadsAircraft%i.asw\n', optim.ASWINGIODir, optim.designID);
SetNodes(optim, fid3 );

% set iterations limit
fprintf(fid3,'oper\n');
fprintf(fid3,'K\n');
fprintf(fid3,'I\n');
fprintf(fid3,'40\n'); % set number of iteration allowable
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');

% Select sensor outputs
fprintf(fid3,'oper\n');
fprintf(fid3,'P\n');
fprintf(fid3,'S\n');
fprintf(fid3,'\n');
fprintf(fid3,'22 0\n'); % Fc
fprintf(fid3,'23 0\n'); % Fs
fprintf(fid3,'24 0\n'); % Fn
fprintf(fid3,'25 0\n'); % Mc
fprintf(fid3,'26 0\n'); % Ms
fprintf(fid3,'27 0\n'); % Mn
fprintf(fid3,'28 0\n'); % rx_m
fprintf(fid3,'29 0\n'); % ry_m
fprintf(fid3,'30 0\n'); % rz_m
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');
fprintf(fid3,'\n');


% if there are no control surfaces on the wing the run roll case else
% deflect each wing control surface
if ~isfield(optim.wing, 'controlSurface')
    fprintf(fid3,'pget\n');
    fprintf(fid3,'%sSLF.pnt\n', optim.ASWINGIODir);
    
    %Set up trimmed SLF
    fprintf(fid3,'oper\n');
    fprintf(fid3,'init\n');
    fprintf(fid3,'!L\n');
    fprintf(fid3,'\t%3.3f\n', optim.MGTOW_kg*9.81);
    fprintf(fid3,'!V\n');
    fprintf(fid3,'\t%3.3f\n', optim.wing.vn.vCruise_ms);
    fprintf(fid3,'X\n');
    fprintf(fid3,'R\n');
    
    % Turn on graphics
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
    
    % set constraints for surface deflection
    fprintf(fid3,'oper\n');
    fprintf(fid3,'%%\n');
    fprintf(fid3,'9 9\n');
    fprintf(fid3,'17 17\n');
    fprintf(fid3,'19 19\n');
    fprintf(fid3,'20 20\n');
    fprintf(fid3,'21 21\n');
    fprintf(fid3,'22 22\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    
    % Turn off graphics
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
    
    fprintf(fid3,'oper\n');
    fprintf(fid3,'.\n');
    fprintf(fid3,['!F1\n']);
    fprintf(fid3,'\t%3.3f\n', optim.(optim.aircraft.controlSurfaces.roll.surfaceName).controlSurface(optim.aircraft.controlSurfaces.roll.contolSurfaceIndex).maxDeflection_deg);
    
    % run and save
    fprintf(fid3,'XX\n');
    fprintf(fid3, '%3.3f', optim.environment.ControlSurfaceDeflection.dt_s);
    fprintf(fid3,' ');
    fprintf(fid3, '\t%3.3f\n', optim.environment.ControlSurfaceDeflection.n_steps); %set time step and num steps
    fprintf(fid3,'P\n');
    fprintf(fid3,'W\n');
    fprintf(fid3,'%sLoadsOutput2%i.txt\n', optim.ASWINGIODir, optim.designID);
    fprintf(fid3,'Append\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    
    % reset all control surfaces back to 0 deflection
    fprintf(fid3,'oper\n');
    fprintf(fid3,'init\n');
    fprintf(fid3,'!F1\n');
    fprintf(fid3,'0\n');
    fprintf(fid3,'!F2\n');
    fprintf(fid3,'0\n');
    fprintf(fid3,'!F3\n');
    fprintf(fid3,'0\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    
else
    for i = 1:length(optim.wing.controlSurface)
        controlSurfaceMapping = find(optim.wing.controlSurface(i).schedule);
        for n = 1:length(controlSurfaceMapping)
            fprintf(fid3,'pget\n');
            fprintf(fid3,'%sSLF.pnt\n', optim.ASWINGIODir);
            
            %Set up trimmed slf
            fprintf(fid3,'oper\n');
            fprintf(fid3,'init\n');
            fprintf(fid3,'!L\n');
            fprintf(fid3,'\t%3.3f\n', optim.MGTOW_kg*9.81);
            fprintf(fid3,'!V\n');
            fprintf(fid3,'\t%3.3f\n', optim.wing.vn.vCruise_ms);
            fprintf(fid3,'X\n');
            fprintf(fid3,'R\n');
            
            % Turn on graphics
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
            
            % set constraints for surface deflection
            fprintf(fid3,'oper\n');
            fprintf(fid3,'%%\n');
            fprintf(fid3,'9 9\n');
            fprintf(fid3,'17 17\n');
            fprintf(fid3,'19 19\n');
            fprintf(fid3,'20 20\n');
            fprintf(fid3,'21 21\n');
            fprintf(fid3,'22 22\n');
            fprintf(fid3,'\n');
            fprintf(fid3,'\n');
            fprintf(fid3,'\n');
            fprintf(fid3,'\n');
            
            % Turn off graphics
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
            
            % reinitialize aircraft
            fprintf(fid3,'oper\n');
            fprintf(fid3,'.\n');
            fprintf(fid3,['!F' num2str(controlSurfaceMapping(n)) '\n']);
            fprintf(fid3,'\t%3.3f\n', optim.wing.controlSurface(i).maxDeflection_deg);
            
            % run and save
            fprintf(fid3,'XX\n');
            fprintf(fid3, '%3.3f', optim.environment.ControlSurfaceDeflection.dt_s);
            fprintf(fid3,' ');
            fprintf(fid3, '\t%3.3f\n', optim.environment.ControlSurfaceDeflection.n_steps); %set time step and num steps
            fprintf(fid3,'P\n');
            fprintf(fid3,'W\n');
            fprintf(fid3,'%sLoadsOutput2%i.txt\n', optim.ASWINGIODir, optim.designID);
            fprintf(fid3,'Append\n');
            fprintf(fid3,'\n');
            fprintf(fid3,'\n');
            fprintf(fid3,'\n');
            
            % reset all control surfaces back to 0 deflection
            fprintf(fid3,'oper\n');
            fprintf(fid3,'!F1\n');
            fprintf(fid3,'0\n');
            fprintf(fid3,'!F2\n');
            fprintf(fid3,'0\n');
            fprintf(fid3,'!F3\n');
            fprintf(fid3,'0\n');
            fprintf(fid3,'\n');
            fprintf(fid3,'\n');
            fprintf(fid3,'\n');
            
        end
    end
end

fprintf(fid3,'quit\n');
fclose(fid3);



end

