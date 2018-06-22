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

function [ output_args ] = EmpennageControlSurfaceDeflectionLoadCasesInputString( optim )
% Run aircraft through deflection of each empennage control surface.

% create file
fid3 = fopen([optim.ASWINGIODir 'EmpanngeDeflectioLoadsInput' num2str(optim.designID) '.txt'],'wt');

if ~isfield(optim, 'htail') || ~isfield(optim, 'vtail')
    % write nothing if no empennage exists
else
    % turn off graphics
    fprintf(fid3,'plpa\n');
    fprintf(fid3,'G F\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    
    % find response time
    optim.environment.ControlSurfaceDeflection.n_steps  = ceil(optim.wing.cref_m/optim.wing.vn.vEasTopRightCorner_ms*10/optim.environment.ControlSurfaceDeflection.dt_s);
    
    % load aircraft file
    fprintf(fid3,'load %sLoadsAircraft%i.asw\n', optim.ASWINGIODir, optim.designID);
    SetNodes(optim, fid3 );
    
    % set interation limit
    fprintf(fid3,'oper\n');
    fprintf(fid3,'K\n');
    fprintf(fid3,'I\n');
    fprintf(fid3,'40\n'); % set number of iteration allowable
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'\n');
    
    % select sensor outputs
    fprintf(fid3,'oper\n');
    fprintf(fid3,'P\n');
    fprintf(fid3,'S\n');
    fprintf(fid3,'\n');
    fprintf(fid3,'22 0\n'); % Fc
    fprintf(fid3,'23 0\n'); % Fc
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
    
    
    
    % loop through all surfaces on vtail
    for i = 1:length(optim.vtail.controlSurface)
        controlSurfaceMapping = find(optim.vtail.controlSurface(i).schedule);
        for n = 1:length(controlSurfaceMapping)
            % load pnt file
            fprintf(fid3,'pget\n');
            fprintf(fid3,'%sSLF.pnt\n', optim.ASWINGIODir);
            
            % set up trimmed slf
            fprintf(fid3,'oper\n');
            fprintf(fid3,'init\n');
            fprintf(fid3,'!\n');
            fprintf(fid3,'L\n');
            fprintf(fid3,'\t%3.3f\n', optim.MGTOW_kg*9.81);
            fprintf(fid3,'\n');
            fprintf(fid3,'!\n');
            fprintf(fid3,'V\n');
            fprintf(fid3,'\t%3.3f\n', optim.wing.vn.vEasTopRightCorner_ms);
            fprintf(fid3,'\n');
            fprintf(fid3,'X\n');
            fprintf(fid3,'R\n');
            
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
            % Set up rudder deflection case
            fprintf(fid3,'oper\n');
            fprintf(fid3,'.\n');
            fprintf(fid3,'!\n');
            fprintf(fid3,['F' num2str(controlSurfaceMapping(n)) '\n']);
            fprintf(fid3,'\t%3.3f\n', optim.vtail.controlSurface(i).maxDeflection_deg);
            fprintf(fid3,'\n');
            % run and save
            fprintf(fid3,'XX\n');
            fprintf(fid3, '%3.3f', optim.environment.ControlSurfaceDeflection.dt_s);
            fprintf(fid3,' ');
            fprintf(fid3, '\t%3.3f\n', optim.environment.ControlSurfaceDeflection.n_steps); %set time step and num steps
            fprintf(fid3,'P\n');
            fprintf(fid3,'W\n');
            fprintf(fid3,'%sVtailDeflectionLoads%i.txt\n', optim.ASWINGIODir, optim.designID);
            fprintf(fid3,'Append\n');
            fprintf(fid3,'\n');
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
    
    % Elevator
    fprintf(fid3,'load %sLoadsAircraft%i.asw\n', optim.ASWINGIODir, optim.designID);
    SetNodes(optim, fid3 );
    
    
    
    % loop through all surfaces on htail
    for i = 1:length(optim.htail.controlSurface)
        controlSurfaceMapping = find(optim.htail.controlSurface(i).schedule);
        for n = 1:length(controlSurfaceMapping)
            fprintf(fid3,'pget\n');
            fprintf(fid3,'%sSLF.pnt\n', optim.ASWINGIODir);
            
            %Set up trimmed slf
            fprintf(fid3,'oper\n');
            fprintf(fid3,'init\n');
            fprintf(fid3,'!\n');
            fprintf(fid3,'L\n');
            fprintf(fid3,'\t%3.3f\n', optim.MGTOW_kg*9.81);
            fprintf(fid3,'\n');
            fprintf(fid3,'!\n');
            fprintf(fid3,'V\n');
            fprintf(fid3,'\t%3.3f\n', optim.wing.vn.vEasTopRightCorner_ms);
            fprintf(fid3,'\n');
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
            fprintf(fid3,'!\n');
            fprintf(fid3,['F' num2str(controlSurfaceMapping(n)) '\n']);
            fprintf(fid3,'\t%3.3f\n', optim.htail.controlSurface(i).maxDeflection_deg);
            fprintf(fid3,'\n');
            
            % run and save
            fprintf(fid3,'XX\n');
            fprintf(fid3, '%3.3f', optim.environment.ControlSurfaceDeflection.dt_s);
            fprintf(fid3,' ');
            fprintf(fid3, '\t%3.3f\n', optim.environment.ControlSurfaceDeflection.n_steps); %set time step and num steps
            fprintf(fid3,'P\n');
            fprintf(fid3,'W\n');
            fprintf(fid3,'%sHtailDeflectionLoads%i.txt\n', optim.ASWINGIODir, optim.designID);
            fprintf(fid3,'append\n');
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

