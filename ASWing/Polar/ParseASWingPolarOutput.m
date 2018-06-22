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
  
function [ RAWASWingPolarData ] = ParseASWingPolarOutput( filename, optim )
% Parse output files for polar.
fileID = fopen(filename,'r');

polarRun = -1;
recordData = false;
RAWASWingPolarData = [];
internalSwitching = [];
while 1
    tline = fgetl(fileID);
    %if you have reached the end of the file break
    if tline == -1
        break
    end
    
    record = strfind(tline, ['Converging Point           1  ...']);
    if ischar(tline)
        if isfinite(record)
            polarRun = polarRun+1;
        end
        
        % if we are to the polar run then look for values to parse
        if polarRun == 1
            
            currentLineIsBeamName = strfind(tline, 'Surface  Beam') ;
            
            %If its a reflected index then don't record
            if any(isfinite(currentLineIsBeamName)) && ~any(strfind(tline, 'Reflect'))
                index = strfind(tline, ':');
                entityname  = strtrim(tline(index+2:end));
                
                
                if ~isfield(RAWASWingPolarData, entityname)
                    RAWASWingPolarData.(entityname) = [];
                    entityWrite.(entityname) = 0;
                    internalSwitching.(entityname).numProcessed = 1;
                    
                end
                internalSwitching.(entityname).numProcessed = internalSwitching.(entityname).numProcessed+1;
                instanceNumber = mod(internalSwitching.(entityname).numProcessed, length(optim.(entityname).N));
                if instanceNumber == 0
                    instanceNumber = length(optim.(entityname).N);
                end
                
                %Create output matrix for current entity
                CurrentEntityOutputMatrix = [];
                
                % step 4 lines down to where data begins
                tline = fgetl(fileID);
                tline = fgetl(fileID);
                tline = fgetl(fileID);
                tline = fgetl(fileID);
                
                % Set Variable to tell downstream to record values if its
                % the first instance of this surface at this alpha
                if mod(entityWrite.(entityname),length(optim.(entityname).N)) == 0
                    recordData = true;
                end
                entityWrite.(entityname) = entityWrite.(entityname) + 1;
                
            end
            % If you have reached the end then stop recording and crunch
            % output numbers.
            isEndOfCurrentEntity = strfind(tline, ' .............................................................');
            if all( any(isfinite(isEndOfCurrentEntity)) && recordData)
                recordData = false;
                % if output data struct is not created then create
                for i = 1:length(optim.aircraft.aero.ASWINGpolar.outputColumNumbers)
                    
                        
                        if ~isfield(RAWASWingPolarData.(entityname),optim.aircraft.aero.ASWINGpolar.outputColumNames{i})
                            RAWASWingPolarData.(entityname).(optim.aircraft.aero.ASWINGpolar.outputColumNames{i}){instanceNumber} = CurrentEntityOutputMatrix(:,i+1)';
                        else
                            if length(RAWASWingPolarData.(entityname).(optim.aircraft.aero.ASWINGpolar.outputColumNames{i}){instanceNumber}) == length( CurrentEntityOutputMatrix(:,i+1))
                                RAWASWingPolarData.(entityname).(optim.aircraft.aero.ASWINGpolar.outputColumNames{i}){instanceNumber} = ...
                                                        [RAWASWingPolarData.(entityname).(optim.aircraft.aero.ASWINGpolar.outputColumNames{i}){instanceNumber}; CurrentEntityOutputMatrix(:,i+1)'];
                            else %insert NaNs
                                RAWASWingPolarData.(entityname).(optim.aircraft.aero.ASWINGpolar.outputColumNames{i}){instanceNumber} = ...
                                                        [RAWASWingPolarData.(entityname).(optim.aircraft.aero.ASWINGpolar.outputColumNames{i}){instanceNumber}; NaN(1,length(RAWASWingPolarData.(entityname).(optim.aircraft.aero.ASWINGpolar.outputColumNames{i}){instanceNumber}))];
                            end
                        end
                    end
                   %Record t
            if ~isfield(RAWASWingPolarData.(entityname), 't')
                RAWASWingPolarData.(entityname).t = CurrentEntityOutputMatrix(:,1)';
            end      
            end
            % Record data
            if recordData      
                %CurrentEntityOutputMatrix = [CurrentEntityOutputMatrix; str2num(insertBefore(tline,'-','  '))];
                CurrentEntityOutputMatrix = [CurrentEntityOutputMatrix; str2num(tline)];  
            end
        end
end
end
end

