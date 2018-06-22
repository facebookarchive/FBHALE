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
  
function [ optim ] = SetNodes(optim, printFile )
% Specify aero and structural nodes.

currentBeamNumber = 1;

fprintf(printFile,'\n');
fprintf(printFile,'\n');


for i = 1:length(optim.aircraft.beamNames)
    % If pod then don't do anything
    for n = 1:sum(optim.(optim.aircraft.beamNames{i}).N)
        if strcmp(optim.aircraft.beamNames{i}, 'pod') || strcmp(optim.aircraft.beamNames{i}, 'boom') 
            currentBeamNumber = currentBeamNumber + 1;
        else
            fprintf(printFile,'node\n');
            fprintf(printFile,['B ' num2str(currentBeamNumber) '\n']);
            fprintf(printFile,'I\n');
            fprintf(printFile,[num2str(optim.(optim.aircraft.beamNames{i}).aero.ASWINGpolar.nNodes) '\n']);
            fprintf(printFile,'\n');
            fprintf(printFile,'\n');
            currentBeamNumber = currentBeamNumber + 1;
        end
    end
end

fprintf(printFile,'\n');
fprintf(printFile,'\n');
fprintf(printFile,'\n');
fprintf(printFile,'\n');
end
