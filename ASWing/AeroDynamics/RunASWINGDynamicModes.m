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
  
function [ optim ] = RunASWINGDynamicModes( optim, useStructure)
% This function finds the dynamic modes of the aircraft across its
% operating altitude range. For each mode with an imaginary part greater
% than half of the minimum structural natural frequency this function
% outputs the real part and the damping term. This function also populates
% the minimum damping ratio for the vehicle. 

%% Run ASWING
% Write Aircraft ASW File
optim = WriteASWingFile(optim, [optim.ASWINGIODir 'DynamicModesAircraft' num2str(optim.designID) '.asw'] , 'dynamicModes', useStructure);

% Write the ASWING input string
WriteDynamicModesInput(optim)

% Run ASWING input string
system(['aswing<' optim.ASWINGIODir 'DynamicModesInputString' num2str(optim.designID) '.txt>' GetNullDevice() ' 2>&1']); %>nul

%% Parse Outputs
% Gather eigen mode files
files = dir([optim.ASWINGIODir '*.e00']);

realParts       = [];
imaginaryParts  = [];

% parse and store outputs
for i = 1:length(files)
    %parse outputfiles
    output = ParseEigenModes(files(i).name);
    realParts = [realParts;output(:,2)];
    imaginaryParts = [imaginaryParts;output(:,3)];
end

% Remove modes with imaginary parts with a magnitude less then the minimum
% natural frequency
realParts       = realParts(abs(imaginaryParts)>optim.wing.structure.minNaturalFreq_rad_s/2);
imaginaryParts  = imaginaryParts(abs(imaginaryParts)>optim.wing.structure.minNaturalFreq_rad_s/2);
dampingRatios   = cos(atan(abs(imaginaryParts./realParts))).*sign(-realParts);

% Record results to optim structure
[~, index] = max(realParts);
optim.wing.eigenModes.realPart     =  realParts(index);
optim.wing.eigenModes.dampingTerm  =  imaginaryParts(index);
optim.wing.eigenModes.minDampingRatio = min(dampingRatios);

end

