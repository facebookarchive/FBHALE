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
  
function [ optim ] = RunASWINGDivergence( optim, useStructure)

%% Write Aircraft
optim = WriteASWingFile(optim, [optim.ASWINGIODir 'DivergenceAircraft' num2str(optim.designID) '.asw'] , 'dynamicModes', useStructure);

WriteDivergenceInput(optim)
 

%% Run ASWING
system(['aswing<' optim.ASWINGIODir 'DivergenceInputString' num2str(optim.designID) '.txt>' GetNullDevice() ' 2>&1']);

%% Parse Outputs

% read eigenvalues

fid = fopen(sprintf('%sDivergenceEVs%i.e00', optim.ASWINGIODir, optim.designID),'r');
lns = textscan(fid,'%f %f %f','HeaderLines',3);
eigs = lns{2};
freqs = lns{3}/(2*pi);
eigs_div = eigs(freqs<1e-3); % choose zero order mode(s)

% check stability
% unstable: divergenceLevel>0
% stable: divergenceLevel<0

if  isempty(eigs_div)
    warning('Zero order/divergence mode not found! Defaulting to stable..');
    optim.wing.divergenceLevel = -eps;
else
    optim.wing.divergenceLevel = max(eigs_div);
end 


end



