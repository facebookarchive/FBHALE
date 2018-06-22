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
  
function [ F2_deg ] = RunASWINGElevonDef( optim)
%Runs wing and fuselage ASWing to get cnbeta Cmbeta ect. and checks for
%aileron reversal.

%% Write Aircraft
optim = WriteASWingFile(optim, [optim.ASWINGIODir 'GetElevlonDefAircraft' num2str(optim.designID) '.asw' ], 'AeroCoefficients', [1 1 1 1]);

%% Write ASWing input string 
WriteElevonDefInputString(optim)

%% Run ASWING
system(['aswing<' optim.ASWINGIODir 'ElevonDefInputString' num2str(optim.designID) '.txt>' optim.ASWINGIODir 'RawElevonDefData.txt ' ' 2>&1']);

F2_deg = ParseF2Def([optim.ASWINGIODir 'RawElevonDefData.txt']);



end

