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

function MakeBinary (aircraftName)
% This script creates an executable version of the framework for the appropriate aircraft. 
% Ensure aswing pnt files and the prop airfoil file are located in the same directory 
% as the executable while running.
% RunMDO is the gateway function that is automatically modified based on the user-choice of the
% aircraft. 

% Add necessary folders to path:
addpath(genpath('Runs')); rmpath(genpath('Runs')); addpath('Runs');
aircraftInputfolder = ['Runs' filesep aircraftName];
airfoilADBsfolder = ['Runs' filesep 'AirfoilADBs'];
addpath(genpath(aircraftInputfolder));
addpath(genpath(airfoilADBsfolder));

% Set aircraft in run-time gateway:
G = regexp(fileread(['Binary' filesep 'RunMDO.m']), '\n', 'split');
G{43} = ['aircraftName = ''' aircraftName ''';'];
fid = fopen(['Binary' filesep 'RunMDO.m'], 'w');
fprintf(fid, '%s\n', G{:});
fclose(fid);

% Build executable:
 eval(['mcc -m RunMDO.m -a Runs/' aircraftName ' -a Solar -a StructuralSizing/Material_Data -a Performance/Solar -a Performance/Coverage -a StructuralSizing/Airfoil_Data -a Aerodynamics/Propulsion -d Binary -o PaleHale'])
