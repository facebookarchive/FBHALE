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
  
function [ optim ] = WriteASWingFile(optim, filename, operatingMode, useStructure, varargin)
% Write .asw file to file under the specified name, file format and data
% used is based on specified operating mode. Use of structural data
% determined by values n useStrucutre. 

aswingIn = fopen(filename,'wt');

%set aero database
if isempty(varargin)
    aeroADB = 'Performace';
elseif strcmp(varargin, 'LoadsADB')
    aeroADB = 'loadsAeroDataBase';
elseif strcmp(varargin, 'AeroCoefficients')
    aeroADB = 'AeroCoefficients';
else
    error('Unknow ADB type specified')
end

%% Tramsform structural data to global refrence frame
optim = BuildAircraftGeometry (optim);

%% Write Header
optim = WriteConfigurationHeader( optim, aswingIn );

%% Write Point Masses
optim = WritePointMasses( optim, aswingIn );

%% Write propulsion
optim = PlacePropulsoin( optim,aswingIn );

%% Add Sensors
optim = PlaceSensors( optim, operatingMode, aswingIn );
  
%% Write Beams (aero and non aero surfaces)
optim = WriteBeams( optim, aswingIn, useStructure, aeroADB );

%%
fclose(aswingIn);
end

