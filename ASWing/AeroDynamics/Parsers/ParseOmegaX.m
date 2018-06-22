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
  
% This parser finds the value of OmegaX in a steady point output from
% ASWING. This is typically used in the control reversal study.
function OmegaX = ParseOmegaX(filename)

% Load the file
fileID = fopen(filename,'r');

% Find line where rates in body axis start
while 1
    tline = fgetl(fileID);
    if ischar(tline)
        startRow = strfind(tline, ' Flow angles,rates  in Body Axes');
        if isfinite(startRow)
            
            % Move next line over
            tline = fgetl(fileID);
            line = tline;
            break
        end
    end
end

% Find index in input string 'line' where the x-rate starts
startPatern  = 'Wx =';
endPatern    = 'deg/s';
indexStart   = strfind(line, startPatern);
indexEnd     = strfind(line, endPatern);
OmegaX       = str2double(line(indexStart+length(startPatern):indexEnd-1));

fclose(fileID);
