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
function [ F1, F2 ]= ParseControlSurfaceDef(filename)

% Load the file
fileID  = fopen(filename,'r');
outputs = {};
% Find line where rates in body axis start
while 1
     tline = fgetl(fileID);
     
    %if you have reached the end of the file break
    if tline == -1
        break
    end
    
    if ischar(tline)
        startRow = strfind(tline, '  Flap 1 ');
        if any(isfinite(startRow)) && contains(tline, 'Flap 1 :')
            
            % Move next line over
            outputs{end+1} = tline;
            tline = fgetl(fileID);

        end
    end
end

% Find index in input string 'line' where the x-rate starts
stall = outputs{2};
dive  = outputs{4};

F1Stall = str2num(stall(11:17));
F1Dive  = str2num(dive(11:17));
F1 = [F1Stall F1Dive];

F2Stall = str2num(stall(40:47));
F2Dive  = str2num(dive(40:47));
F2 = [F2Stall F2Dive];

fclose(fileID);
