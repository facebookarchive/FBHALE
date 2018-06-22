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
function  F2 = ParseF2Def(filename)

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
F2 = [];
for i = 1:length(outputs)
    loc =  outputs{1};
    F2(end+1) = str2num(loc(strfind(loc, 'Flap 2 :')+8 :strfind(loc, 'Flap 3 :')- 1));
end


fclose(fileID);
