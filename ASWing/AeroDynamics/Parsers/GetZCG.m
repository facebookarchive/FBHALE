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
  
function [ zcg ] = GetZCG( filename )

fileID = fopen(filename,'r');
LoadsData = [];

% Find line where vtail starts
line  = 0;
while 1
    tline = fgetl(fileID);
    line = line + 1;
    if ischar(tline)
        startRow = strfind(tline, '  Mass centroid at (x y z) = ');
        
        if isfinite(startRow)
            break
        end
    end
end

% Starts reading until we hit the dots
index = strfind(tline, '= (');
zcg = str2num(tline(index+28:index+38));

fclose(fileID);

end

