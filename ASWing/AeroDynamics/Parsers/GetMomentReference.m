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
  
function [ xRef, zRef ] = GetMomentReference( filename )

fileID = fopen(filename,'r');

% Find line where vtail starts
line  = 0;
xRow = [];
while 1
    tline = fgetl(fileID);
    line = line + 1;
    if ischar(tline)
        startRow = strfind(tline, ' Moment reference...  x ');
        zRow     = strfind(tline, '                     z = ');
        
        if isfinite(startRow)
            xRow  = tline;
        end
        
        if any(isfinite(xRow)) && any(isfinite(zRow))
            zRow = tline;
            break
        end
            
    end
end

% Starts reading until we hit the dots
index   = strfind(xRow, '= ');
xRef    = str2num(xRow(index+4:index+11));

index   = strfind(zRow, '= ');
zRef    = str2num(tline(index+4:index+11));

fclose(fileID);

end

