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
  
function [ cnb, cn, cyb, cy ] = ParseDeltaMaxCn( filename )

% Load file
fileID = fopen(filename,'r');

% Find line where vtail starts
line  = 0;
while 1
    tline = fgetl(fileID);
    line = line + 1;
    if ischar(tline)
        startRowCY = strfind(tline, 'Fy /qS =  CY =');
        startRowCYBeta = strfind(tline, 'y force    |    CYu =');
        startRowCN = strfind(tline, 'Fz /qS =  CL =');
        startRowCnBeta = strfind(tline, 'Yaw   z mom.|    Cnu =');
        
        if isfinite(startRowCY)
            tlineCY = tline;
        end
        
        if isfinite(startRowCYBeta)
            tlineCYBeta = tline;
        end
        
        if isfinite(startRowCN)
            tlineCN = tline;
        end
        
        if isfinite(startRowCnBeta)
            tlineCnBeta = tline;
            break
        end
        
    end
end

% Starts reading until we hit the dots
index = strfind(tlineCnBeta, 'Cnb =');
cnb = str2num(tline(index+5:end));

index = strfind(tlineCN, 'Cn =');
cn = str2num(tlineCN(index+5:end));

index = strfind(tlineCYBeta, 'CYb =');
cyb = str2num(tlineCYBeta(index+5:end));

index = strfind(tlineCY, 'CY =');
cy = str2num(tlineCY(index+5:index+19));

fclose(fileID);

end
