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
  
function [ cnb, cyb, cxa, cza, cma, cx, cm, cz ] = ParseCnBeta( filename )

% Load file
fileID = fopen(filename,'r');

% Find line where vtail starts
line  = 0;
while 1
    tline = fgetl(fileID);
    line = line + 1;
    if ischar(tline)
        startRowCxa = strfind(tline, ' x force    |    CXu' );
        startRowCza = strfind(tline, 'z force    |    CLu');
        startRowCma = strfind(tline, 'Pitch y mom.|    Cmu');
        startRowCY = strfind(tline, '  y force    |    CYu =');
        startRowCn = strfind(tline, ' Yaw   z mom.|    Cnu = ');
        startRowCx = strfind(tline, 'Fx /qS =  CX =');
        startRowCz = strfind(tline, 'Fz /qS =  CL =');
        startRowCm = strfind(tline, 'My /qSc =  Cm =');
          
        if isfinite(startRowCY)
            tlineCY = tline;
        elseif  isfinite(startRowCxa)
            tlineCxa = tline;
        elseif  isfinite(startRowCza)
            tlineCza = tline;
        elseif  isfinite(startRowCma)
            tlineCma = tline;
        elseif  isfinite(startRowCx)
             tlineCx= tline;
        elseif isfinite(startRowCz)
            tlineCz= tline;
        elseif isfinite(startRowCm)
            tlineCm = tline;
        end
        if isfinite(startRowCn)
            tlineCn = tline;
            break
        end
    end
end

% Starts reading until we hit the dots
index = strfind(tlineCn, 'Cnb =');
cnb = str2num(tline(index+5:end));

index = strfind(tlineCY, 'CYb =');
cyb = str2num(tlineCY(index+5:end));

%Cxa
index = strfind(tlineCxa, 'CXa =');
cxa = str2num(tlineCxa(index+5:index+5+11));

%Cza
index = strfind(tlineCza, 'CLa =');
cza = str2num(tlineCza(index+5:index+5+11));

%Cma
index = strfind(tlineCma, 'Cma =');
cma = str2num(tlineCma(index+5:index+5+11));

% Cx
index = strfind(tlineCx, 'CX =');
cx = str2num(tlineCx(index+4:index+4+11));
% Cz
index = strfind(tlineCz, 'CL =');
cz = str2num(tlineCz(index+4:index+4+11));

%Cm
index = strfind(tlineCm, 'Cm =');
cm = str2num(tlineCm(index+4:end));

fclose(fileID);
end
