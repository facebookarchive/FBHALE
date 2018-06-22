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
  
function [ RAWData ] = ParseStructuralQuantities( filename )

% file search options depending on OS 
if isdeployed
filenameSearch = filename;
elseif contains (computer, 'WIN')
filenameSearch = filename(max(strfind(filename, '\'))+1:end);
else
filenameSearch = filename(max(strfind(filename, '/'))+1:end);  
end

if ~exist(filenameSearch, 'file')
    RAWData =[];
else
    fileID = fopen(filename,'r');
    
    RAWData = [];
    
    while 1
        tline = fgetl(fileID);
        %if you have reached the end of the file break
        if tline == -1
            break
        end
        
        if strcmp(tline(1), '#')
            %skip to next line
        else
            RAWData = [RAWData; str2num(tline)];
        end
        
    end
end
end