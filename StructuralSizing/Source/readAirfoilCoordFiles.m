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
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

Modified from the original coblade repository. License for the original 
file appears below:

Copyright (c) 2012, Danny Sale 
Copyright (c) 2016, H.J. Sommer 
Copyright (c) 2009, John D'Errico 
Copyright (c) 2017, Yair Altman 
Copyright (c) 2016, S. Samuel Chen 
All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are 
met:

* Redistributions of source code must retain the above copyright 
notice, this list of conditions and the following disclaimer. 
* Redistributions in binary form must reproduce the above copyright 
notice, this list of conditions and the following disclaimer in 
the documentation and/or other materials provided with the distribution 
* Neither the name of the Penn State University nor the names 
of its contributors may be used to endorse or promote products derived 
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
POSSIBILITY OF SUCH DAMAGE.
%}
  
function NormAFcoords = readAirfoilCoordFiles(SIM, BLADE)

NormAFcoords(BLADE.NUM_SEC,1).x = [];    % preallocate structure array
NormAFcoords(BLADE.NUM_SEC,1).y = [];    % preallocate structure array
for i = 1:BLADE.NUM_SEC
    
    % open the airfoil file
    nul = which(BLADE.afFile{i});  %Sometimes fopen fails, using "which" helps to alleviate this issue.
    fid = fopen(BLADE.afFile{i}, 'r');
    if fid == -1
        error(['ERROR: Could not locate and open file ' [SIM.airfoilDir filesep BLADE.afFile{i}]]);
    end
    
    % skip the header lines
    for j = 1:4
        fgetl(fid);
    end
    
    % read the table of x-y coordinates
    xy = fscanf(fid, '%f %f', [2, Inf]);
    
    % close the airfoil file
    fclose(fid);
    
    % transpose, so the xy data matches the orientation of the file
    xy = xy';
    
    
    % check for errors in the user inputs
    if length(xy) < 4
        error(['Error: In file ' BLADE.afFile{i} ' the coordinates should contain at least 4 points.']);
    end
    if xy(1,1) ~= 0 || xy(1,2) ~= 0
        error(['Error: In file ' BLADE.afFile{i} ' the leading edge coordinate is not located at (x,y) = (0,0).']);
    end
    [x_max ii] = max(xy(:,1));
    if x_max > 1
        error(['Error: In file ' BLADE.afFile{i} ' the maximum x-coordinate exceeds the chord boundary (it should be equal to 1).']);
    end
    x_upper = xy(1:ii,1);
    y_upper = xy(1:ii,2);
    x_lower = xy(ii:end,1);
    y_lower = xy(ii:end,2);
    if y_upper(2) < y_lower(end)
        % note: this is not a perfect test for clockwise ordering, but I think it
        % will be sufficient given the other requirements for coordinate ordering
        error(['Error: In file ' BLADE.afFile{i} ' the coordinates are not labeled in clock-wise order.']);
    end
    
    % bullet-proofing against repeated x-coordinate reads (for linux-OS writes) by simply adding eps to create unique points:
    x_upper(find(diff(x_upper)==0)+1) = x_upper(diff(x_upper)==0)+1e-6;
    x_lower(find(diff(x_lower)==0)+1) = x_lower(diff(x_lower)==0)-1e-6;
    
    xy(1:ii,1)   =  x_upper;
    xy(ii:end,1) =  x_lower;
    
    NormAFcoords(i).x = xy(:,1);
    NormAFcoords(i).y = xy(:,2);  
    
    if any( diff(x_upper) <= 0 )
        error(['Error: In file ' BLADE.afFile{i} ' the upper surface is not a single valued function.']);
    end
    if any( diff(x_lower) >= 0 )
        % note: we excluded the trailing edge in this check, to allow for finite thickness trailing edges
        error(['Error: In file ' BLADE.afFile{i} ' the lower surface is not a single valued function.']);
    end
    
end

end % function readAirfoilCoordFiles
