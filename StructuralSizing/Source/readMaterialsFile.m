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
  
function MATS = readMaterialsFile(SIM, BLADE, OPT)

fid = fopen(BLADE.MATS_FILE, 'r');
if fid == -1
    error(['ERROR: Could not locate and open file ' [SIM.materialDir filesep BLADE.MATS_FILE]]);
end
    
% skip the header lines
for j = 1:3
    fgetl(fid);
end

% read the table of materials data
matData = textscan(fid, '%f %f %f %f %f %f %f %f %f %f %f %f %s');

% close the materials file
fclose(fid);

%% check for errors on the materials data
if size(matData, 2) ~= 13
    error(['ERROR: Expected to read 13 columns in material data file, but the number of columns is ' num2str(size(matData, 2))]);
end

MATS.matID   = matData{:,1};  
MATS.E11     = matData{:,2};
MATS.E22     = matData{:,3};
MATS.G12     = matData{:,4};
MATS.nu12    = matData{:,5};
MATS.density = matData{:,6};
MATS.s11_fT  = matData{:,7};
MATS.s11_fC  = matData{:,8};
MATS.s22_yT  = matData{:,9};
MATS.s22_yC  = matData{:,10};
MATS.s12_y   = matData{:,11};
MATS.minThickness = matData{:,12};
MATS.matName = matData{:,13};
if OPT.OPTIMIZE && size(MATS.matID, 1) ~= 9
    error(['ERROR: When OPTIMIZE = true, the number of rows for material data is expected to be equal to 8, but instead the number of rows is ' num2str(size(matData{:,1}, 1))]);
end
if numel( unique(MATS.matID) ) ~= numel(MATS.matID) || any(~mod(MATS.matID,1) == 0)
    error('ERROR: The values for matID must be unique positive integers.')
end
if any(MATS.E11 <=0)
    error('ERROR: Values for E11 must be positive.');
end
if any(MATS.E22 <=0)
    error('ERROR: Values for E22 must be positive.');
end
if any(MATS.G12 <=0)
    error('ERROR: Values for G12 must be positive.');
end
if any(MATS.density < 0)
    error('ERROR: Values for density must be positive.');
end
if any(MATS.s11_fT < 0)
    error('ERROR: Values for s11_fT are expected to be positive.');
end
if any(MATS.s11_fC > 0)
    error('ERROR: Values for s11_fC are expected to be negative.');
end
if any(MATS.s22_yT < 0)
    error('ERROR: Values for s22_yT are expected to be positive.');
end
if any(MATS.s22_yC > 0)
    error('ERROR: Values for s22_yC are expected to be negative.');
end
if any(MATS.s12_y < 0)
    error('ERROR: Values for s12_y are expected to be positive.');
end
if any( MATS.nu12 > sqrt(MATS.E11./MATS.E22) )
    error('ERROR: Material properies are not physically possible. The relation nu12 < sqrt(E11/E22) must be satisfied.');
end

end % function readMaterialsFile

