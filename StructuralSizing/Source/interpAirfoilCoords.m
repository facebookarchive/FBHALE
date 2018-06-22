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
  
function NormAFcoords = interpAirfoilCoords(OrigNormAFcoords, BLADE)

switch BLADE.INTERP_AF 
    case 'none'
        % we still need to interpolate the coordinates to measure the percent
        % thickness, but we will use the original airfoil coordinates in the end
        x_cUp = linspace(0, 1, 200 + 1)';
        
    case 'equal'
        x_cUp = linspace(0, 1, BLADE.N_AF/2 + 1)';
        
    case 'cosine'
        x_cUp = cosspace(0, 1, BLADE.N_AF/2 + 1, 'both');
end
x_cLo = flipud( x_cUp );

NormAFcoords(BLADE.NUM_SEC,1).x           = [];       
NormAFcoords(BLADE.NUM_SEC,1).y           = [];      
NormAFcoords(BLADE.NUM_SEC,1).maxPerThick = [];       
for i = 1:BLADE.NUM_SEC
        
    oldX           = OrigNormAFcoords(i).x;
    oldY           = OrigNormAFcoords(i).y;
    [unused i_TE]  = min(abs(oldX - 1)); % find the index of the trailing edge 
    oldXup         = oldX(1:i_TE);
    oldYup         = oldY(1:i_TE);
    oldXlo         = [oldX(i_TE:end); 0];
    oldYlo         = [oldY(i_TE:end); 0];

    newYup = interp1(oldXup, oldYup, x_cUp);
    newYlo = interp1(oldXlo, oldYlo, x_cLo);
    
    NormAFcoords(i).x           = [x_cUp; x_cLo(2:end-1)];
    NormAFcoords(i).y           = [newYup; newYlo(2:end-1)];  
    NormAFcoords(i).maxPerThick = max(newYup - flipud(newYlo));
end

switch BLADE.INTERP_AF
    case 'none'
        % overwrite the interpolated coordinates with the original coordinates
        for i = 1:BLADE.NUM_SEC
            NormAFcoords(i).x = OrigNormAFcoords(i).x;
            NormAFcoords(i).y = OrigNormAFcoords(i).y;
        end
    otherwise
        return
end

end % function interpAirfoilCoords

