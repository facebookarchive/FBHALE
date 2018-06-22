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
  
function [xCapSt_inb, ...        
          xCapEnd_inb, ...   
          xCapSt_oub, ...
          xCapEnd_oub, ...
          inbChLoc, ...
          oubChLoc,...
          UD_nPly_Dist,...
          sparCap_core_nPly_Dist,...
          boxSkin_nPly_Dist,...
          web_core_nPly_Dist...
          ] = assignDesignVars(xo, OPT, BLADE, WEB)
      
%% re-assign some structure variable names (for convenience)
INB_STN  = OPT.INB_STN;
OUB_STN  = OPT.OUB_STN;
pitAxis  = BLADE.pitAxis;
NUM_WEBS = WEB.NUM_WEBS;
NUM_SEC  = BLADE.NUM_SEC;

%% assign the elemtents of vector xo to meaningful variable names
w_cap_inb   = xo(1);
w_cap_oub   = xo(2);
UD_nPly = xo(3:4);
sparCap_core_nPly = xo(5:6);
boxSkin_nPly = xo(7:8);
web_core_nPly = xo(9:10);

% web chordwise locations
xCapSt_inb  = pitAxis(INB_STN) - w_cap_inb/2;
xCapEnd_inb = pitAxis(INB_STN) + w_cap_inb/2;
xCapSt_oub  = pitAxis(OUB_STN) - w_cap_oub/2;
xCapEnd_oub = pitAxis(OUB_STN) + w_cap_oub/2;
if NUM_WEBS > 1
    inbChLoc = linspace(xCapSt_inb, xCapEnd_inb, NUM_WEBS)';
    oubChLoc = linspace(xCapSt_oub, xCapEnd_oub, NUM_WEBS)';
else
    inbChLoc = (xCapSt_inb + xCapEnd_inb)/2;
    oubChLoc = (xCapSt_oub + xCapEnd_oub)/2;
end 

% Interpolate other variables:
UD_nPly_Dist            = fix(linspace (UD_nPly(1),UD_nPly(2),NUM_SEC));
sparCap_core_nPly_Dist  = fix(linspace (sparCap_core_nPly(1),sparCap_core_nPly(2),NUM_SEC));
boxSkin_nPly_Dist       = fix(linspace (boxSkin_nPly(1),boxSkin_nPly(2),NUM_SEC));
web_core_nPly_Dist      = fix(linspace (web_core_nPly(1),web_core_nPly(2),NUM_SEC));

end % function assignDesignVars

