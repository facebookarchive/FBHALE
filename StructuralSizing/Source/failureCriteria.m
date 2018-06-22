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
  
function [s11TFailureCriteria,  s11CFailureCriteria,  s22TFailureCriteria,  s22CFailureCriteria,  s12SFailureCriteria, maxBucklingStress] = ...
    failureCriteria(BLADE, WEB, LaminaSS, Buckle, FOS, RemoveStationsStressCalc)


%% re-assign some structure variable names (for convenience)
NUM_SEC      = BLADE.NUM_SEC;
nWebs        = WEB.nWebs;

%%
s11_fcT_top = cell(NUM_SEC, 1); 
s11_fcC_top = cell(NUM_SEC, 1); 
s22_fcT_top = cell(NUM_SEC, 1); 
s22_fcC_top = cell(NUM_SEC, 1); 
s12_fcS_top = cell(NUM_SEC, 1); 

s11_fcT_bot = cell(NUM_SEC, 1); 
s11_fcC_bot = cell(NUM_SEC, 1); 
s22_fcT_bot = cell(NUM_SEC, 1); 
s22_fcC_bot = cell(NUM_SEC, 1); 
s12_fcS_bot = cell(NUM_SEC, 1); 

s11_fcT_web = cell(NUM_SEC, 1); 
s11_fcC_web = cell(NUM_SEC, 1); 
s22_fcT_web = cell(NUM_SEC, 1); 
s22_fcC_web = cell(NUM_SEC, 1); 
s12_fcS_web = cell(NUM_SEC, 1); 

buckle_fc_top  = cell(NUM_SEC, 1); 
buckle_fc_bot  = cell(NUM_SEC, 1); 
buckle_fc_web  = cell(NUM_SEC, 1); 

for i = 1:floor(NUM_SEC -  RemoveStationsStressCalc)
        
    % ---- Top ----
    s11_fcT_top{i}   = cell2mat( cat(1, LaminaSS(i).Top.s_11_fc_T) );
    s11_fcC_top{i}   = cell2mat( cat(1, LaminaSS(i).Top.s_11_fc_C) );
    s22_fcT_top{i}   = cell2mat( cat(1, LaminaSS(i).Top.s_22_fc_T) );
    s22_fcC_top{i}   = cell2mat( cat(1, LaminaSS(i).Top.s_22_fc_C) );
    s12_fcS_top{i}   = cell2mat( cat(1, LaminaSS(i).Top.s_12_fc_S) );
    buckle_fc_top{i} = Buckle(i).Top;
   
    
    % ---- Bottom ----
    s11_fcT_bot{i}   = cell2mat( cat(1, LaminaSS(i).Bot.s_11_fc_T) );
    s11_fcC_bot{i}   = cell2mat( cat(1, LaminaSS(i).Bot.s_11_fc_C) );
    s22_fcT_bot{i}   = cell2mat( cat(1, LaminaSS(i).Bot.s_22_fc_T) );
    s22_fcC_bot{i}   = cell2mat( cat(1, LaminaSS(i).Bot.s_22_fc_C) );
    s12_fcS_bot{i}   = cell2mat( cat(1, LaminaSS(i).Bot.s_12_fc_S) );
    buckle_fc_bot{i} = Buckle(i).Bot;
    
    % ---- Web ----
    if nWebs(i) >= 1
        s11_fcT_web{i}   = cell2mat( cat(1, LaminaSS(i).Web.s_11_fc_T) );
        s11_fcC_web{i}   = cell2mat( cat(1, LaminaSS(i).Web.s_11_fc_C) );
        s22_fcT_web{i}   = cell2mat( cat(1, LaminaSS(i).Web.s_22_fc_T) );
        s22_fcC_web{i}   = cell2mat( cat(1, LaminaSS(i).Web.s_22_fc_C) );
        s12_fcS_web{i}   = cell2mat( cat(1, LaminaSS(i).Web.s_12_fc_S) );
        buckle_fc_web{i} = Buckle(i).Web;
    else
        s11_fcT_web{i}   = [];
        s11_fcC_web{i}   = [];
        s22_fcT_web{i}   = [];
        s22_fcC_web{i}   = [];
        s12_fcS_web{i}   = [];
        buckle_fc_web{i} = [];
    end
    
end
s11_fcT   = cell2mat( [s11_fcT_top; s11_fcT_bot; s11_fcT_web] );
s11_fcC   = cell2mat( [s11_fcC_top; s11_fcC_bot; s11_fcC_web] );
s22_fcT   = cell2mat( [s22_fcT_top; s22_fcT_bot; s22_fcT_web] );
s22_fcC   = cell2mat( [s22_fcC_top; s22_fcC_bot; s22_fcC_web] );
s12_fcS   = cell2mat( [s12_fcS_top; s12_fcS_bot; s12_fcS_web] );
buckle_fc = cell2mat( [buckle_fc_top; buckle_fc_bot; buckle_fc_web] );

maxBucklingStress   = max(buckle_fc)*FOS;
s11TFailureCriteria = max(s11_fcT)*FOS;
s11CFailureCriteria = max(s11_fcC)*FOS; 
s22TFailureCriteria = max(s22_fcT)*FOS;
s22CFailureCriteria = max(s22_fcC)*FOS; 
s12SFailureCriteria = max(s12_fcS)*FOS;  

end % function fitnessFunction
