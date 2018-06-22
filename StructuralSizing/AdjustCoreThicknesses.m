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
  
function [sparCap_core_nPly_fore sparCap_core_nPly_mid ...
sparCap_core_nPly_aft sparCap_core_nPly_bot] = AdjustCoreThicknesses (sparCap_core_nPly, entity)
% Function to manually tweak core thicknesses to strengthen the structure locally
% against buckling.

if strcmp(entity,'wing')
    sparCap_core_nPly_fore     = sparCap_core_nPly;
    sparCap_core_nPly_fore(2)  = sparCap_core_nPly_fore(1);
    sparCap_core_nPly_fore(5)  = sparCap_core_nPly_fore(1);
    sparCap_core_nPly_fore(4)  = ceil(sparCap_core_nPly_fore(5)*0.5);
    sparCap_core_nPly_fore(6)  = ceil(sparCap_core_nPly_fore(5)*0.5);
    
    sparCap_core_nPly_aft      = ceil(sparCap_core_nPly*0.7);
    sparCap_core_nPly_aft(2)   = sparCap_core_nPly_aft(1)*1.25;
    sparCap_core_nPly_aft(3)   = sparCap_core_nPly_aft(1);
    sparCap_core_nPly_aft(5)   = sparCap_core_nPly_aft(1);
    sparCap_core_nPly_aft(4)   = ceil(sparCap_core_nPly_aft(1)*0.6);
    sparCap_core_nPly_aft(6)   = ceil(sparCap_core_nPly_aft(5)*0.6);
    
    sparCap_core_nPly_mid      = ceil(sparCap_core_nPly*1.7);
    sparCap_core_nPly_mid(2:3) = sparCap_core_nPly_mid(1);
    sparCap_core_nPly_mid(5)   = sparCap_core_nPly_mid(1);
    sparCap_core_nPly_mid(4)   = ceil(sparCap_core_nPly_mid(1)*0.6);
    sparCap_core_nPly_mid(6)   = ceil(sparCap_core_nPly_mid(5)*0.6);
    
    sparCap_core_nPly_bot      = ones(1,length(sparCap_core_nPly));
else
    sparCap_core_nPly_fore     = sparCap_core_nPly;
    sparCap_core_nPly_aft      = sparCap_core_nPly;
    sparCap_core_nPly_mid      = sparCap_core_nPly;
    sparCap_core_nPly_bot      = ones(1,length(sparCap_core_nPly));
end
