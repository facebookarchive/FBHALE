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
  
function [lamNum, nPlies, tPly, fibAng, E11, E22, G12, nu12, density, matID, matName, primaryCoreMatID] = ...
    BuildLayup (panelType, UD_nPly, PW_nPly, coreType, core_nPly, MATS, includeFairing)
% This script builds the panel layup based on lamSchedule.mat data.	

data = load('lamSchedule.mat');
matID  = data.lamSchedule(:,1);
plyCnt = data.lamSchedule(:,2);
fibAng = data.lamSchedule(:,3);

switch coreType
    case 'soft'
        primaryCoreMatID = 5;
    case 'stiff'
        primaryCoreMatID = 6;
end


switch panelType
    
    case 'Top-Fairing'
        if includeFairing
        plyCnt(1:5) = 1;
        else
        plyCnt(end) = 1;   
        end
        
    case 'Bot-Fairing'
        if includeFairing
        plyCnt(end-1) = 1;
        else
        plyCnt(end) = 1;      
        end
        
    case 'Web'
        plyCnt(15) = 1 + PW_nPly;
        plyCnt(16) = core_nPly;
        plyCnt(17) = 1 + PW_nPly;
        matID(16)  = primaryCoreMatID;
        
    case 'Top-UD'
        if includeFairing
        plyCnt(1:5) = 1;
        end
        plyCnt(16) = core_nPly;
        matID(16)  = primaryCoreMatID;
        
        k = 0;
        while UD_nPly > 0
            if floor(UD_nPly/4)
                plyCnt(14-k) =  4;
                plyCnt(15-k) =  1;
                plyCnt(17+k) =  1;
                plyCnt(18+k) =  4;
                UD_nPly      = UD_nPly - 4;
            else
                plyCnt(14-k) =  mod(UD_nPly,4);
                plyCnt(15-k) =  1;
                plyCnt(17+k) =  1;
                plyCnt(18+k) =  mod(UD_nPly,4);
                UD_nPly      = UD_nPly -  mod(UD_nPly,4);
            end
            k = k+2;
        end
               plyCnt(15)    =  plyCnt(15) + PW_nPly;
               plyCnt(17)    =  plyCnt(17) + PW_nPly;
        
    case 'Bot-UD'
        if includeFairing
        plyCnt(end-1) = 1;
        end
        plyCnt(16) = core_nPly;
        matID(16)  = primaryCoreMatID;
        
        k = 0;
        while UD_nPly > 0
            if floor(UD_nPly/4)
                plyCnt(14-k) =  4;
                plyCnt(15-k) =  1;
                plyCnt(17+k) =  1;
                plyCnt(18+k) =  4;
                UD_nPly      = UD_nPly - 4;
            else
                plyCnt(14-k) =  mod(UD_nPly,4);
                plyCnt(15-k) =  1;
                plyCnt(17+k) =  1;
                plyCnt(18+k) =  mod(UD_nPly,4);
                UD_nPly      = UD_nPly -  mod(UD_nPly,4);
            end
            k = k+2;
        end        
               plyCnt(15)    =  plyCnt(15) + PW_nPly;
               plyCnt(17)    =  plyCnt(17) + PW_nPly;
        
end

matID   = matID(plyCnt>0);
nPlies  = plyCnt(plyCnt>0);
fibAng  = fibAng(plyCnt>0);
matName = MATS.matName(matID);
lamNum  = (1:numel(matID))';
E11     = MATS.E11(matID);
E22     = MATS.E22(matID);
G12     = MATS.G12(matID);
nu12    = MATS.nu12(matID);
density = MATS.density(matID);
tPly    = MATS.minThickness(matID);

end





