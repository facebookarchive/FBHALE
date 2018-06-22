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
  
function [WEB SECNODES LamData] = defineLaminateData(entity, xo, TopUDScale, BotUDScale, ThicknessScaleStations, numStiffeners,...
    OPT, BLADE, WEB, MATS, nSegsTopBot)

%% re-assign some structure variable names (for convenience)
INB_STN  = OPT.INB_STN;
OUB_STN  = OPT.OUB_STN;
NUM_SEC  = BLADE.NUM_SEC;
zSec     = BLADE.zSec;
chord    = BLADE.chord;
pitAxis  = BLADE.pitAxis;
nWebs    = WEB.nWebs;

%% assign the elements of the design vector into meaningful variable names
[xCapSt_inb, ...        
          xCapEnd_inb, ...   
          xCapSt_oub, ...
          xCapEnd_oub, ...
          inbChLoc, ...
          oubChLoc,...
          UD_nPly,...
          sparCap_core_nPly,...
          boxSkin_nPly,...
          web_core_nPly...
          ] = assignDesignVars(xo, OPT, BLADE, WEB);

%% Ensure spars (widths) are linear for a given non-linear chord distribution
xCapSt        = planformLine(zSec, chord, pitAxis, xCapSt_inb,  xCapSt_oub,  INB_STN, OUB_STN);
xCapEnd       = planformLine(zSec, chord, pitAxis, xCapEnd_inb, xCapEnd_oub, INB_STN, OUB_STN);
xsec_node_st  = xCapSt  ./ chord + pitAxis;
xsec_node_end = xCapEnd ./ chord + pitAxis;

%% Build laminate configuration

xNodeTop           = cell(NUM_SEC, 1);
xNodeBot           = cell(NUM_SEC, 1);
LamData(NUM_SEC,1) = struct('Top',[],'Bot',[],'Web',[]);

UD_nPly_base = ceil(linspace(1,1,NUM_SEC));

% tweak core thicknesses locally to alleviate buckling issues 
[sparCap_core_nPly_fore sparCap_core_nPly_mid ...
sparCap_core_nPly_aft sparCap_core_nPly_bot] = AdjustCoreThicknesses (sparCap_core_nPly, entity);

% include fairing stiffnesses only for wing 
if strcmp(entity, 'wing')
    includeFairing = true;
else
    includeFairing = false;
end

for i = 1:NUM_SEC
    % on the top and bottom, assign the number of segments and chordwise
    % locations of the segment boundaries
    xNodeTop{i} = 0;
    xNodeBot{i} = 0;
    numSections = numStiffeners*2-1;
    sectionWidth = (xsec_node_end(i) - xsec_node_st(i))/numSections;        
    for j = 1:numSections
    xNodeTop{i} = [xNodeTop{i};xsec_node_st(i)+ (j-1)*sectionWidth];
    xNodeBot{i} = [xNodeBot{i};xsec_node_st(i)+ (j-1)*sectionWidth];
    end
    xNodeTop{i} = [xNodeTop{i};xsec_node_end(i);1];
    xNodeBot{i} = [xNodeBot{i};xsec_node_end(i);1];
    
    % top surface laminate data
    LamData(i).Top.nLam    = zeros(nSegsTopBot(i), 1); % number of laminas in the laminate
    LamData(i).Top.lamNum  =  cell(nSegsTopBot(i), 1); % lamina number
    LamData(i).Top.nPlies  =  cell(nSegsTopBot(i), 1); % number of plies
    LamData(i).Top.tPly    =  cell(nSegsTopBot(i), 1); % thickness of each ply
    LamData(i).Top.fibAng  =  cell(nSegsTopBot(i), 1); % fiber angle
    LamData(i).Top.matID   =  cell(nSegsTopBot(i), 1); % material identification number
    LamData(i).Top.matName =  cell(nSegsTopBot(i), 1); % material name
    LamData(i).Top.t       = zeros(nSegsTopBot(i), 1); % total thickness of the laminate
    LamData(i).Top.E_eff   = zeros(nSegsTopBot(i), 1); % effective Young's modulus of the laminate
    LamData(i).Top.G_eff   = zeros(nSegsTopBot(i), 1); % effective shear modulus of the laminate
    LamData(i).Top.nu_eff  = zeros(nSegsTopBot(i), 1); % effective Poisson ratio of the laminate
    LamData(i).Top.massByV = zeros(nSegsTopBot(i), 1); % effective density of the laminate
    LamData(i).Top.A       =  cell(nSegsTopBot(i), 1); % extensional stiffness matrix of the laminate
    LamData(i).Top.B       =  cell(nSegsTopBot(i), 1); % coupling matrix of the laminate
    LamData(i).Top.D       =  cell(nSegsTopBot(i), 1); % bending stiffness matrix of the laminate
    LamData(i).Top.ABD     =  cell(nSegsTopBot(i), 1); % ABD matrix of the laminate
    LamData(i).Top.Qbar    =  cell(nSegsTopBot(i), 1); % transformed reduced stiffness matrix for each lamina in the laminate (a multi-dimensional array)
    LamData(i).Top.T       =  cell(nSegsTopBot(i), 1); % transformation matrix, which transforms strains/stresses in the x-y axes to the principal axes
    LamData(i).Top.z_i     =  cell(nSegsTopBot(i), 1); % ply interface locations (z-coordinate in plate coordinate system)
    
    for n = 1:nSegsTopBot(i)
        
        % assign stiffer core material only for stiffened panel sections. 
        if ~mod(n,2)
            coreType = 'stiff';
        else
            coreType = 'soft';
        end

         if    n == 1  || n ==  numSections+2   % leading and trailing edge
             
        [lamNum,nPlies,tPly,fibAng,E11,E22,G12,nu12,density,matID,matName,primaryCoreMatID] = ....
            BuildLayup ('Top-Fairing', 0, 0, 'soft', 0, MATS, includeFairing);
            
         elseif  ~mod(n,2) && any(ismember(i,ThicknessScaleStations))  % stiffened section
             
        [lamNum,nPlies,tPly,fibAng,E11,E22,G12,nu12,density,matID,matName,primaryCoreMatID] = ....
            BuildLayup ('Top-UD', fix(UD_nPly(i)*TopUDScale(n/2)), boxSkin_nPly(i), coreType, sparCap_core_nPly_fore(i),MATS, includeFairing);

         else  % unstiffened section
             
                    [lamNum,nPlies,tPly,fibAng,E11,E22,G12,nu12,density,matID,matName,primaryCoreMatID] = ....
            BuildLayup ('Top-UD', UD_nPly_base(i), boxSkin_nPly(i), coreType, sparCap_core_nPly_mid(i), MATS, includeFairing);
        end
        
        
        % determine laminate properties by classical lamination theory (CLT)
        CLT = cltAnalysis(lamNum,nPlies,tPly,fibAng,E11,E22,G12,nu12,density);
        LamData(i).Top.nLam(n)             = length(lamNum);
        LamData(i).Top.lamNum{n}           = lamNum;
        LamData(i).Top.nPlies{n}           = nPlies;
        LamData(i).Top.tPly{n}             = tPly;
        LamData(i).Top.fibAng{n}           = fibAng;
        LamData(i).Top.matID{n}            = matID;
        LamData(i).Top.matName{n}          = matName;
        LamData(i).Top.t(n)                = CLT.t;
        LamData(i).Top.E_eff(n)            = CLT.E_eff;
        LamData(i).Top.Ex(n)               = CLT.Ex;
        LamData(i).Top.Ey(n)               = CLT.Ey;
        LamData(i).Top.G_eff(n)            = CLT.G_eff;
        LamData(i).Top.nu_eff(n)           = CLT.nu_eff;
        LamData(i).Top.massByV(n)          = CLT.massByV;
        LamData(i).Top.A{n}                = CLT.A;
        LamData(i).Top.B{n}                = CLT.B;
        LamData(i).Top.D{n}                = CLT.D;
        LamData(i).Top.ABD{n}              = CLT.ABD;
        LamData(i).Top.Qbar{n}             = CLT.Qbar;
        LamData(i).Top.T{n}                = CLT.T;
        LamData(i).Top.z_i{n}              = CLT.z_i;
        LamData(i).Top.primaryCoreMatID(n) = primaryCoreMatID;
    end % for n = 1:nSegsTopBot(i)
    
    % bottom surface laminate data is same as top in optimization mode
    LamData(i).Bot = LamData(i).Top;
    
    for n = 1:nSegsTopBot(i)
        
        if strcmp(entity,'wing')
            coreType = 'soft';
        else
            if ~mod(n,2)
                coreType = 'stiff';
            else
                coreType = 'soft';
            end
        end
        
         if      n == 1  || n ==  numSections+2   % leading and trailing edge
            
        [lamNum,nPlies,tPly,fibAng,E11,E22,G12,nu12,density,matID,matName,primaryCoreMatID] = ...
            BuildLayup ('Bot-Fairing', 0, 0, 'soft', 0, MATS, includeFairing);
            
         elseif  ~mod(n,2) && any(ismember(i,ThicknessScaleStations))   % stiffened section      
         
        [lamNum,nPlies,tPly,fibAng,E11,E22,G12,nu12,density,matID,matName,primaryCoreMatID] = ...
            BuildLayup ('Bot-UD', fix(UD_nPly(i)*BotUDScale(n/2)), boxSkin_nPly(i), coreType, sparCap_core_nPly_bot(i), MATS, includeFairing);
        
         else   % unstiffened section
             
        [lamNum,nPlies,tPly,fibAng,E11,E22,G12,nu12,density,matID,matName,primaryCoreMatID] = ...
            BuildLayup ('Bot-UD', UD_nPly_base(i), boxSkin_nPly(i), coreType, sparCap_core_nPly_bot(i), MATS, includeFairing);
        end
                
        % determine laminate properties by classical lamination theory (CLT)
        CLT = cltAnalysis(lamNum,nPlies,tPly,fibAng,E11,E22,G12,nu12,density);
        LamData(i).Bot.nLam(n)             = length(lamNum);
        LamData(i).Bot.lamNum{n}           = lamNum;
        LamData(i).Bot.nPlies{n}           = nPlies;
        LamData(i).Bot.tPly{n}             = tPly;
        LamData(i).Bot.fibAng{n}           = fibAng;
        LamData(i).Bot.matID{n}            = matID;
        LamData(i).Bot.matName{n}          = matName;
        LamData(i).Bot.t(n)                = CLT.t;
        LamData(i).Bot.E_eff(n)            = CLT.E_eff;
        LamData(i).Bot.Ex(n)               = CLT.Ex;
        LamData(i).Bot.Ey(n)               = CLT.Ey;
        LamData(i).Bot.G_eff(n)            = CLT.G_eff;
        LamData(i).Bot.nu_eff(n)           = CLT.nu_eff;
        LamData(i).Bot.massByV(n)          = CLT.massByV;
        LamData(i).Bot.A{n}                = CLT.A;
        LamData(i).Bot.B{n}                = CLT.B;
        LamData(i).Bot.D{n}                = CLT.D;
        LamData(i).Bot.ABD{n}              = CLT.ABD;
        LamData(i).Bot.Qbar{n}             = CLT.Qbar;
        LamData(i).Bot.T{n}                = CLT.T;
        LamData(i).Bot.z_i{n}              = CLT.z_i;
        LamData(i).Bot.primaryCoreMatID(n) = primaryCoreMatID;
    end % for n = 1:nSegsTopBot(i)
    
    
    % web laminate data
    if nWebs(i) >= 1
        
        LamData(i).Web.nLam    = zeros(nWebs(i), 1);
        LamData(i).Web.lamNum  =  cell(nWebs(i), 1);
        LamData(i).Web.nPlies  =  cell(nWebs(i), 1);
        LamData(i).Web.tPly    =  cell(nWebs(i), 1);
        LamData(i).Web.fibAng  =  cell(nWebs(i), 1);
        LamData(i).Web.matID   =  cell(nWebs(i), 1);
        LamData(i).Web.matName =  cell(nWebs(i), 1);
        LamData(i).Web.t       = zeros(nWebs(i), 1);
        LamData(i).Web.E_eff   = zeros(nWebs(i), 1);
        LamData(i).Web.G_eff   = zeros(nWebs(i), 1);
        LamData(i).Web.nu_eff  = zeros(nWebs(i), 1);
        LamData(i).Web.massByV = zeros(nWebs(i), 1);
        LamData(i).Web.A       =  cell(nWebs(i), 1);
        LamData(i).Web.B       =  cell(nWebs(i), 1);
        LamData(i).Web.D       =  cell(nWebs(i), 1);
        LamData(i).Web.ABD     =  cell(nWebs(i), 1);
        LamData(i).Web.Qbar    =  cell(nWebs(i), 1);
        LamData(i).Web.T       =  cell(nWebs(i), 1);
        LamData(i).Web.z_i     =  cell(nWebs(i), 1);
       
        for n = 1:nWebs(i)
           
           [lamNum,nPlies,tPly,fibAng,E11,E22,G12,nu12,density,matID,matName,primaryCoreMatID] = ....
            BuildLayup ('Web', 0, boxSkin_nPly(i), 'soft', web_core_nPly(i), MATS, includeFairing);
                        
            % determine laminate properties by classical lamination theory (CLT)
            CLT = cltAnalysis(lamNum,nPlies,tPly,fibAng,E11,E22,G12,nu12,density);
            LamData(i).Web.nLam(n)             = length(lamNum);
            LamData(i).Web.lamNum{n}           = lamNum;
            LamData(i).Web.nPlies{n}           = nPlies;
            LamData(i).Web.tPly{n}             = tPly;
            LamData(i).Web.fibAng{n}           = fibAng;
            LamData(i).Web.matID{n}            = matID;
            LamData(i).Web.matName{n}          = matName;
            LamData(i).Web.t(n)                = CLT.t;
            LamData(i).Web.E_eff(n)            = CLT.E_eff;
            LamData(i).Web.Ex(n)               = CLT.Ex;
            LamData(i).Web.Ey(n)               = CLT.Ey;
            LamData(i).Web.G_eff(n)            = CLT.G_eff;
            LamData(i).Web.nu_eff(n)           = CLT.nu_eff;
            LamData(i).Web.massByV(n)          = CLT.massByV;
            LamData(i).Web.A{n}                = CLT.A;
            LamData(i).Web.B{n}                = CLT.B;
            LamData(i).Web.D{n}                = CLT.D;
            LamData(i).Web.ABD{n}              = CLT.ABD;
            LamData(i).Web.Qbar{n}             = CLT.Qbar;
            LamData(i).Web.T{n}                = CLT.T;
            LamData(i).Web.z_i{n}              = CLT.z_i;      
            LamData(i).Web.primaryCoreMatID(n) = primaryCoreMatID;
            
        end
        
    else
        % create placeholder values
        LamData(i).Web.nLam    = 0;
        LamData(i).Web.lamNum  = [];
        LamData(i).Web.nPlies  = [];
        LamData(i).Web.tPly    = [];
        LamData(i).Web.fibAng  = [];
        LamData(i).Web.matID   = [];
        LamData(i).Web.matName = [];
        LamData(i).Web.t       = [];
        LamData(i).Web.E_eff   = [];
        LamData(i).Web.G_eff   = [];
        LamData(i).Web.nu_eff  = [];
        LamData(i).Web.massByV = [];
        LamData(i).Web.A       = [];
        LamData(i).Web.B       = [];
        LamData(i).Web.D       = [];
        LamData(i).Web.ABD     = [];
        LamData(i).Web.Qbar    = [];
        LamData(i).Web.T       = [];
        LamData(i).Web.z_i     = [];
    end
    
end

%% Collect output
WEB.inbChLoc = inbChLoc;
WEB.oubChLoc = oubChLoc;

%% Determine the normalized chord locations of the webs and perform error checking
xWebNode = defineWebNodes(BLADE, WEB);
% check to see if the webs extend beyond the leading or trailing edge of the blade
if any(any(xWebNode < 0))
    error('ERROR: some of the shear webs extend beyond the leading edge. x/c < 0');
end
if any(any(xWebNode > 1))
    error('ERROR: some of the shear webs extend beyond the trailing edge. x/c > 1');
end

webLocs   = cell(NUM_SEC, 1);	% normalized chord locations of the shear web nodes
embNdsTop = cell(NUM_SEC, 1);	% normalized chord locations of the top panel endpoints, including the chord locations of the webs
embNdsBot = cell(NUM_SEC, 1);	% normalized chord locations of the bottom panel endpoints, including the chord locations of the webs
for i = 1:NUM_SEC
    % web nodes
    wl           = xWebNode(i,:);
    webLocs{i}   = wl(~isnan(wl))'; % an empty array if no webs exist at this station
    % top of the airfoil
    embNdsTop{i} = consolidator([xNodeTop{i}; webLocs{i}], [] ,[], 1e-4);  	% a sorted array in ascending order
    % bottom of the airfoil
    embNdsBot{i} = consolidator([xNodeBot{i}; webLocs{i}], [] ,[], 1e-4);	% a sorted array in ascending order
end

%% Collect output
SECNODES.nSegsTop  = nSegsTopBot;
SECNODES.nSegsBot  = nSegsTopBot;
SECNODES.xNodeTop  = xNodeTop;
SECNODES.xNodeBot  = xNodeBot;
SECNODES.webLocs   = webLocs;
SECNODES.embNdsTop = embNdsTop;
SECNODES.embNdsBot = embNdsBot;

end % function defineLaminateData


