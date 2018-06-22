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
  
function Out = definePanel(flag, x_nodes, y_nodes, i_sNodes, i_wNodes, zSec, LamData, iter)

switch flag
    case {'top','bottom'}
        
        i_swNodes = consolidator([i_sNodes; i_wNodes], [] ,[], 1e-4);
        nPanels   = numel(i_swNodes) - 1;
        panelSt   = i_swNodes(1:end-1);
        panelEnd  = i_swNodes(2:end);
        lam_ID    = zeros(nPanels, 1);
        for n = 1:nPanels
            for ii = 1:numel(i_sNodes) - 1
                if i_sNodes(ii) <= panelSt(n) && panelEnd(n) <= i_sNodes(ii+1)
                    lam_ID(n) = ii;
                    break
                end
            end
          
        end
    case 'web'
        
        nPanels = i_wNodes;
        
end

if nPanels < 1
    % create placeholder values, so the code does not crash later on
    Out.nPanels = 0;
    Out.x       = [];
    Out.y       = [];
    Out.z       = [];
    Out.xs      = [];
    Out.ys      = [];
    Out.zs      = [];
    Out.s       = []; 
    Out.x_cen   = [];
    Out.y_cen   = [];
    Out.t       = [];    
    Out.crsArea = [];
    Out.massByL = [];
    Out.massByV = [];
    Out.Ix      = [];     
    Out.Iy      = [];     
    Out.Ixy     = [];  
    Out.nLam    = [];
    Out.lamNum  = [];
    Out.nPlies  = [];
    Out.tPly    = [];
    Out.fibAng  = [];
    Out.matID   = [];
    Out.matName = [];
    Out.E_eff   = [];  
    Out.G_eff   = []; 
    Out.nu_eff  = []; 
    Out.A       = [];      
    Out.B       = [];      
    Out.D       = [];      
    Out.ABD     = [];    
    Out.Qbar    = [];  
    Out.T       = [];  
    Out.z_i     = [];      
    return
end

x       = cell(nPanels, 1);   % x-coordinate of the polygon forming the panel, w.r.t. reference x-y axes
y       = cell(nPanels, 1);   % y-coordinate of the polygon forming the panel, w.r.t. reference x-y axes
z       = cell(nPanels, 1);   % corresponding z-coordinate of the polygon x-y coordinates
xs      = cell(nPanels, 1);   % x-coordinate of the panel midwall, w.r.t. reference x-y axes
ys      = cell(nPanels, 1);   % y-coordinate of the panel midwall, w.r.t. reference x-y axes
zs      = cell(nPanels, 1);   % corresponding z-coordinate of the panel midwall
s       = cell(nPanels, 1);   % midwall curvilinear s-coordinate of the panel
x_cen   = zeros(nPanels, 1);  % geometric centroid of the panel, x-coordinate w.r.t. reference x-y axes
y_cen   = zeros(nPanels, 1);  % geometric centroid of the panel, y-coordinate w.r.t. reference x-y axes
t       = zeros(nPanels, 1);  % wall thickness of the panel
crsArea = zeros(nPanels, 1);  % cross sectional area of the panel
massByL = zeros(nPanels, 1);  % mass per unit length (span) of the panel
massByV = zeros(nPanels, 1);  % mass per volume (density) of the panel
Ix      = zeros(nPanels, 1);  % second area moment of inertia w.r.t. reference x axis
Iy      = zeros(nPanels, 1);  % second area moment of inertia w.r.t. reference y axis
Ixy     = zeros(nPanels, 1);  % second area cross moment of inertia w.r.t. reference x-y axes
nLam    = zeros(nPanels, 1);  % number of laminas in the n-th panel
lamNum  = cell(nPanels, 1);   % lamina number
nPlies  = cell(nPanels, 1);   % number of plies in the lamina
tPly    = cell(nPanels, 1);   % thickness of each ply
fibAng  = cell(nPanels, 1);   % fiber angle
matID   = cell(nPanels, 1);   % material identification number
matName = cell(nPanels, 1);   % material name
E_eff   = zeros(nPanels, 1);  % effective Young's modulus of the panel
G_eff   = zeros(nPanels, 1);  % effective shear modulus of the panel 
nu_eff  = zeros(nPanels, 1);  % effective Poisson ratio of the panel 
A       = cell(nPanels, 1);   % extensional stiffness matrix of the panel
B       = cell(nPanels, 1);   % coupling stiffness matrix of the panel
D       = cell(nPanels, 1);   % bending stiffness matrix of the panel
ABD     = cell(nPanels, 1);   % ABD matrix of the panel
Qbar    = cell(nPanels, 1);   % transformed reduced stiffness matrix for each lamina in the panel (a multi-dimensional array)
T       = cell(nPanels, 1);   % transformation matrix, which transforms strains/stresses in the x-y axes to the principal axes (a multi-dimensional array)
z_i     = cell(nPanels, 1);   % ply interface locations (z-coordinate in plate coordinate system)                   

for n = 1:nPanels
        
    switch flag
        case {'top','bottom'}
            if strcmp(flag,'top')
                direct = 1;
            else
                direct = -1;
            end 

            % material properties of the panel
            nLam(n)    = LamData.nLam(lam_ID(n));
            lamNum{n}  = LamData.lamNum{lam_ID(n)};
            nPlies{n}  = LamData.nPlies{lam_ID(n)};
            tPly{n}    = LamData.tPly{lam_ID(n)};
            fibAng{n}  = LamData.fibAng{lam_ID(n)};
            matID{n}   = LamData.matID{lam_ID(n)};
            matName{n} = LamData.matName{lam_ID(n)};
            t(n)       = LamData.t(lam_ID(n));
            E_eff(n)   = LamData.E_eff(lam_ID(n));
            Ex(n)      = LamData.Ex(lam_ID(n));
            Ey(n)      = LamData.Ey(lam_ID(n));
            G_eff(n)   = LamData.G_eff(lam_ID(n));
            nu_eff(n)  = LamData.nu_eff(lam_ID(n));
            massByV(n) = LamData.massByV(lam_ID(n));
            A{n}       = LamData.A{lam_ID(n)};
            B{n}       = LamData.B{lam_ID(n)};
            D{n}       = LamData.D{lam_ID(n)};
            ABD{n}     = LamData.ABD{lam_ID(n)};
            Qbar{n}    = LamData.Qbar{lam_ID(n)};
            T{n}       = LamData.T{lam_ID(n)};
            z_i{n}     = LamData.z_i{lam_ID(n)};

            % airfoil nodes of the panel
            xNodes = x_nodes(panelSt(n):panelEnd(n));
            yNodes = y_nodes(panelSt(n):panelEnd(n));

            % x-y coordinates around the perimeter of the panel, forming a non-closed polygon
            alpha = atan2( finiteDiff(yNodes), finiteDiff(xNodes) );
            beta  = alpha - pi/2*direct;
            x{n}  = [xNodes; flipud( xNodes + t(n).*cos(beta) )];
            y{n}  = [yNodes; flipud( yNodes + t(n).*sin(beta) )];

            % x-y coordinates of the midline of the panel
            xs{n} = xNodes + (t(n)/2).*cos(beta);
            ys{n} = yNodes + (t(n)/2).*sin(beta);

            % curvilinear s-coordinate of the panel midline
            N    = length(xs{n});
            s{n} = zeros(N,1);
            for i = 1:N-1
                r         = hypot(xs{n}(i+1) - xs{n}(i), ys{n}(i+1) - ys{n}(i));
                s{n}(i+1) = s{n}(i) + r;
            end
                
        case 'web'
            
            % material properties of the panel
            nLam(n)    = LamData.nLam(n);
            lamNum{n}  = LamData.lamNum{n};
            nPlies{n}  = LamData.nPlies{n};
            tPly{n}    = LamData.tPly{n};
            fibAng{n}  = LamData.fibAng{n};
            matID{n}   = LamData.matID{n};
            matName{n} = LamData.matName{n};
            t(n)       = LamData.t(n);
            E_eff(n)   = LamData.E_eff(n);
            Ex(n)      = LamData.Ex(n);
            Ey(n)      = LamData.Ey(n);            
            G_eff(n)   = LamData.G_eff(n);
            nu_eff(n)  = LamData.nu_eff(n);
            massByV(n) = LamData.massByV(n);
            A{n}       = LamData.A{n};
            B{n}       = LamData.B{n};
            D{n}       = LamData.D{n};
            ABD{n}     = LamData.ABD{n};
            Qbar{n}    = LamData.Qbar{n};
            T{n}       = LamData.T{n};
            z_i{n}     = LamData.z_i{n};
            
            % x-y coordinates around the perimeter of the web panel, forming a non-closed polygon
            alpha = atan2( finiteDiff(y_nodes{n}), finiteDiff(x_nodes{n}) );
            beta  = alpha + pi/2;
            x1    = x_nodes{n} + (t(n)/2).*cos(beta);
            y1    = y_nodes{n} + (t(n)/2).*sin(beta);
            beta  = alpha - pi/2;
            x2    = x_nodes{n} + (t(n)/2).*cos(beta);
            y2    = y_nodes{n} + (t(n)/2).*sin(beta);
            x{n}  = [x1; flipud(x2)];
            y{n}  = [y1; flipud(y2)];

            % x-y coordinates of the midline of the web panel
            xs{n} = x_nodes{n};
            ys{n} = y_nodes{n};

            % curvilinear s-coordinate of the web panel midline
            N    = length(xs{n});
            s{n} = zeros(N,1);
            for i = 1:N-1
                r         = hypot(xs{n}(i+1) - xs{n}(i), ys{n}(i+1) - ys{n}(i));
                s{n}(i+1) = s{n}(i) + r;
            end
            
    end % switch flag
    
    % z-coordinates of the panel nodes
    z{n}  = zSec .* ones(size(x{n}));
    zs{n} = zSec .* ones(size(s{n}));
        
    % geometric properties of the panel
    [Geom Iner] = polygeom2(x{n}, y{n});
    crsArea(n)  = Geom.A;
    x_cen(n)    = Geom.x_c;
    y_cen(n)    = Geom.y_c;
    massByL(n)  = crsArea(n)*massByV(n);
    Ix(n)       = Iner.Ix;
    Iy(n)       = Iner.Iy;
    Ixy(n)      = Iner.Ixy;
    
end % for n = 1:nPanels

%% Collect the output
Out.nPanels          = nPanels;
Out.x                = x;
Out.y                = y;
Out.z                = z;
Out.xs               = xs;
Out.ys               = ys;
Out.zs               = zs;
Out.s                = s; 
Out.x_cen            = x_cen;
Out.y_cen            = y_cen;
Out.t                = t;    
Out.crsArea          = crsArea;
Out.massByL          = massByL;
Out.massByV          = massByV;
Out.Ix               = Ix;     
Out.Iy               = Iy;     
Out.Ixy              = Ixy;  
Out.nLam             = nLam;
Out.lamNum           = lamNum;
Out.nPlies           = nPlies;
Out.tPly             = tPly;
Out.fibAng           = fibAng;
Out.matID            = matID;
Out.matName          = matName;
Out.E_eff            = E_eff;  
Out.Ex               = Ex;
Out.Ey               = Ey;
Out.G_eff            = G_eff; 
Out.nu_eff           = nu_eff; 
Out.A                = A;      
Out.B                = B;      
Out.D                = D;      
Out.ABD              = ABD;    
Out.Qbar             = Qbar;  
Out.T                = T;  
Out.z_i              = z_i; 
Out.primaryCoreMatID = LamData.primaryCoreMatID;

end % function definePanel



                
          
