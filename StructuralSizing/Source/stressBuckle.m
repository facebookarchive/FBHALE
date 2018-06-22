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
  
function Buckle = stressBuckle(Panel, NormS, ShearS, flag, MATS)

nPanels = Panel.nPanels;
CalFac  = 1.10; % FEM Calibration factor
Buckle  = [];

if nPanels < 2
    return
end

switch flag
    case {'top','bottom'}
        
        for n =  2:nPanels-1
            
            b                  = Panel.s{n}(end);
            t                  = Panel.t(n);
            E_eff              = Panel.E_eff(n);
            nu_eff             = Panel.nu_eff(n);
            avg_r              = radiusCurvature(Panel.xs{n}, Panel.ys{n});
            tc                 = max(Panel.tPly{n}.*Panel.nPlies{n});
            D66                = Panel.ABD{n}(end,end);
            D11                = Panel.ABD{n}(end-2,end-2);
            D22                = Panel.ABD{n}(end-1,end-1);
            D12                = Panel.ABD{n}(end-2,end-1);
            Gc                 = MATS.G12(Panel.primaryCoreMatID(n));
                       
            % critical buckling load for axial compression
            
            % flat panel:                                            % Kollar
            
            if n ==3
            
                % simply supported BC:
                Nxf                 = pi^2/(b^2)*(2*sqrt(D11*D22)+2*(D12+2*D66)); 

                else

                % one edge fixed and one edge free
%                 K                  = (2*D66+D12)/sqrt(D11*D22);
%                 mu                 = D12/(2*D66+D12);
%                 if K <= 1
%                 Nxf                 = sqrt(D11*D22)/b^2*(15.1*K*sqrt(1-mu) + 7*(1-K));
%                 else
%                 Nxf                 = sqrt(D11*D22)/b^2*(15.1*sqrt(1-mu) + (K-1)*6*(1-mu));
%                 end
                
                % one edge simply supported, other end free
                Nxf = 12*D66/b^2;
            
            end
                       
            % curved panel: cylinder
            Nxc                = 0.3*(t/avg_r)*E_eff*t;               % Roark, experimental
            
            % total load:
            %Nx                = sqrt(Nxc^2+0.25*Nxf^2) + 0.5*Nxf;    % Redshaw                     
            Nx                 = (Nxf + Nxc);                         % Wenzek
            Nx_cr              = Nx/(1+Nx/(tc*Gc)); 
            
            Nx                 = -mean(NormS.stress_zz{n}*t);
            if Nx < 0 
                Nx = 0;
            end
            
           Buckle              = [Buckle; Nx/Nx_cr];
        end
                
    case 'web'
        
        
        for n = 1:nPanels

            b             = Panel.s{n}(end);
            t             = Panel.t(n);
            tc            = max(Panel.tPly{n}.*Panel.nPlies{n});
            D66           = Panel.ABD{n}(end,end); 
            D11           = Panel.ABD{n}(end-2,end-2);
            D22           = Panel.ABD{n}(end-1,end-1);
            D12           = Panel.ABD{n}(end-2,end-1);
            Gc            = MATS.G12(Panel.primaryCoreMatID(n));
            
            
            % critical buckling load for axial compression
            Nxw_cr        = pi^2/(b^2)*(2*sqrt(D11*D22)+2*(D12+2*D66));
            Nxw_cr        = Nxw_cr/(1+Nxw_cr/(tc*Gc));
            
            % critical buckling load for linearly-varying load            
            Nxb_cr        = (pi/b)^2*(13.4*sqrt(D11.*D12)+10.4*(D12+2*D66));
            Nxb_cr        = Nxb_cr/(1+Nxb_cr/(tc*Gc));
            
            % critical buckling load for shear
            K             = (2*D66 + D12)/(sqrt(D11*D22));          
            if K<1
            Nxy_cr        = (2/b)^2*(D11.*D22^3)^(0.25)*(8.125+5.045*K);
            else
            Nxy_cr        = (2/b)^2*(D22.*(D12+2*D66))^(0.5)*(11.7+1.46/K^2);
            end           
            Nxy_cr        = Nxy_cr/(1+Nxy_cr/(tc*Gc));
            
            
            Nxy           = mean(abs(ShearS.stress_zs{n})*t);
            Nxw           = -mean(NormS.stress_zz{n}*t);
            if Nxw < 0
                Nxw = 0;
            end
            Nxb           = max((NormS.stress_zz{n})*t - mean(NormS.stress_zz{n}*t));
            
            lambda        = roots([(Nxy/Nxy_cr)^2+(Nxb/Nxb_cr)^2 (Nxw/Nxw_cr)^2 -1]);
            
            if ~isempty(lambda)
            Buckle        = [Buckle; max([1./lambda(lambda>0)])];
            else
            Buckle        = [Buckle; 0];
            end
            
        end
                  
end
Buckle = Buckle*CalFac;
end % function stressBuckle
