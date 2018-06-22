%{
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
  
function [ShearS tor_stff] = stressShear(Top, ...
                                         Bot, ...
                                         Web, ...
                                         Cell, ...
                                         nPanelsTop, ...
                                         nPanelsBot, ...
                                         nCells, ...
                                         x_cm, ...
                                         y_cm, ...
                                         x_tc, ...
                                         y_tc, ...
                                         axial_stff, ...
                                         EIx, ...
                                         EIy, ...
                                         EIxy, ...
                                         Vx, ...
                                         Vy, ...
                                         Mz, ...
                                         pz_w, ...
                                         pz_c)
     
%% Transverse shear flow
% constants needed for integration of the shear flow functions over the corresponding open section
dH = EIx*EIy - EIxy^2;
c1 = (pz_w + pz_c)/axial_stff;
c2 = ( Vy - (pz_w + pz_c)*(y_cm - y_tc))/dH;
c3 = (-Vx + (pz_w + pz_c)*(x_cm - x_tc))/dH;
d1 = -c2*EIy - c3*EIxy;
d2 = c2*EIxy + c3*EIx;

fo_t = cell(nCells, 1);      % shear flow function over the top surface of the corresponding open section
fo_b = cell(nCells, 1);      % shear flow function over the bottom surface of the corresponding open section
fo_w = cell(nCells+1, 1);    % shear flow function over the webs of the corresponding open section
    % create placeholder values for the ghost webs
    fo_w{1}        = 0; % the left ghost web
    fo_w{nCells+1} = 0; % the right ghost web
for n = 1:nCells
    fo_t{n} = 0 + c1.*Top.As{n} + d1.*Top.Qx{n} + d2.*Top.Qy{n};
end
for n = 2:nCells
    fo_w{n} = fo_t{n-1}(end) + c1.*Web.As{n} + d1.*Web.Qx{n} + d2.*Web.Qy{n};
end
    fo_b{1} = 0 + c1.*Bot.As{1} + d1.*Bot.Qx{1} + d2.*Bot.Qy{1};
for n = 2:nCells
    fo_b{n} = fo_b{n-1}(end) + fo_w{n}(end) + c1.*Bot.As{n} + d1.*Bot.Qx{n} + d2.*Bot.Qy{n};
end

% build the system of equations to solve for the closing shear constants
A = zeros(nCells, nCells);
b = zeros(nCells, 1);
for n = 1:nCells
    A(n,n) = trapzf(Top.s{n}, 1./(Top.G{n}.*Top.t{n})) + ...
             trapzf(Bot.s{n}, 1./(Bot.G{n}.*Bot.t{n})) + ...
             trapzf(Web.s{n}, 1./(Web.G{n}.*Web.t{n})) + ...
             trapzf(Web.s{n+1}, 1./(Web.G{n+1}.*Web.t{n+1}));
    b(n)   = trapzf(Bot.s{n}, fo_b{n}./(Bot.G{n}.*Bot.t{n})) - ...
             trapzf(Top.s{n}, fo_t{n}./(Top.G{n}.*Top.t{n})) + ...
             trapzf(Web.s{n}, fo_w{n}./(Web.G{n}.*Web.t{n})) - ...
             trapzf(Web.s{n+1}, fo_w{n+1}./(Web.G{n+1}.*Web.t{n+1}));
end
for n = 1:nCells - 1;
    A(n,n+1) = -1 * trapzf(Web.s{n+1}, 1./(Web.G{n+1}.*Web.t{n+1}));
end
for n = 2:nCells;
    A(n,n-1) = -1 * trapzf(Web.s{n}, 1./(Web.G{n}.*Web.t{n}));
end

% solve for the closing shear constants
fc = A \ b;

%% Torsional shear flows

% construct the system of equations
A  = zeros(nCells, nCells);
b  = zeros(nCells, 1); 
C  = zeros(nCells, nCells);
D  = zeros(nCells, nCells);
A1 = zeros(nCells, nCells);
A2 = zeros(nCells, nCells);
A3 = zeros(nCells, nCells);
for n = 1:nCells
    D(nCells,n) = Cell.area(n);
end
for n = 1:nCells - 1
    C(n,1)    = (1/Cell.area(1)) * (trapzf(Top.s{1}, 1./(Top.G{1}.*Top.t{1})) + ...
                                    trapzf(Bot.s{1}, 1./(Bot.G{1}.*Bot.t{1})) + ...
                                    trapzf(Web.s{1}, 1./(Web.G{1}.*Web.t{1})) + ...
                                    trapzf(Web.s{2}, 1./(Web.G{2}.*Web.t{2})));
    C(n,2)    = (-1/Cell.area(1)) * trapzf(Web.s{2}, 1./(Web.G{2}.*Web.t{2}));
    
    A1(n,n)   = (1/Cell.area(n+1)) * trapzf(Web.s{n+1}, 1./(Web.G{n+1}.*Web.t{n+1}));
    
    A2(n,n+1) = (-1/Cell.area(n+1)) * (trapzf(Top.s{n+1}, 1./(Top.G{n+1}.*Top.t{n+1})) + ...
                                       trapzf(Bot.s{n+1}, 1./(Bot.G{n+1}.*Bot.t{n+1})) + ...
                                       trapzf(Web.s{n+1}, 1./(Web.G{n+1}.*Web.t{n+1})) + ...
                                       trapzf(Web.s{n+2}, 1./(Web.G{n+2}.*Web.t{n+2})));
end
for n = 1:nCells - 2
    A3(n,n+2) = (1/Cell.area(n+1)) * trapzf(Web.s{n+2}, 1./(Web.G{n+2}.*Web.t{n+2}));
end
A         = A1 + A2 + A3 + C + D ; % coefficient matrix A
b(nCells) = Mz/2;

if nCells > 1
    % this system of equations for the torsional problem, as it was derived, is
    % badly scaled (ill-conditioned system).  This can be helped by scaling the
    % last equation in this system, and then the solution should be more accurate.  
    aa          = A(1:nCells-1,:);
    aa          = aa( aa ~= 0 );
    bb          = A(nCells,:);
    scaleFactor = mean(abs(aa)) / mean(bb);
    A(nCells,:) = A(nCells,:) .* scaleFactor;
    b(nCells)   = b(nCells) .* scaleFactor;
end

% solve for the torsional shear flow in each cell
q = A \ b;

% compute the torsional stiffness
if nCells < 2
    % twist rate of cell 1 (all cells have the same twist rate)
    k1 = 1/(2*Cell.area(1)) * (trapzf(Top.s{1},  q(1)./(Top.G{1}.*Top.t{1})) - ...
                               trapzf(Bot.s{1}, -q(1)./(Bot.G{1}.*Bot.t{1})));
    
else
    k1 = 1/(2*Cell.area(1)) * (trapzf(Top.s{1},          q(1)./(Top.G{1}.*Top.t{1})) + ...
                               trapzf(Web.s{2}, (q(1) - q(2))./(Web.G{2}.*Web.t{2})) - ...
                               trapzf(Bot.s{1},         -q(1)./(Bot.G{1}.*Bot.t{1})));  
end
tor_stff = Mz / k1;

%% Total shear flows (superposition of transverse & torsional shear flows)
f_top_temp = cell(nCells, 1);
f_bot_temp = cell(nCells, 1);
for n = 1:nCells
    % transverse shear flow
    f_top_temp{n} = fo_t{n} + fc(n) + q(n);
    f_bot_temp{n} = fo_b{n} - fc(n) - q(n);
end
f_top = cat(1, f_top_temp{:});
f_bot = cat(1, f_bot_temp{:});

f_web = cell(nCells-1, 1);
for n = 2:nCells
    % transverse shear flow
    f_web{n-1} = fo_w{n} + fc(n-1) - fc(n) + q(n-1) - q(n);
end

%% shear stresses
t_top         = cell2mat(Top.t);
t_bot         = cell2mat(Bot.t);
stress_zs_top = f_top ./ t_top;
stress_zs_bot = f_bot ./ t_bot;

stress_zs_web = cell(nCells-1, 1);
for n = 2:nCells
    stress_zs_web{n-1} = f_web{n-1} ./ Web.t{n};
end

% now map the indices to match the original panel indicies
shearFlow_t = cell(nPanelsTop, 1);
stress_zs_t = cell(nPanelsTop, 1);
for n = 1:nPanelsTop
    shearFlow_t{n} =         f_top(Top.iPan_St(n):Top.iPan_End(n));
    stress_zs_t{n} = stress_zs_top(Top.iPan_St(n):Top.iPan_End(n));
end

shearFlow_b = cell(nPanelsBot, 1);
stress_zs_b = cell(nPanelsBot, 1);
for n = 1:nPanelsBot
    shearFlow_b{n} =         f_bot(Bot.iPan_St(n):Bot.iPan_End(n));
    stress_zs_b{n} = stress_zs_bot(Bot.iPan_St(n):Bot.iPan_End(n));
end

nWebs = nCells-1;
if nWebs < 1
    % create dummy output
    shearFlow_w = [];
    stress_zs_w = [];
else
    shearFlow_w = cell(nWebs, 1);
    stress_zs_w = cell(nWebs, 1);
    for n = 1:nWebs
        shearFlow_w{n} = f_web{n};
        stress_zs_w{n} = stress_zs_web{n};
    end
end

%% Collect the output
ShearS.Top.shearFlow = shearFlow_t;
ShearS.Top.stress_zs = stress_zs_t;
ShearS.Bot.shearFlow = shearFlow_b;
ShearS.Bot.stress_zs = stress_zs_b;
ShearS.Web.shearFlow = shearFlow_w;
ShearS.Web.stress_zs = stress_zs_w;

end % function stressShear

