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
  
function [x_sc y_sc] = shearCenter(Panel, ...
    Top, ...
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
    EIxy)

% apply an arbitrary load (transverse shear ONLY) for this shear flow analysis
pz_w = 0;
pz_c = 0;
Mz   = 0;

% the first load case
Vx1 = 1;
Vy1 = 0;
ShearS1 = stressShear(Top, ...
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
    Vx1, ...
    Vy1, ...
    Mz, ...
    pz_w, ...
    pz_c);

% the second load case
Vx2 = 0;
Vy2 = 1;
ShearS2 = stressShear(Top, ...
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
    Vx2, ...
    Vy2, ...
    Mz, ...
    pz_w, ...
    pz_c);

mz1_t = zeros(nPanelsTop, 1);
mz2_t = zeros(nPanelsTop, 1);
for n = 1:nPanelsTop
    s_t      = Panel.Top.s{n};
    ro_t     = normDist(Panel.Top.xs{n}, Panel.Top.ys{n});
    f1_t     = ShearS1.Top.shearFlow{n};
    f2_t     = ShearS2.Top.shearFlow{n};
    mz1_t(n) = trapzf(s_t, ro_t.*f1_t);
    mz2_t(n) = trapzf(s_t, ro_t.*f2_t);
end

mz1_b = zeros(nPanelsBot, 1);
mz2_b = zeros(nPanelsBot, 1);
for n = 1:nPanelsBot
    s_b      = Panel.Bot.s{n};
    ro_b     = normDist(Panel.Bot.xs{n}, Panel.Bot.ys{n});
    f1_b     = ShearS1.Bot.shearFlow{n};
    f2_b     = ShearS2.Bot.shearFlow{n};
    mz1_b(n) = -trapzf(s_b, ro_b.*f1_b);
    mz2_b(n) = -trapzf(s_b, ro_b.*f2_b);
end

mz1_w = zeros(nCells-1, 1);
mz2_w = zeros(nCells-1, 1);
for n = 1:nCells-1
    s_w      = Panel.Web.s{n};
    ro_w     = normDist(Panel.Web.xs{n}, Panel.Web.ys{n});
    f1_w     = ShearS1.Web.shearFlow{n};
    f2_w     = ShearS2.Web.shearFlow{n};
    mz1_w(n) = trapzf(s_w, ro_w.*f1_w);
    mz2_w(n) = trapzf(s_w, ro_w.*f2_w);
end
if (~isempty(mz1_w) && ~isempty(mz2_w))
mz2_w(1) = -mz2_w(1);
mz1_w(1) = -mz1_w(1);
end
x_sc = -sum([mz2_t; mz2_b; mz2_w]) / Vy2;
y_sc = sum([mz1_t; mz1_b; mz1_w]) / Vx1;

end % function shearCenter

%% ============================================================================
function ro = normDist(xs, ys)

% slope and y-intersection of the tangent line at a point along the curve
m = finiteDiff(ys)./finiteDiff(xs);
b = ys - m.*xs;

% intersection of the tangent line, and a line which starts at the origin
% and is perpendicular to the tangent line
x_int = b ./ (-1./m - m);
y_int = m.*x_int + b;

% perpendicular distance from the origin to the tangent line at a point along the curve
ro = hypot(x_int, y_int);
if any(isnan(ro))
    ro = abs(xs); 
end

end % function normDist

