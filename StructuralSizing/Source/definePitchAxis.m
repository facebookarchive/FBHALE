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
  
function pitAxis = definePitchAxis(BLADE, OPT)

%% re-assign some structure variable names (for convenience)
zSec        = BLADE.zSec;
chord       = BLADE.chord;
PITAXIS_VAL = OPT.PITAXIS_VAL;

%%
M           = length(zSec);
pitAxis     = zeros(M,1);
[unused jj] = max(chord);	% find location of max chord

% if the max chord is outboard of the blade root, assume that the blade
% is circular at the root
if jj < 3
    
    if jj == 1
        % the max chord is at the blade root
        pitAxis(:) = PITAXIS_VAL;
    else
        pitAxis(1)    = 0.5;
        pitAxis(2:M)  = PITAXIS_VAL;
    end
    
else
    % pose an optimization problem to minimize the blade curvature in the
    % transition between a circle and the maximum chord: 
    pitAxis(1)    = 0.5;
    pitAxis(jj:M) = PITAXIS_VAL;
    
    N          = jj - 2;        % the number of transition sections
    transChord = chord(2:jj-1); % the chord values within the transition region
    cSt        = chord(2);      % chord at the last circular section   
    cEnd       = chord(jj);     % the maximum chord
    paSt       = 0.5;           % pitch axis at the last circular section  
    paEnd      = PITAXIS_VAL;      % pitch axis at the max chord and remaining outboard sections  

    % create the A matrix for the linear inequality constraints
    eqnsLE = zeros(N-1,N);
    eqnsTE = zeros(N-1,N);
    for i = 1:N-1
        eqnsLE(i,i)   =  transChord(i); 
        eqnsLE(i,i+1) = -transChord(i+1); 
        eqnsTE(i,i)   = -transChord(i);
        eqnsTE(i,i+1) =  transChord(i+1);
    end
    A = [eqnsLE; eqnsTE];

    % create the b vector           
    bLE = zeros(N-1,1);
    bTE = zeros(N-1,1);
    for i = 1:N-1
        bTE(i) = transChord(i+1) - transChord(i);
    end
    b = [bLE; bTE];

    % set the options for fmincon
    Options = optimset('Algorithm', 'interior-point', ...  	% 'interior-point', 'active-set', 'sqp'
                       'Diagnostics', 'off', ...
                       'Display', 'off', ...                % 'iter-detailed'
                       'FinDiffType', 'central', ...
                       'UseParallel', 'always', ...
                       'MaxIter', 100, ...
                       'TolX', 1e-6, ...
                       'PlotFcns', []);       % {@optimplotx,@optimplotfunccount,@optimplotfval,@optimplotconstrviolation,@optimplotstepsize,@optimplotfirstorderopt}

    % get initial guess & bounds for the unknown pitch axis values
    [xo lb ub] = initPitAxis(cSt,cEnd,paSt,paEnd,transChord);
    
	% this function returns a measure of the blade curvature
    FUN = @(x) pitAxisCurv(x,zSec,chord,pitAxis,jj);
    
    % find the pitch axis values which minimizes the curvature 
    paTran = fmincon(FUN,xo,A,b,[],[],lb,ub,[],Options);
    pitAxis(2:jj-1) = paTran;
               
end

end % function definePitchAxis

%% ========================================================================
function [xo lb ub] = initPitAxis(cSt, cEnd, paSt, paEnd, transChord)
% This function determines the upper and lower bounds for the pitch axis
% and return an intitial guess for the unknown pitch axis values

b1 = paSt  * cSt  ./ transChord;
b2 = (transChord - (1 - paSt)*cSt)   ./ transChord;
b3 = (transChord - (1 - paEnd)*cEnd) ./ transChord;
b4 = paEnd * cEnd ./ transChord;

lb = b1;
ub = b2;
for i = 2:length(transChord)
    if b3(i) > b1(i)
        lb(i) = b3(i);
    end
    if b4(i) < b2(i)
        ub(i) = b4(i);
    end
end

lb = min(lb, ub);
ub = max(lb, ub);

xo = (lb + ub) ./ 2;

end % function initPitAxis

%% ========================================================================
function paCurv = pitAxisCurv(paTrans, zSec, chord, pitAxis, jj)
% This function returns a measure of the blade curvature in the transition
% between the circular root and max chord

pitAxis(2:jj-1) = paTrans;

LE       = -pitAxis .* chord;                           % planform LE
TE       = (1 - pitAxis) .* chord;                      % planform TE
LE_slope = finiteDiff(LE) ./ finiteDiff(zSec);          % slope of LE
TE_slope = finiteDiff(TE) ./ finiteDiff(zSec);          % slope of TE
LE_curv  = finiteDiff(LE_slope) ./ finiteDiff(zSec);    % curvature of LE
TE_curv  = finiteDiff(TE_slope) ./ finiteDiff(zSec);    % curvature of TE

if numel(paTrans) >= 3
    % define the measure of blade curvature as the mean radius of curvature of
    LE_meanRad = radiusCurvature(zSec(2:jj-1), LE(2:jj-1));
    TE_meanRad = radiusCurvature(zSec(2:jj-1), TE(2:jj-1));
    paCurv     = -1 * (LE_meanRad + TE_meanRad);
else
    % define the measure of blade curvature as the area under the absolute 
    % value of the curvatures
    paCurv = trapzf(zSec, abs(LE_curv) + abs(TE_curv));
end

% % 
% figure(99)
% clf
% set(gcf,'color','white')
% subplot(3,1,1)
% hold on
% plot(zSec, LE, '.-r')                   
% plot(zSec, TE, '.-b')                
% xlabel('z (m)')
% ylabel('x (m)')
% axis equal
% 
% subplot(3,1,2)
% hold on
% plot(zSec, LE_slope, '.-r')
% plot(zSec, TE_slope, '.-b')
% xlabel('z (m)')
% ylabel('slope')
% 
% subplot(3,1,3)
% hold on
% plot(zSec, LE_curv, '.-r')
% plot(zSec, TE_curv, '.-b')
% xlabel('z (m)')
% ylabel('curvature')

end % function pitAxisCurv
