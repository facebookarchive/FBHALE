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
  
function [x_web y_web] = webCoords(Panel, xw_top, yw_top, xw_bot, yw_bot, nWebs, WEB_NODES)

nPanelsTop = Panel.Top.nPanels;
nPanelsBot = Panel.Bot.nPanels;

% compute the x-y coordinates of the inside edge of the top panels
x_t_ou = cell(nPanelsTop, 1);
y_t_ou = cell(nPanelsTop, 1);
x_t_in = cell(nPanelsTop, 1);
y_t_in = cell(nPanelsTop, 1);
for k = 1:nPanelsTop
    nPts   = numel(Panel.Top.x{k});
    x_t_ou{k} = Panel.Top.x{k}(1:nPts/2);
    y_t_ou{k} = Panel.Top.y{k}(1:nPts/2);
    x_t_in{k} = Panel.Top.x{k}(end:-1:nPts/2+1);
    y_t_in{k} = Panel.Top.y{k}(end:-1:nPts/2+1);    
end
x_top_ou = cat(1, x_t_ou{:});
y_top_ou = cat(1, y_t_ou{:});
x_top_in = cat(1, x_t_in{:});
y_top_in = cat(1, y_t_in{:});

% compute the x-y coordinates of the inside edge of the bottom panels
x_b_ou = cell(nPanelsBot, 1);
y_b_ou = cell(nPanelsBot, 1);
x_b_in = cell(nPanelsBot, 1);
y_b_in = cell(nPanelsBot, 1);
for k = 1:nPanelsBot
    nPts   = numel(Panel.Bot.x{k});
    x_b_ou{k} = Panel.Bot.x{k}(1:nPts/2);
    y_b_ou{k} = Panel.Bot.y{k}(1:nPts/2);
    x_b_in{k} = Panel.Bot.x{k}(end:-1:nPts/2+1);
    y_b_in{k} = Panel.Bot.y{k}(end:-1:nPts/2+1);    
end
x_bot_ou = cat(1, x_b_ou{:});
y_bot_ou = cat(1, y_b_ou{:});
x_bot_in = cat(1, x_b_in{:});
y_bot_in = cat(1, y_b_in{:});

% compute the x-y coordinates of the shear web midline
x_web = cell(nWebs, 1);
y_web = cell(nWebs, 1);
for i = 1:nWebs
            
    r_top = hypot(xw_top(i) - x_top_ou, yw_top(i) - y_top_ou);
    r_bot = hypot(xw_bot(i) - x_bot_ou, yw_bot(i) - y_bot_ou);
    [unused i_top] = min(r_top);
    [unused i_bot] = min(r_bot);
    
    x_web{i} = linspace(x_top_in(i_top), x_bot_in(i_bot), WEB_NODES)';
    y_web{i} = linspace(y_top_in(i_top), y_bot_in(i_bot), WEB_NODES)';
    
end


end % function webCoords
