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
  
function [Top Bot Web Cell] = preProcessShearFlowData(NUM_SEC, x_tc, y_tc, Panel, xw_top, yw_top, xw_bot, yw_bot, nCells)

% concatenate some of the panel data, these data structures are more convenient to work with for the shear flow calculations
T = catPanelData(NUM_SEC, [Panel.Top]); % structure array storing data relating to the top panel geometry and properties
B = catPanelData(NUM_SEC, [Panel.Bot]); % structure array storing data relating to the bottom panel geometry and properties

Top(NUM_SEC,1)  = struct('s',  [], ...
                         'xs', [], ...
                         'ys', [], ...
                         't',  [], ...
                         'E',  [], ...
                         'G',  [], ...
                         'As', [], ...
                         'Qx', [], ...
                         'Qy', [], ...
                         'iPan_St',  [], ...
                         'iPan_End', []);
Bot(NUM_SEC,1)  = struct('s',  [], ...
                         'xs', [], ...
                         'ys', [], ...
                         't',  [], ...
                         'E',  [], ...
                         'G',  [], ...
                         'As', [], ...
                         'Qx', [], ...
                         'Qy', [], ...
                         'iPan_St',  [], ...
                         'iPan_End', []); 
Web(NUM_SEC,1)  = struct('s',  [], ...
                         'xs', [], ...
                         'ys', [], ...
                         't',  [], ...
                         'E',  [], ...
                         'G',  [], ...
                         'As', [], ...
                         'Qx', [], ...
                         'Qy', []);   
Cell(NUM_SEC,1) = struct('area',  [], ...
                         'x_cen', [], ...
                         'y_cen', []);                     
for i = 1:NUM_SEC
       
    % find the panel indicies along the top & bottom which separate the cells
    indTop = zeros(nCells(i)-1, 1);
    indBot = zeros(nCells(i)-1, 1);
    for n = 1:nCells(i)-1           
        [unused indTop(n)] = min( hypot(T(i).x_ou - xw_top{i}(n), T(i).y_ou - yw_top{i}(n)) );
        [unused indBot(n)] = min( hypot(B(i).x_ou - xw_bot{i}(n), B(i).y_ou - yw_bot{i}(n)) );            
    end
    a       = findClosest(T(i).iPan_St,  indTop);
    b       = findClosest(T(i).iPan_End, indTop);
    iTopSt  = [1; T(i).iPan_St(a)];
    iTopEnd = [T(i).iPan_End(b); length(T(i).s)];
    a       = findClosest(B(i).iPan_St,  indBot);
    b       = findClosest(B(i).iPan_End, indBot);
    iBotSt  = [1; B(i).iPan_St(a)];
    iBotEnd = [B(i).iPan_End(b); length(B(i).s)];

    s_t  = cell(nCells(i), 1);
    xs_t = cell(nCells(i), 1);
    ys_t = cell(nCells(i), 1);
    t_t  = cell(nCells(i), 1);
    E_t  = cell(nCells(i), 1);
    G_t  = cell(nCells(i), 1);
    As_t = cell(nCells(i), 1);
    Qx_t = cell(nCells(i), 1);
    Qy_t = cell(nCells(i), 1);
    for n = 1:nCells(i)
        s_t{n}  =     T(i).s(iTopSt(n):iTopEnd(n)) - T(i).s(iTopSt(n));
        xs_t{n} =    T(i).xs(iTopSt(n):iTopEnd(n));
        ys_t{n} =    T(i).ys(iTopSt(n):iTopEnd(n));
        t_t{n}  =     T(i).t(iTopSt(n):iTopEnd(n));
        E_t{n}  = T(i).E_eff(iTopSt(n):iTopEnd(n));
        G_t{n}  = T(i).G_eff(iTopSt(n):iTopEnd(n));
        As_t{n} = cumtrapzf(s_t{n}, E_t{n}.*t_t{n});
        Qx_t{n} = cumtrapzf(s_t{n}, E_t{n}.*t_t{n}.*(ys_t{n} - y_tc(i)));
        Qy_t{n} = cumtrapzf(s_t{n}, E_t{n}.*t_t{n}.*(xs_t{n} - x_tc(i)));        
    end

    s_b  = cell(nCells(i), 1);
    xs_b = cell(nCells(i), 1);
    ys_b = cell(nCells(i), 1);
    t_b  = cell(nCells(i), 1);
    E_b  = cell(nCells(i), 1);
    G_b  = cell(nCells(i), 1);
    As_b = cell(nCells(i), 1);
    Qx_b = cell(nCells(i), 1);
    Qy_b = cell(nCells(i), 1);
    for n = 1:nCells(i)
        s_b{n}  =     B(i).s(iBotSt(n):iBotEnd(n)) - B(i).s(iBotSt(n));
        xs_b{n} =    B(i).xs(iBotSt(n):iBotEnd(n));
        ys_b{n} =    B(i).ys(iBotSt(n):iBotEnd(n));
        t_b{n}  =     B(i).t(iBotSt(n):iBotEnd(n));
        E_b{n}  = B(i).E_eff(iBotSt(n):iBotEnd(n));
        G_b{n}  = B(i).G_eff(iBotSt(n):iBotEnd(n));
        As_b{n} = cumtrapzf(s_b{n}, E_b{n}.*t_b{n});
        Qx_b{n} = cumtrapzf(s_b{n}, E_b{n}.*t_b{n}.*(ys_b{n} - y_tc(i)));
        Qy_b{n} = cumtrapzf(s_b{n}, E_b{n}.*t_b{n}.*(xs_b{n} - x_tc(i)));        
    end

    s_w  = cell(nCells(i)+1, 1);
    xs_w = cell(nCells(i)+1, 1);
    ys_w = cell(nCells(i)+1, 1);
    t_w  = cell(nCells(i)+1, 1);
    E_w  = cell(nCells(i)+1, 1);
    G_w  = cell(nCells(i)+1, 1);
    As_w = cell(nCells(i)+1, 1);
    Qx_w = cell(nCells(i)+1, 1);
    Qy_w = cell(nCells(i)+1, 1);
        % create placeholder values for the ghost webs
        s_w{1}    = [0; 0];                             % the left ghost web has zero length (i.e. it gets ignored in the calculations)
        xs_w{1}   = [xs_t{1}(1); xs_t{1}(1)];           % these other values should be non-zero (and length >= 2) to avoid crashing the code
        ys_w{1}   = [ys_t{1}(1); ys_t{1}(1)];
        t_w{1}    = [ t_t{1}(1);  t_t{1}(1)];
        E_w{1}    = [ E_t{1}(1);  E_t{1}(1)];
        G_w{1}    = [ G_t{1}(1);  G_t{1}(1)];
        As_w{1}   = [As_t{1}(1); As_t{1}(1)];
        Qx_w{1}   = [Qx_t{1}(1); Qx_t{1}(1)];
        Qy_w{1}   = [Qy_t{1}(1); Qy_t{1}(1)];
        s_w{end}  = [0; 0];                             % the right ghost web has zero length (i.e. it gets ignored in the calculations)
        xs_w{end} = [xs_t{end}(end); xs_t{end}(end)];   % these other values should be non-zero (and length >= 2) to avoid crashing the code
        ys_w{end} = [ys_t{end}(end); ys_t{end}(end)];
        t_w{end}  = [ t_t{end}(end);  t_t{end}(end)];
        E_w{end}  = [ E_t{end}(end);  E_t{end}(end)];
        G_w{end}  = [ G_t{end}(end);  G_t{end}(end)];
        As_w{end} = [As_t{end}(end); As_t{end}(end)];
        Qx_w{end} = [Qx_t{end}(end); Qx_t{end}(end)];
        Qy_w{end} = [Qy_t{end}(end); Qy_t{end}(end)];
    for n = 2:nCells(i)
        s_w{n}  = Panel(i).Web.s{n-1};
        xs_w{n} = Panel(i).Web.xs{n-1};
        ys_w{n} = Panel(i).Web.ys{n-1};
        t_w{n}  = Panel(i).Web.t(n-1)     .* ones(size(s_w{n}));
        E_w{n}  = Panel(i).Web.E_eff(n-1) .* ones(size(s_w{n}));
        G_w{n}  = Panel(i).Web.G_eff(n-1) .* ones(size(s_w{n}));
        As_w{n} = cumtrapzf(s_w{n}, E_w{n}.*t_w{n});
        Qx_w{n} = cumtrapzf(s_w{n}, E_w{n}.*t_w{n}.*(ys_w{n} - y_tc(i)));
        Qy_w{n} = cumtrapzf(s_w{n}, E_w{n}.*t_w{n}.*(xs_w{n} - x_tc(i)));    
    end
    
    % compute the area enclosed by the cells, and cell centroids
    area_cell = zeros(nCells(i), 1);
    xc_cell   = zeros(nCells(i), 1);
    yc_cell   = zeros(nCells(i), 1);
    for n = 1:nCells(i)
        % area enclosed by the midwalls of the cell panels
        x_mw = [xs_t{n}; xs_w{n+1}; flipud(xs_b{n}); flipud(xs_w{n})];
        y_mw = [ys_t{n}; ys_w{n+1}; flipud(ys_b{n}); flipud(ys_w{n})];
        Geom = polygeom2(x_mw, y_mw);
        
        area_cell(n) = Geom.A;
        xc_cell(n)   = Geom.x_c;
        yc_cell(n)   = Geom.y_c; 
    end
    
    % Collect the output
    Top(i).s        = s_t;
    Top(i).xs       = xs_t;
    Top(i).ys       = ys_t;
    Top(i).t        = t_t;
    Top(i).E        = E_t;
    Top(i).G        = G_t;
    Top(i).As       = As_t;
    Top(i).Qx       = Qx_t;
    Top(i).Qy       = Qy_t;
    Top(i).iPan_St  = T(i).iPan_St;
    Top(i).iPan_End = T(i).iPan_End;
    
    Bot(i).s        = s_b;
    Bot(i).xs       = xs_b;
    Bot(i).ys       = ys_b;
    Bot(i).t        = t_b;
    Bot(i).E        = E_b;
    Bot(i).G        = G_b;
    Bot(i).As       = As_b;
    Bot(i).Qx       = Qx_b;
    Bot(i).Qy       = Qy_b;
    Bot(i).iPan_St  = B(i).iPan_St;
    Bot(i).iPan_End = B(i).iPan_End;
    
    Web(i).s  = s_w;
    Web(i).xs = xs_w;
    Web(i).ys = ys_w;
    Web(i).t  = t_w;
    Web(i).E  = E_w;
    Web(i).G  = G_w;
    Web(i).As = As_w;
    Web(i).Qx = Qx_w;
    Web(i).Qy = Qy_w;
    
    Cell(i).area  = area_cell;
    Cell(i).x_cen = xc_cell;
    Cell(i).y_cen = yc_cell;
end

end % function preProcessShearFlowData

%% ------------------------------------------------
function Out = catPanelData(NUM_SEC, Panel)    

Out(NUM_SEC,1) = struct('iPan_St',  [], ...
                        'iPan_End', [], ...
                        's',        [], ...
                        'x_ou',     [], ...
                        'y_ou',     [], ...
                        'xs',       [], ...
                        'ys',       [], ...
                        't',        [], ...
                        'E_eff',    [], ...
                        'G_eff',    []);
for i = 1:NUM_SEC
   
    x_ou = cell(Panel(i).nPanels,1);    % x-coordinates on the outside of the panel
    y_ou = cell(Panel(i).nPanels,1);    % y-coordinates on the outside of the panel
    s    = cell(Panel(i).nPanels,1);
    t    = cell(Panel(i).nPanels,1);
    E    = cell(Panel(i).nPanels,1);
    G    = cell(Panel(i).nPanels,1);
    for n = 1:Panel(i).nPanels
        
        nPts    = numel(Panel(i).x{n});
        x_ou{n} = Panel(i).x{n}(1:nPts/2);
        y_ou{n} = Panel(i).y{n}(1:nPts/2);
        
        s{n} = Panel(i).s{n};
        if n >= 2
           s{n} = s{n} + s{n-1}(end); 
        end
        ons  = ones(size(s{n}));
        t{n} = Panel(i).t(n)     .* ons;
        E{n} = Panel(i).E_eff(n) .* ons;
        G{n} = Panel(i).G_eff(n) .* ons;
    end
    
    len_s    = cellfun('length',s);
    iPan_End = cumsum(len_s);
    iPan_St  = [1; iPan_End(1:end-1)+1];

    % Collect the output
    Out(i).iPan_St  = iPan_St;
    Out(i).iPan_End = iPan_End;
    Out(i).s        = cat(1,s{:});
    Out(i).x_ou     = cat(1,x_ou{:});
    Out(i).y_ou     = cat(1,y_ou{:});
    Out(i).xs       = cat(1,Panel(i).xs{:});
    Out(i).ys       = cat(1,Panel(i).ys{:});
    Out(i).t        = cat(1,t{:});
    Out(i).E_eff    = cat(1,E{:});
    Out(i).G_eff    = cat(1,G{:});

end

end % function catPanelData
