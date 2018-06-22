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
  
function xWebNode = defineWebNodes(BLADE, WEB)

%% re-assign some structure variable names (for convenience)
NUM_SEC  = BLADE.NUM_SEC;
zSec     = BLADE.zSec;
chord    = BLADE.chord;
pitAxis  = BLADE.pitAxis;
NUM_WEBS = WEB.NUM_WEBS;
inbStn   = WEB.inbStn;
oubStn   = WEB.oubStn;
inbChLoc = WEB.inbChLoc;
oubChLoc = WEB.oubChLoc;

%%
xWebNode = NaN(NUM_SEC, NUM_WEBS);  % x/c location of the webs, use NaN as a placeholder where webs do not exist
for n = 1:NUM_WEBS
    x  = zSec(inbStn(n):oubStn(n));
    pa = pitAxis(inbStn(n):oubStn(n));
    c  = chord(inbStn(n):oubStn(n));
    y1 = (inbChLoc(n) - pa(1))*c(1);
    y2 = (oubChLoc(n) - pa(end))*c(end);
    x1 = x(1);
    x2 = x(end);
    m  = (y2 - y1)/(x2 - x1);
    y  = m.*(x - x1) + y1;
    
    xWebNode(inbStn(n):oubStn(n),n) = y./c + pa;
end

for i = 1:NUM_SEC
    if any( diff(xWebNode(i,:)) <= 0 )
        error(['ERROR: The webs are not allowed to cross, but they appear to cross at station ' num2str(i)]);
    end
end

end % function defineWebNodes

