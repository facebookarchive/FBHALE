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
  
function Out = stressNormal(Panel, ...
                            x_tc, ...
                            y_tc, ...
                            axial_stff, ...
                            EIx, ...
                            EIy, ...
                            EIxy, ...
                            Vz, ...
                            Mx, ...
                            My)

nPanels = Panel.nPanels;                        
                        
if nPanels < 1                        
    Out.stress_zz      = [];
    return
end

stress_zz     = cell(nPanels, 1);  % array of normal stresses, calculated around the perimeter of the panel
for n = 1:nPanels
    x     = Panel.x{n};
    y     = Panel.y{n};
    E_eff = Panel.E_eff(n);
    
    stress_zz{n} = E_eff .* ( Vz./axial_stff ...
                           - (My.*EIx + Mx.*EIxy)./(EIx.*EIy - EIxy.^2) .* (x - x_tc) ...
                           + (Mx.*EIy + My.*EIxy)./(EIx.*EIy - EIxy.^2) .* (y - y_tc) );
                                      
end

% See Section 6.2: Structural Analysis by Bachau and Craig
%% Collect the output
Out.stress_zz = stress_zz;   

end % function stressNormal
