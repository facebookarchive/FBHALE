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
  
function meanR = radiusCurvature(x, y)

% the function computes the mean radius of curvature for a given set of x-y coordinates
% credit for these algorithms should be given to Roger Stafford, who posted
% these algorithms on the MATLAB newsreader:
% http://www.mathworks.com/matlabcentral/newsreader/view_thread/152405  
% http://www.mathworks.com/matlabcentral/newsreader/view_thread/294297#796465

% make sure that x and y are column vectors
x = x(:);
y = y(:);

if numel(x) < 3
    meanR = realmax('double');
    
elseif numel(x) == 3
    x21   = x(2) - x(1); 
    y21   = y(2) - y(1);
    x31   = x(3) - x(1); 
    y31   = y(3) - y(1);
    h21   = x21^2 + y21^2; 
    h31   = x31^2 + y31^2;
    d     = 2*(x21*y31-x31*y21);
    meanR = sqrt(h21*h31*((x(3) - x(2))^2 + (y(3) - y(2))^2)) / abs(d);
    
else
    % take mean of coordinates
    mx = mean(x); 
    my = mean(y);
    % get differences from means
    X = x - mx; 
    Y = y - my; 
    % get variances
    dx2 = mean(X.^2); 
    dy2 = mean(Y.^2);
    % solve a least mean squares problem
    t = [X, Y] \ ((X.^2-dx2 + Y.^2-dy2)/2); 
    % t is the 2 x 1 solution array [a0; b0]
    a0 = t(1); 
    b0 = t(2);
    % calculate the radius
    meanR = sqrt(dx2 + dy2 + a0^2 + b0^2); 
    
end
    
end % function radiusCurvature

