%{
Copyright (c) 1998, H.J. Sommer
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
  
function [Geom, Iner] = polygeom2(x, y) 
%
% This is a modified version of the original polygeom.m code published by
% H.J. Sommers: http://www.mathworks.com/matlabcentral/fileexchange/319-polygeom-m
%   
%   Changes by D.Sale include:
%   -added a check to see if inputs are empty and to return empty values
%   -changed the outputs to structure data class
%   -removed some error checking on the inputs
%   -removed computation of polar moment 
%   -removed computation of principal inertias and principal axes
%   -removed calls to shiftdim()
%   -removed calls to mean()

% check if inputs are empty
if isempty(x) || isempty(y)
    Geom.A     = [];
    Geom.x_c   = [];
    Geom.y_c   = [];
    Geom.P     = [];
    Iner.Ix    = [];
    Iner.Iy    = [];
    Iner.Ixy   = [];
    Iner.Iu    = [];
    Iner.Iv    = [];
    Iner.Iuv   = [];
    return;
end

% make sure that x and y are column vectors
x = x(:);
y = y(:);

% number of vertices
n = numel(x);

% temporarily shift data to mean of vertices for improved accuracy
xm  = sum(x) / n;
ym  = sum(y) / n;
ons = ones(n,1);
x   = x - xm*ons;
y   = y - ym*ons;

% delta x and delta y
dx = x( [ 2:n 1 ] ) - x;
dy = y( [ 2:n 1 ] ) - y;

% summations for CW boundary integrals
A   = sum( y.*dx - x.*dy )/2;
Axc = sum( 6*x.*y.*dx -3*x.*x.*dy +3*y.*dx.*dx +dx.*dx.*dy )/12;
Ayc = sum( 3*y.*y.*dx -6*x.*y.*dy -3*x.*dy.*dy -dx.*dy.*dy )/12;
Ixx = sum( 2*y.*y.*y.*dx -6*x.*y.*y.*dy -6*x.*y.*dy.*dy ...
          -2*x.*dy.*dy.*dy -2*y.*dx.*dy.*dy -dx.*dy.*dy.*dy )/12;
Iyy = sum( 6*x.*x.*y.*dx -2*x.*x.*x.*dy +6*x.*y.*dx.*dx ...
          +2*y.*dx.*dx.*dx +2*x.*dx.*dx.*dy +dx.*dx.*dx.*dy )/12;
Ixy = sum( 6*x.*y.*y.*dx -6*x.*x.*y.*dy +3*y.*y.*dx.*dx ...
          -3*x.*x.*dy.*dy +2*y.*dx.*dx.*dy -2*x.*dx.*dy.*dy )/24;
P   = sum( sqrt( dx.*dx +dy.*dy ) );

% check for CCW versus CW boundary
if A < 0,
    A   = -A;
    Axc = -Axc;
    Ayc = -Ayc;
    Ixx = -Ixx;
    Iyy = -Iyy;
    Ixy = -Ixy;
end

% centroidal moments
xc  = Axc / A;
yc  = Ayc / A;
Iuu = Ixx - A*yc*yc;
Ivv = Iyy - A*xc*xc;
Iuv = Ixy - A*xc*yc;

% replace mean of vertices
x_cen = xc + xm;
y_cen = yc + ym;
Ixx   = Iuu + A*y_cen*y_cen;
Iyy   = Ivv + A*x_cen*x_cen;
Ixy   = Iuv + A*x_cen*y_cen;

%% Collect output
Geom.A     = A;
Geom.x_c   = x_cen;
Geom.y_c   = y_cen;
Geom.P     = P;
Iner.Ix    = Ixx;
Iner.Iy    = Iyy;
Iner.Ixy   = Ixy;
Iner.Iu    = Iuu;
Iner.Iv    = Ivv;
Iner.Iuv   = Iuv;

end % function polygeom2

