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
%}
  
function [wingPolar, airfoilPolars] = LiftingLineFourierDecomposition(y_m, c_m, twists_deg, AoA_deg, airfoilPolars, varargin)
% This lifting line method solves the for the Fourier coefficients in the
% downwash expansion to ensure consistency of the local lift coefficient.
% This local lift coefficient is provided by either actual 2D polars or a
% polynomial description.
% A Newton method is used to drive the solution to zero with analytical
% derivatives, ensuring well-behavedness and efficiency of the method.

% Check if an already existing wingPolar or flow field was passed. If so,
% use as initial point.
if nargin > 5 && ~isempty(varargin)
    wingPolar                       = varargin{1};
else
    wingPolar                       = [];
end

if isempty(wingPolar)
    wingPolar.AoA_deg               = [];
    wingPolar.yalphai_m             = {};
    wingPolar.alphai_rad            = {};
    wingPolar.gamma               	= {};
    wingPolar.An                    = {};
    wingPolar.n                     = [];
    wingPolar.CL                    = [];
    wingPolar.y_m                   = {};
    wingPolar.c_m                   = {};
    wingPolar.CDi                   = [];
    wingPolar.cl                    = {};
    wingPolar.cdi                   = {}; 
    wingPolar.isConverged           = [];
end

% If there's already a flow field captured in polars, then use it as
% initial guess
if ~isempty(wingPolar.An)
    An         = wingPolar.An{end};
    yalphai_m  = wingPolar.yalphai_m{end};
    CL0        = wingPolar.CL(end);
    CDi0       = wingPolar.CDi(end);
else
    An         = [1 0*y_m(1:end-1)]'; % Should figure out an elliptical guess
    yalphai_m  = y_m;
    CL0        = 1;
    CDi0       = 1;
end

% Local representation
approach = 'local'; % options: local or polynomial

% Initialize
error = 1;
n     = 5;

while error > 5e-3 && n < 30
    % Lifting line solve will rediscretize y_m and c_m based on n and
    % interpolate alphai_rad on that grid
    [yalphai_m, An, alphai_rad, wingPolar0] = LiftingLineSolve(n, y_m, c_m, AoA_deg, twists_deg, An, yalphai_m, airfoilPolars, approach);
    error = abs(wingPolar0.CL-CL0)./(abs(CL0)+ .01 * (abs(CL0)<1e-3)) + abs(wingPolar0.CDi-CDi0)./abs(CDi0);
    CL0   = wingPolar0.CL;
    CDi0  = wingPolar0.CDi;
    n     = floor(1.5*n);
end

% Storing
wingPolar.AoA_deg(end+1)   	= AoA_deg;
wingPolar.yalphai_m{end+1}	= yalphai_m;
wingPolar.cl{end+1}         = wingPolar0.cl;
wingPolar.cdi{end+1}        = wingPolar0.cdi;
wingPolar.alphai_rad{end+1}	= alphai_rad;
wingPolar.An{end+1}       	= An;
wingPolar.n(end+1)         	= n;
wingPolar.CL(end+1)      	= wingPolar0.CL;
wingPolar.CDi(end+1)      	= wingPolar0.CDi;
wingPolar.gamma{end+1}   	= wingPolar0.gamma;
wingPolar.y_m{end+1}        = wingPolar0.y_m;
wingPolar.c_m{end+1}        = wingPolar0.c_m;
wingPolar.isConverged(end+1)= wingPolar0.isConverged;

end


function [yalphai_m, An, alphai_rad, wingPolar] = LiftingLineSolve(n, y_m, c_m, AoA_deg, twists_deg, An, yalphai_m, airfoilPolars, approach)

% Check for problem symmetry. If it is symmetric, then use the same n number
% of span stations but for half
zeroYindex     	= find(y_m == 0);
SymmetricGeom	= [c_m; abs(y_m); twists_deg]';
if size(SymmetricGeom, 1) == 2 * zeroYindex - 1

    isSymmetricGeom	= max(max(abs(SymmetricGeom(1:zeroYindex, :) - SymmetricGeom(end:-1:zeroYindex, :)))) < 10*eps;
    if strcmp(approach, 'polynomial')
        isSymmetricAirfoil= max(max(abs(airfoilPolars.pCLMatrix(1:zeroYindex, :) - airfoilPolars.pCLMatrix(end:-1:zeroYindex, :)))) < 10*eps;
    else
        isSymmetricAirfoil= max(max(abs(airfoilPolars.cls(1:zeroYindex, :) - airfoilPolars.cls(end:-1:zeroYindex, :)))) < 10*eps;
    end
    isSymmetric = isSymmetricGeom & isSymmetricAirfoil;
else
    isSymmetric = false;
end

% Rediscretize based on cos discretization
if isSymmetric
    span  = 2*max(abs(y_m));
    theta = linspace(0, pi/2, n+1);
    theta = theta(2:end);
    n     = length(theta);
    y     = 1/2 * span * cos(theta);
    y(end)= 0;
    S  	  = trapz(y_m, c_m);
else
    span  = 2 * max(y_m);
    S     = trapz(y_m, c_m);
    theta = linspace(0, pi/2, n/2 + 2);
    theta = [theta(1:end-1) theta+pi/2];
    theta = theta(2:end-1);
    n     = length(theta);
    y     = 1/2 * span * cos(theta(end:-1:1));
end

AR = span*span/S;

% Interpolate provided distributions on new grid
c           = interp1q(y_m', c_m', y')';
twist_rad   = interp1q(y_m', twists_deg'*pi/180, y')';
An          = interp1(yalphai_m', An, y', 'linear', 0);

if strcmp(approach, 'polynomial')
    airfoilPolars.pCLMatrixReduced = interp1(y_m, airfoilPolars.pCLMatrix, y);
    airfoilPolars.pCLalphaReducedMatrix = interp1(y_m, airfoilPolars.pCLalphaMatrix, y);
    airfoilPolars.approach   = approach;
else
    airfoilPolars.clsReduced = interp1(airfoilPolars.ys_m, airfoilPolars.cls, y);
    airfoilPolars.approach   = approach;
end

% Solve for the system
if isSymmetric
    indices = 2*(1:n)-1;
else
    indices = 1:n;
end

% As we approach stall, the Newton method can sometimes struggle. Try with
% it, and move on to an optimization formulation if it fails.
X0 = An;

% Newton formulation
F  = @(X) LiftingLineSolver(airfoilPolars, AoA_deg + twist_rad*180/pi, X, theta, c, span, AR, indices, approach);
J  = @(X) ComputeJacobian(airfoilPolars, AoA_deg, X, theta, c, span, AR, indices, approach);
[X1, Y1] = Newton(F, X0, 'J', J, 'relax', .95, 'Nmax', 50);

[~, CL, CDi, gamma, alphai_rad, cl] = LiftingLineSolver(airfoilPolars, AoA_deg + twist_rad*180/pi, X1, theta, c, span, AR, indices, approach);
An = X1;

% if symmetric, then unfold
if isSymmetric
    y       = [-y y(end-1:-1:1)];
    theta   = Unfold(theta);
    c       = Unfold(c);
    twist_rad = Unfold(twist_rad);
    gamma     = Unfold(gamma);
    alphai_rad= Unfold(alphai_rad);
    cl        = Unfold(cl);
    An        = Unfold(An')';
end

yalphai_m           = y;
wingPolar.y_m       = y;
wingPolar.theta     = theta;
wingPolar.c_m       = c;
wingPolar.twist_rad = twist_rad;
wingPolar.alphai_rad = alphai_rad;
wingPolar.gamma     = gamma;
wingPolar.An        = An;
wingPolar.cl        = cl;
wingPolar.cdi       = -2/S*gamma.*sin(alphai_rad);
wingPolar.CL        = CL;
wingPolar.CDi       = CDi;

% Check for unconverged results
if abs(Y1) > 1e-4
    wingPolar.isConverged = false;
else
    wingPolar.isConverged = true;
end

end

function [cl_error, CL, CDi, gamma, alphai_rad, cl] = LiftingLineSolver(airfoilPolars, AoA_deg, An, theta, c, span, AR, indices, approach)

% Compute alphai based on An distribution
n         = length(An);
alphai_rad= asin(sum( (indices'*ones(1, n)) .* (An * ones(1, n)) .* sin( indices'*ones(1, n) .* (ones(n, 1) * theta))) ./ sin( theta ));

% Compute cl distribution based on the polar
[~, ~, cl0] = SectionalData(airfoilPolars, AoA_deg, alphai_rad, approach);

% Compute cl distribution based on the An values
gamma       = sum(2 * span * An * ones(1, n) .* sin( indices'*ones(1, n) .* (ones(n, 1) * theta)));
cl          = 2 .* gamma ./ c;
cl_error    = cl - cl0';

% Compute forces based on An
CL = An(1) * pi * AR;
CDi = real(pi * AR * sum( indices' .* An.^2));

end

function J = ComputeJacobian(airfoilPolars, AoA_deg, An, theta, c, span, AR, indices, approach)

% Compute alphai based on An distribution
n         = length(An);
alphai_rad= asin(sum( (indices'*ones(1, n)) .* (An * ones(1, n)) .* sin( indices'*ones(1, n) .* (ones(n, 1) * theta))) ./ sin( theta ));

% Compute Jacobian in matrix format
[clalpha, ~, ~] = SectionalData(airfoilPolars, AoA_deg, alphai_rad, approach);
J = sin( indices .* theta' ) .* (4*span./c' + clalpha'.* indices ./ sin( theta' ));

end

% AoA contains twist.
function [a, alpha0_rad, cl] = SectionalData(airfoilPolars, AoA_deg, alphai_rad, approach)

if strcmp(approach, 'polynomial')
    
    % Polynomial fit approach
    pCLMatrix       = airfoilPolars.pCLMatrixReduced;
    pCLalphaMatrix 	= airfoilPolars.pCLalphaReducedMatrix;
    alpha_deg       = AoA_deg + alphai_rad * 180/pi;
    a               = sum(pCLalphaMatrix .* [alpha_deg'.^2 alpha_deg' ones(length(alpha_deg), 1)], 2);
    cl              = sum(pCLMatrix .* [alpha_deg'.^3 alpha_deg'.^2 alpha_deg' ones(length(alpha_deg), 1)], 2);
    alpha0_rad      = (-cl'./a' + alpha_deg) * pi/180;
    a               = a' * 180/pi;
    
else
    
    % Local fit with second order derivative
    alpha_deg       = AoA_deg - alphai_rad * 180/pi;
    alpha_deg(alpha_deg > max(airfoilPolars.alphas_deg)) = max(airfoilPolars.alphas_deg);
    alpha_deg(alpha_deg < min(airfoilPolars.alphas_deg)) = min(airfoilPolars.alphas_deg);
    daP_deg         = 1e-1 * ones(1, length(alpha_deg));
    daM_deg         = -daP_deg;
    
    alphaPlus_deg   = alpha_deg + daP_deg;
    alphaMinus_deg  = alpha_deg + daM_deg;
    
    alphaPlus_deg(alphaPlus_deg > max(airfoilPolars.alphas_deg)) = max(airfoilPolars.alphas_deg);
    alphaMinus_deg(alphaMinus_deg < min(airfoilPolars.alphas_deg)) = min(airfoilPolars.alphas_deg);
    daP_deg = alphaPlus_deg - alpha_deg;
    daP_deg(abs(daP_deg) < 1e-4) = 1;
    daM_deg = alphaMinus_deg - alpha_deg;
    daM_deg(abs(daM_deg) < 1e-4) = 1;
    
    for s = 1:length(alphai_rad)
        cl(s, 1)        = interp1q(airfoilPolars.alphas_deg', airfoilPolars.clsReduced(s, :)', alpha_deg(s));
        clPlus          = interp1q(airfoilPolars.alphas_deg', airfoilPolars.clsReduced(s, :)', alphaPlus_deg(s));
        clMinus         = interp1q(airfoilPolars.alphas_deg', airfoilPolars.clsReduced(s, :)', alphaMinus_deg(s));
        aPlus           = (clPlus - cl(s))./daP_deg(s);
        aMinus          = (clMinus - cl(s))./daM_deg(s);
        a(s, 1)         = (aMinus + aPlus)/2' * 180/pi;
        alpha0_rad      = (-cl(s)'./a(s)' + alpha_deg) * pi/180;
    end  
    
end

end

function V = Unfold(v)
    V = [v v(end-1:-1:1)];
end
