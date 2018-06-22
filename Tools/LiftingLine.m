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
  
function [wingPolar, airfoilPolars] = LiftingLine(x_m, y_m, z_m, c_m, twists_deg, AoA_deg, airfoilPolars, varargin)
% This lifting line method solves the strengths of the horseshoe vortices
% placed at the spanwise discretized quarter chord.
% The local lift coefficient is provided by either actual 2D polars or a
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
    wingPolar.tcp                   = {};
    wingPolar.alphai_rad            = {};
    wingPolar.gamma               	= {};
    wingPolar.n                     = [];
    wingPolar.CL                    = [];
    wingPolar.t                     = {};
    wingPolar.c_m                   = {};
    wingPolar.CDi                   = [];
    wingPolar.cl                    = {};
    wingPolar.f_lift_m2             = {};
    wingPolar.cdi                   = {}; 
    wingPolar.controlPoints_m       = {};
	wingPolar.isConverged           = [];
end

% Prepare geometry structure for twist and chord look-up based on span
if sum(y_m == 0) < 1
    t0 = min(abs(y_m));
    t   = [t0 t0+cumsum(sqrt( diff(x_m(y_m>=0)).^2 + diff(y_m(y_m>=0)).^2 + diff(z_m(y_m>=0)).^2)) ];
    t   = [-t(end:-1:1) t];
elseif min(y_m) < 0
    t0 = 0;
    t   = [t0 t0+cumsum(sqrt( diff(x_m(y_m>=0)).^2 + diff(y_m(y_m>=0)).^2 + diff(z_m(y_m>=0)).^2)) ];
    t   = [-t(end:-1:2) t];
else
    t0 = 0;
    t   = [t0 t0+cumsum(sqrt( diff(x_m(y_m>=0)).^2 + diff(y_m(y_m>=0)).^2 + diff(z_m(y_m>=0)).^2)) ];
end
t          = t/max(abs(t));
geometry.t = t;
geometry.c_m = c_m;
geometry.twist_deg = twists_deg;

% If there's already a flow field captured in polars, then use it as
% initial guess
if ~isempty(wingPolar.gamma)
    gamma      = wingPolar.gamma{end};
    t          = wingPolar.t{end};
    tcp        = wingPolar.tcp{end};
    CL0        = wingPolar.CL(end);
    CDi0       = wingPolar.CDi(end);
    n          = wingPolar.n(end);
else
    n          = 10;
    tcp        = t(1:end-1);
    gamma      = zeros(1, length(y_m)-1); % Should figure out an elliptical guess
    CL0        = 1;
    CDi0       = 1;
end

% Local representation
approach = 'local'; % options: local or polynomial or xfoil

% Initialize
n = 50;
error = 1;

while error > 1e-4 && n <= 100
    
    % Lifting line solve will rediscretize y_m and c_m based on n and
    % interpolate alphai_rad on that grid
    [tcp, gamma, alphai_rad, wingPolar0] = LiftingLineSolve(n, x_m, y_m, z_m, c_m, AoA_deg, twists_deg, gamma, tcp, airfoilPolars, approach, geometry);
    error = abs(wingPolar0.CL-CL0)./(abs(CL0)+ .01 * (abs(CL0)<1e-3)) + abs(wingPolar0.CDi-CDi0)./abs(CDi0);
    CL0   = wingPolar0.CL;
    CDi0  = wingPolar0.CDi;
    n     = floor(1.5*n);
    
end

% Storing
wingPolar.AoA_deg(end+1)   	= AoA_deg;
[wingPolar.tcp{end+1}, I]	= sort(tcp);
wingPolar.controlPoints_m{end+1} = wingPolar0.controlPoints_m(:, I);
wingPolar.cl{end+1}         = wingPolar0.cl(I);
wingPolar.f_lift_m2{end+1}  = sqrt( sum((wingPolar0.f_lift_m2(:, I))).^2 );
wingPolar.cdi{end+1}        = wingPolar0.cdi(I);
wingPolar.alphai_rad{end+1}	= alphai_rad(I);
wingPolar.n(end+1)         	= n;
wingPolar.CL(end+1)      	= wingPolar0.CL;
wingPolar.CDi(end+1)      	= wingPolar0.CDi;
wingPolar.gamma{end+1}   	= wingPolar0.gamma;
wingPolar.t{end+1}          = wingPolar0.t;
wingPolar.c_m{end+1}        = wingPolar0.c_m;
wingPolar.isConverged(end+1)= wingPolar0.isConverged;
end


function [tcp, gamma, alphai_rad, wingPolar] = LiftingLineSolve(n, x_m, y_m, z_m, c_m, AoA_deg, twists_deg, gamma, tcp0, airfoilPolars, approach, geometry)

% Check for problem symmetry. If it is symmetric, then use the same n number
% of span stations but for half
try
	SymmetricGeom	= [c_m; abs(y_m); twists_deg]';
    if sum(y_m == 0) > 0
        startIndex     	= find(y_m >= 0, 1, 'first');
    else
        startIndex     	= find(y_m >= 0, 1, 'first') - 1;
    end
    isSymmetricGeom	= max(max(abs(SymmetricGeom(1:startIndex-1, :) - SymmetricGeom(end:-1:startIndex+1, :)))) < 10*eps;
    isSymmetricAirfoil= max(max(abs(airfoilPolars.cls(1:startIndex-1, :) - airfoilPolars.cls(end:-1:startIndex+1, :)))) < 10*eps;
    isSymmetric = isSymmetricGeom & isSymmetricAirfoil;
catch
    isSymmetric = false;
end

isSymmetric = 0;

% Check if only half of the lifting surfaces was defined i.e symmetry is
% enforced
if min(y_m) >= 0
    
    % Mirror geometry
    x_m = Mirror(x_m, 1);
    y_m = Mirror(y_m, -1);
    z_m = Mirror(z_m, 1);
    c_m = Mirror(c_m, 1);
    twists_deg = Mirror(twists_deg, 1);
    
    % Mirror ADB
    airfoilPolars.t    = Mirror(airfoilPolars.t, -1);
    airfoilPolars.cls  = [airfoilPolars.cls(end:-1:2, :); airfoilPolars.cls];
    airfoilPolars.alphas_deg  = [airfoilPolars.alphas_deg(end:-1:2, :); airfoilPolars.alphas_deg];
    
    isSymmetric = true;
end

% Compute the quarter chord distance for discretization
if sum(y_m == 0) < 1
    t00  = min(abs(y_m));
    t0   = [t00 t00+cumsum(sqrt( diff(x_m(y_m>=0)).^2 + diff(y_m(y_m>=0)).^2 + diff(z_m(y_m>=0)).^2)) ];
    t0   = [-t0(end:-1:1) t0];
else
    t00  = 0;
    t0   = [t00 t00+cumsum(sqrt( diff(x_m(y_m>=0)).^2 + diff(y_m(y_m>=0)).^2 + diff(z_m(y_m>=0)).^2)) ];
    t0   = [-t0(end:-1:2) t0];
end

% Non-dimentionalize distance
t0 = t0/max(abs(t0));

% Rediscretize based on cos discretization
theta = linspace(0, pi/2, n/2+2);
theta = [theta(1:end-1) theta+pi/2];
thetaCP = theta(1:end-1) + diff(theta)/2;
t     = cos(theta);
tcp   = cos(thetaCP);

% Interpolate provided distributions on new grid
c           = interp1f(geometry.t, geometry.c_m, t')';
twist_rad   = interp1f(geometry.t, geometry.twist_deg, t')'*pi/180;
gamma       = interp1(tcp0', gamma, tcp, 'linear', 0);

airfoilPolars.clsReduced = interp1(airfoilPolars.tQC/max(airfoilPolars.tQC), airfoilPolars.cls, tcp);
if size(airfoilPolars.alphas_deg, 1) == 1
    airfoilPolars.AoAsReduced_deg = ones(length(t), 1) * airfoilPolars.alphas_deg;
else
    airfoilPolars.AoAsReduced_deg = interp1(airfoilPolars.tQC/max(airfoilPolars.tQC), airfoilPolars.alphas_deg, tcp);
end
airfoilPolars.approach   = approach;

% Locus of points
points(1, :) = interp1f(t0, x_m, t);
points(2, :) = interp1f(t0, y_m, t);
points(3, :) = interp1f(t0, z_m, t);

% Locus of control points
controlPoints(1, :) = points(1, 1:end-1) + (points(1, 2:end) - points(1, 1:end-1)) .* (tcp-t(1:end-1))./diff(t);
controlPoints(2, :) = points(2, 1:end-1) + (points(2, 2:end) - points(2, 1:end-1)) .* (tcp-t(1:end-1))./diff(t);
controlPoints(3, :) = points(3, 1:end-1) + (points(3, 2:end) - points(3, 1:end-1)) .* (tcp-t(1:end-1))./diff(t);

% Newton formulation
if isSymmetric
    X0 = gamma(1:length(gamma)/2);
    f = @(X) cat(2, X, X(end:-1:1));
else
    X0 = gamma;
    f = @(X) X;
end

F = @(X) LiftingLineSolver(airfoilPolars, AoA_deg, f(X), twist_rad*180/pi, points, t, controlPoints, tcp, c, approach, isSymmetric);
J = @(X) ComputeJacobian(airfoilPolars, AoA_deg, f(X), twist_rad*180/pi, points, t, controlPoints, tcp, c, approach, isSymmetric);
relax = 1;
[X1, Y1] = Newton(F, X0, 'J', J, 'relax', relax, 'Nmax', 20);

% If it didn't converge try with a lower relaxation factor and start with a
% fresh initial guess
while abs(norm(Y1)) > 1e-4 && relax > .1
    relax = relax/2;
    [X1, Y1] = Newton(F, X1, 'J', J, 'relax', relax, 'Nmax', 50);
end

% Record solution
[~, alphai_rad, cl, fli, cdi, CL, CDi, controlPoints, ~] = LiftingLineSolver(airfoilPolars, AoA_deg, f(X1), twist_rad*180/pi, points, t, controlPoints, tcp, c, approach, isSymmetric);
gamma      = f(X1);
alphai_rad = f(alphai_rad);
cl         = f(cl);
fli        = f(fli);
cdi        = f(cdi);

% Store
wingPolar.t         = t;
wingPolar.tcp       = tcp;
wingPolar.theta     = theta;
wingPolar.c_m       = c;
wingPolar.controlPoints_m = controlPoints;
wingPolar.twist_rad = twist_rad;
wingPolar.alphai_rad = alphai_rad;
wingPolar.gamma     = gamma;
wingPolar.cl        = cl;
wingPolar.cdi       = cdi;
wingPolar.f_lift_m2 = fli;
wingPolar.CL        = CL;
wingPolar.CDi       = CDi;

% Check for unconverged results
if abs(norm(Y1)) > 1e-4
    wingPolar.isConverged = false;
else
    wingPolar.isConverged = true;
end

end

function [cl_error, alphai_rad, cl, fli, cdi, CL, CDi, controlPoint, J] = LiftingLineSolver(airfoilPolars, AoA_deg, Gamma, twists_deg, points, t, controlPoint, tcp, chords, approach, isSymmetric)

% Compute cl and alphai from Gamma distribution

    % Distances between control points and vortex legs
    r1 = repmat(points(:, 1:end-1), 1, size(controlPoint, 2));
    r1 = reshape(r1, [size(points(:, 1:end-1)) size(controlPoint, 2)]);
    cp = repmat(eye(3), size(controlPoint, 2), 1) * controlPoint;
    cp = reshape(cp, [3  size(controlPoint, 2) size(controlPoint, 2)]);
    r1 = r1 - cp;
    r2 = repmat(points(:, 2:end), 1, size(controlPoint, 2));
    r2 = reshape(r2, [size(points(:, 2:end)) size(controlPoint, 2)]);
    r2 = r2 - cp;
    dl = r2 - r1;
    
    % Compute phi and i, local normal vector
    c1_m = chords(1:end-1);
    c2_m = chords(2:end);
  	x1_m = points(1, 1:end-1);
    x2_m = points(1, 2:end);
    y1_m = points(2, 1:end-1);
    y2_m = points(2, 2:end);
    z1_m = points(3, 1:end-1);
    z2_m = points(3, 2:end);
    
    phi_rad = atan2(diff(points(3, :)), abs(diff(points(2, :))));
    i_rad = interp1(t, twists_deg, tcp)*pi/180;
    
    for k = 1:length(phi_rad)
        iM  = [cos(i_rad(k)) 0 sin(i_rad(k));0 1 0; -sin(i_rad(k)) 0 cos(i_rad(k))];
        phiM= [1 0 0;0 cos(phi_rad(k)) sin(phi_rad(k));0 -sin(phi_rad(k)) cos(phi_rad(k))];
        un(k, :) = phiM * iM * [0; 0; -1];
        ua(k, :) = phiM * iM * [1; 0; 0];
    end
    
    % Compute nuij and cl
    % Only on half the geometry if it is considered symmetric
    Vinf = [cosd(AoA_deg); 0; sind(AoA_deg)];

    if isSymmetric
        n = ceil(size(controlPoint, 2)/2);
    else
        n = size(controlPoint, 2);
    end
    
    % Sum up contribution of bound vortices
    [ci, dA, nuij] = HorseShoeVortex(Vinf, r1, r2, c1_m, c2_m, y1_m, y2_m, z1_m, z2_m, x1_m, x2_m);

    G  = Gamma./(ci * norm(Vinf));
    Xi = (ones(3, 1) * ci./dA) .* squeeze(dl(:, :, 1));
    cl = 2 * sqrt(sum(   ( cross(Vinf + squeeze(sum(  nuij .* repmat(cat(3, ones(3, 1) * G, ones(3, 1) * G), 1, 1, floor(size(controlPoint, 2)/2) ) , 2 )), Xi  ) ).^2, 1 )) .* G;
    alphai_rad = atan( dot(Vinf + squeeze(sum(nuij .* repmat(cat(3, ones(3, 1) * G, ones(3, 1) * G), 1, 1, floor(size(controlPoint, 2)/2) ), 2)), un') ...
                     ./ dot(Vinf + squeeze(sum(nuij .* repmat(cat(3, ones(3, 1) * G, ones(3, 1) * G), 1, 1, floor(size(controlPoint, 2)/2) ), 2)), ua'));
    fli  = Gamma .* cross( Vinf + squeeze(sum( repmat(cat(3, ones(3, 1) * Gamma./ci, ones(3, 1) * Gamma./ci), 1, 1, floor(size(controlPoint, 2)/2) )  .* nuij, 2)), squeeze(dl(:, :,  1)));
    fli  = fli(:, 1:n);
    S_m2 = sum(dA(1:n).*cos(phi_rad(1:n)));
    cl   = cl(1:n);
    fli  = fli(:, 1:n);
    
% Compute cl distribution based on the polar. If symmetric, only carry over
% half the span
alphai_rad = alphai_rad(1:n);
[clalpha, ~, cl0] = SectionalData(airfoilPolars, AoA_deg*0, alphai_rad, approach);    
if isSymmetric
    clalpha = [clalpha; clalpha(end:-1:1)];
end

% Compute cl distribution based on the An values
cl_error = cl - cl0';

% Compute Jacobian
Xi = ones(3, 1) * (ci./dA)  .* squeeze(dl(:, :, 1));
nuji = permute(nuij, [1 3 2]);
GMat = repmat(ones(3, 1) * G, 1, 1, length(G));
sumNUIJG = squeeze( sum(nuij .* GMat, 2) );
omegai = Vinf + sumNUIJG;
Omegai = cross(omegai, Xi);

dRdG1 = diag( 2 * norm2(Omegai) );

dRdG1 = dRdG1 + 2 * (G ./ norm2(Omegai) .* norm2(Xi).^2)' * ones(1, length(G)) .* (...
           squeeze( dot( nuji, repmat(sumNUIJG, [1 1 length(G)]) ))  + ...
           squeeze( dot( repmat(Vinf, [1 length(G) length(G)]), nuji)) - ...
           squeeze( dot( repmat(omegai, [1 1 length(G)]), repmat(Xi, [1 1 length(G)])  ) .* dot( nuij.*GMat, repmat(Xi, [1 1 length(G)]))));
Ni     = squeeze(dot(Vinf + sumNUIJG, un'));
Di     = squeeze(dot(Vinf + sumNUIJG, ua'));
thetai = Ni./Di;
dRdG2  = (clalpha ./ (1 + thetai'.^2) ) * ones(1, length(G));
dRdG2  = dRdG2 .* (  (Di' * ones(1, length(G))) .* squeeze(dot(nuji, repmat(un', [1 1 length(G)]) )) - (Ni' * ones(1, length(G))) .* squeeze(dot(nuji, repmat(ua', [1 1 length(G)]))))./(Di'.^2 * ones(1, length(G)));      
J = 1./( ones(length(G), 1) * ci ) * norm(Vinf) .* ( dRdG1 + dRdG2 );

if isSymmetric % Split the Jacobian matrix up so we only solve half of the system
    J0 = J(1:n, 1:n);
    J1 = J(n+1:end, 1:n);
    J  = (J0 + J1(end:-1:1, :));
end

% Integrated forces
FL = sum(fli, 2);
C = [-cosd(AoA_deg) 0 -sind(AoA_deg); 0 0 0; sind(AoA_deg) 0 -cosd(AoA_deg)] * FL;
cdi  = (-cosd(AoA_deg) * fli(1, :) - sind(AoA_deg) * fli(3, :) )/(1/2*S_m2);
CL = C(3) /(1/2*S_m2);
CDi = C(1) /(1/2*S_m2);

end

function J = ComputeJacobian(airfoilPolars, AoA_deg, Gamma, twists_deg, points, t, controlPoints, tcp, chords, approach, isSymmetric)

[~, ~, ~, ~, ~, ~, ~, ~, J] = LiftingLineSolver(airfoilPolars, AoA_deg, Gamma, twists_deg, points, t, controlPoints, tcp, chords, approach, isSymmetric);

end

% Evaluate airfoil alpha0, cla, and cl at current net alpha (contains
% twist)
function [a, alpha0_rad, cl] = SectionalData(airfoilPolars, AoA_deg, alphai_rad, approach)

if strcmp(approach, 'polynomial')
    
    % Polynomial fit approach
    % If selected, the lifting line use the same description as ASWING:
    % CL(alfa) =  airfoilPolars.dCLda * (alfa + alfa0);
    alpha_deg 	= AoA_deg - alphai_rad * 180/pi;
    a           = airfoilPolars.ASWINGReducedQuantities.clalpha * ones(length(alphai_rad), 1);
    alpha0_rad  = airfoilPolars.ASWINGReducedQuantities.alpha0_deg * ones(length(alphai_rad), 1) * pi/180;
    cl          = a*pi/180.*(alpha_deg' - alpha0_rad * 180/pi);
 
elseif strcmp(approach, 'local')
    
    % Local fit with second order derivative
    alpha_deg       = AoA_deg - alphai_rad * 180/pi;
    
    for s = 1:length(alphai_rad)
        
        if alpha_deg(s) > max(airfoilPolars.AoAsReduced_deg(s, :))
            alpha_deg(s) = max(airfoilPolars.AoAsReduced_deg(s, :));
        elseif alpha_deg(s) < min(airfoilPolars.AoAsReduced_deg(s, :))
            alpha_deg(s) = min(airfoilPolars.AoAsReduced_deg(s, :));
        end
        daP_deg         = 1e-1 * ones(1, length(alpha_deg));
        daM_deg         = -daP_deg;
        
        alphaPlus_deg   = alpha_deg + daP_deg;
        alphaMinus_deg  = alpha_deg + daM_deg;
        
        if alphaPlus_deg(s) > max(airfoilPolars.AoAsReduced_deg(s, :))
            alphaPlus_deg(s) = max(airfoilPolars.AoAsReduced_deg(s, :));
        end
        if alphaMinus_deg(s) < min(airfoilPolars.AoAsReduced_deg(s, :))
            alphaMinus_deg(s) = min(airfoilPolars.AoAsReduced_deg(s, :));
        end
        daP_deg = alphaPlus_deg(s) - alpha_deg(s);
        daP_deg(abs(daP_deg) < 1e-4) = 1;
        daM_deg = alphaMinus_deg(s) - alpha_deg(s);
        daM_deg(abs(daM_deg) < 1e-4) = 1;
        
        cl(s, 1)        = interp1f(airfoilPolars.AoAsReduced_deg(s, :)', airfoilPolars.clsReduced(s, :)', alpha_deg(s));
        clPlus          = interp1f(airfoilPolars.AoAsReduced_deg(s, :)', airfoilPolars.clsReduced(s, :)', alphaPlus_deg(s));
        clMinus         = interp1f(airfoilPolars.AoAsReduced_deg(s, :)', airfoilPolars.clsReduced(s, :)', alphaMinus_deg(s));
        aPlus           = (clPlus - cl(s))./daP_deg;
        aMinus          = (clMinus - cl(s))./daM_deg;
        a(s, 1)         = (aMinus + aPlus)/2' * 180/pi;
        alpha0_rad      = (-cl(s)'./a(s)' + alpha_deg) * pi/180;
        alpha0_rad      = (-cl(s)./a(s)*180/pi + alpha_deg) * pi/180;
    end
    
end

end

function [ci, dAi, nuij] = HorseShoeVortex(Vinf, r1, r2, c1, c2, y1, y2, z1, z2, x1, x2)

% Compute average chord and area
ci   = 2/3 * (c1.^2 + c1.*c2 + c2.^2)./(c1 + c2);
dAi  = (c1 + c2)/2 .* sqrt( (x2 - x1).^2 + (y2 - y1).^2 + (z2 - z1).^2 );

% Reshape vectors to tackle all cross products for each control point
siz  = size(r1);
r10  = reshape(r1, 3, siz(2) * siz(3));
r20  = reshape(r2, 3, siz(2) * siz(3));
ci0  = repmat(ci, 1, siz(3));
 
% Carry the sum
nu   = @(r) cross(Vinf*ones(1, length(r10)), r)./(norm2(r).*(norm2(r) - dot(Vinf*ones(1, length(r10)), r)));
mu   = @(r_1, r_2) (norm2(r_1) + norm2(r_2)) .* cross(r_1, r_2) ./ (norm2(r_1) .* norm2(r_2) + dot(r_1, r_2))./(norm2(r_1) .* norm2(r_2));
areColinear = @(r_1, r_2) norm2(cross(r_1, r_2)) < 1e-8;

% Make sure we have mu12(:, i, i) = 0 for all i-s. If not treated, this
% incurs a division by zero resulting in NaNs. Likewise, if an Inf is found
% it means a proper numerical cancellation didn't occur and the result
% should be exactly zero.
mu12              = mu(r10, r20);
mu12Colinear      = areColinear(r10, r20);
mu12(:, mu12Colinear)= 0;
nuij = ones(3, 1) * ci0./(4*pi) .* (nu(r20) - nu(r10) + mu12); 

% Reshape back
nuij = reshape(nuij, 3, siz(2), siz(3));

end

% We need a function to compute the vector norm 2 in matrix format. This
% isn't provided by the off-the-shelf function 'norm'
function B = norm2(A)
    B = sqrt(sum(A.^2, 1));
end