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

function [X, varargout] = Newton(f, X0, varargin)
 % This function uses a Newton method to drive an arbitrary function f to 
% zero.
%
% Inputs:
%       f:  function of vector X, outputs a column or line vector of size
%       (nx1) or (1xn),
%       X0: Initial guess. Column or line vector of size (mx1) or (1xm),
%
% Optional Inputs:
% The optional inputs are specified by a label of the input followed by its
% value, see example.
%       fTol: f tolerance i.e exit criteria based on final value of f. 
%       Default is 1e-5.
%       J: the Jacobian function of X
%       relax: Relaxation factor to help the method converge. Default is 0.5.
%       Nmax: Maximum number of iterations. Default is 100.
%       verbose: Boolean flag i.e 0 or 1 to output warnings and method 
%       iteration results. Default is 0.
%
% Outputs:
%       X: Column or line vector of size (mx1) or (1xm) where f is zero.
%
% Optional Outputs:
%       Y: Column or line vector of size (mx1) or (1xm) of f at ouput 
%       vector X.
%
% Example:  f  = @(X)([X(1)^2 + X(2); 2 * X(2) + 1]);
%           X0 = [1 1];
%           [X, Y] = Newton(f, X0, 'fTol', 1e-5, 'verbose', 1, 'relax', 1);

% Initial method parameters
relax   = 0.5;
Nmax    = 100;
fTol    = 1e-5;
verbose = 0;
JacobianProvided = false;

% Populate method parameters with optional inputs
nOptional = nargin - 2;

if nOptional > 0
    
    % Bullet proofing
    if mod(nOptional, 2) > 0
        error('Either a label or a value is missing for one of the optional inputs');
    else
        % Sweep optional inputs
        for i = 0:nOptional/2-1
            switch varargin{2*i+1}
                case 'relax'
                    relax = varargin{2*(i+1)};
                case 'Nmax'
                    Nmax = varargin{2*(i+1)};
                case 'fTol'
                    fTol = varargin{2*(i+1)};
                case 'verbose'
                    verbose = varargin{2*(i+1)};
                case 'J'
                    J = varargin{2*(i+1)};
                    JacobianProvided = true;
            end
        end
    end
end

if ~JacobianProvided
    % Bullet proofing
    [J0, Y0]  = Jacobian(f, X0);
else
    Y0 = f(X0);
    J0 = J(X0);
end
    
if max(max(isnan(J0))) || max(max(isnan(Y0)))
	error('The initial Jacobian or function evaluation contains a NaN');
end

% Initialize
N  = 0;

% Core method
while max(abs(Y0)) > fTol
    X1 = X0 - reshape(J0\reshape(Y0, length(Y0), 1)*relax, size(X0));
    X0 = X1;
    N  = N + 1;
    if ~JacobianProvided
        [J0, Y0]  = Jacobian(f, X0);
    else
        Y0 = f(X0);
        J0 = J(X0);
    end
    
    if verbose == 1
        disp(['Iteration #' num2str(N) ': Y = ' num2str(reshape(Y0, 1, length(Y0)))]);
    end
    
    if N > Nmax
        if verbose == 1
            warning('Maximum number of iterations reached');
        end
        break
    end
end

% Outputs
X = X0;
if nargout == 2
    varargout{1} = Y0;
end
end
