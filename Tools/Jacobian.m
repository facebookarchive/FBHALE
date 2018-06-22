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
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function calculates the Jacobian J of a function f at point X0.
% The function f can take a vector X of arbitrary dimension and output a
% vector Y also of any dimension.
% 09/16/2014
% Dorian Colas
%
% Inputs:
%       f:  function of vector X, outputs a column or line vector of size
%       (nx1) or (1xn),
%       X0: Column or line vector of size (mx1) or (1xm) where the jacobian
%       matrix is computed.
%
% Outputs:
%       J: Jacobian matrix of size (nxm) of function f at location X0.
%
% Optional Outputs:
%       Y0: Value of the function f at X0.
%
% Example:  f  = @(X)([X(1)^2 + X(2); 2 * X(2) + 1]);
%           X0 = [1 1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [J, varargout] = Jacobian(f, X0)

% Local value
Y0 = f(X0);
if nargout > 1
    varargout{1} = Y0;
end

% Dimensions
m = length(X0);
n = length(Y0);
J = zeros(n, m);

% Perturbation around X0
epsilon = .0001;

% Compute partial derivatives to populate Jacobian matrix J
for i = 1:n
    for j = 1:m
        X1 = X0;
        X2 = X0;
        
        % Local pertubation around initial X0 point
        if X0(j)~=0
            X1(j) = X0(j)*(1 - epsilon/2);
            X2(j) = X0(j)*(1 + epsilon/2);
        else
            X1(j) = epsilon/2;
            X2(j) = -epsilon/2;
        end
        Y1    = f(X1);
        Y2    = f(X2);
        
        % Partial derivative based on finite difference
        % Avoid going out of bounds for J
        if and(isnan(Y1), ~isnan(Y2))
            J(i,j)= (Y2(i) - Y0(i))/(X2(j) - X0(j));
        elseif and(isnan(Y2), ~isnan(Y1))
            J(i,j)= (Y1(i) - Y0(i))/(X1(j) - X0(j));
        else
            J(i,j)= (Y2(i) - Y1(i))/(X2(j) - X1(j));
        end
    end
end
end
