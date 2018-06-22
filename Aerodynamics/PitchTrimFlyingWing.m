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
  
function optim = PitchTrimFlyingWing(optim, iterCnt)
% This function ensures CM=0 at cruise for flying wing aircraft.
% This is enforced by displacing the battery pods in the stream-wise 
% direction.

% Setting up function for solving trim
[f, J, getOptim]   = TrimError(optim);
        
% Try solving with simple interpolation-based method.
% On the first iteration cast a wide net. On the subsequent, carry a local
% search.
X0     = optim.batteries.xCG_m(2);
if iterCnt == 1
    X0 = X0*linspace(-.5, 3, 5);
else
	X0 = X0*linspace(.8, 1.2, 3);
end

for x = 1:length(X0)
    def(x) = f(X0(x));
end
    
index = find(def(1:end-1).*def(2:end)<0, 1, 'first');
index = max(1, index-1);
index = min(length(X0), index+1);

if ~isempty(index)
    X1 = interp1(def(index-1:index+1), X0(index-1:index+1), 0);
else
    X1 = interp1(def, X0, 0, 'linear', 'extrap');
end
f(X1); % Run to gather updated optim

% Fetch optim containing solution
optim = getOptim();
optim = UpdateMassProperties(optim, 'analytical');

end

% Output functions used to solve the battery x-displacement for 
% longitudinal trim.
function [f, j, g] = TrimError(optim)

% Declare function handles
f = @objective;
j = @getJacobian;
g = @getOptim;

% Declare Jacobian
J = zeros(1, 1);

% Objective function for the Newton method
    function Y = objective(X)
                
        % Update battery position along with the deflection impact on the
        % assembly
        optim.wing.x_m = optim.wing.x_m + optim.wing.dxDue2Deflections_m;
                
        % Assign
        optim.batteries.xCG_m(2) = X(1);

    	% Update CG and battery weight (deflected CG)
        optim = UpdateMassProperties(optim, 'analytical');
        
        % Remove deflection from wing assembly to reflect its jig position
        optim.wing.x_m = optim.wing.x_m - optim.wing.dxDue2Deflections_m;
                
        % Shape fuselage
        optim = ShapeFuselage(optim);
        
        % Winglet sizing
        optim = SizeWinglets(optim);
        
        % Check elevon deflection at cruise
        F2_deg = RunASWINGElevonDef(optim);
        
        % Output deflection
        Y(1) = F2_deg;
       
end

% Function to retrieve Jacobian
    function Jacobian = getJacobian(X)
        Jacobian = J;
    end

% Function to retrieve optim after solving. This is useful to avoid
% resolving after the Newton method was run.
    function optim1 = getOptim()
        optim1.wing.winglet =rmfield(optim.wing.winglet, 'zoc0');
        optim1 = optim;
    end

end