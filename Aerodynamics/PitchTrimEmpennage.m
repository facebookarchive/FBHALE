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
  
function optim = PitchTrimEmpennage(optim)
% This function ensures CM=0 at cruise for empennage stabilized aircraft.
% This is enforced by displacing the wing.

% Setting up function for solving trim
[f, J, getOptim]   = TrimError(optim);
        
% Try solving with large relaxation factor. If it doesn't converge, then
% reboot and reduce
relax  = .7;
X0     = [optim.wing.x_m optim.htail.VH * optim.wing.Sref_m2 * optim.wing.cref_m / (optim.htail.x_m - optim.xCGDef_m)];
[~, Y] = Newton(f, X0, 'J', J, 'relax', relax, 'Nmax', 20);

while max(abs(Y)) > .1 && relax > 5e-2
    relax = relax/2;
    [~, Y] = Newton(f, X0, 'J', J, 'relax', relax, 'Nmax', 10);
end

% Fetch optim containing solution
optim = getOptim();
optim = UpdateMassProperties(optim, 'analytical');

end

% Output functions used to solve the wing x-location for longitudinal trim
% as well as the v-tail size.
function [f, j, g] = TrimError(optim)

% Declare function handles
f = @objective;
j = @getJacobian;
g = @getOptim;

% Declare Jacobian
J = zeros(2, 2);

% Objective function for the Newton method
    function Y = objective(X)
        % Update wing position along with the deflection impact on the
        % assembly
        optim.wing.x_m      = X(1) + optim.wing.dxDue2Deflections_m;
                
        % Assign
        optim.htail.Sref_m2 = X(2);
        optim.htail.bref_m  = sqrt(optim.htail.AR * X(2));
        optim.htail.c_m     = X(2) / optim.htail.bref_m * [1 1];

    	% Update CG and battery weight (deflected CG)
        optim = UpdateMassProperties(optim, 'analytical');
        
        % Remove deflection from wing assembly to reflect its jig position
        optim.wing.x_m = optim.wing.x_m - optim.wing.dxDue2Deflections_m;
                
        % Shape fuselage
        optim = ShapeFuselage(optim);
        
        % Vertical tail sizing
        optim = SizeVerticalTail(optim);
        
        % Update deflected CG and battery weight
        optim.wing.x_m = optim.wing.x_m + optim.wing.dxDue2Deflections_m;
        optim = UpdateMassProperties(optim, 'analytical');
        optim.wing.x_m = optim.wing.x_m - optim.wing.dxDue2Deflections_m;
        
        % Error based on target volume coefficient
        lh_m = optim.htail.x_m - optim.xCG_m;
        Sh_m2 = optim.htail.VH * optim.wing.Sref_m2 * optim.wing.cref_m / lh_m;
        dSh_m2 = Sh_m2 - X(2);
 
        % Compute longitudinal aero requirements
        persistent sectionPolars % the htail section polars take a while to load, so record them to avoid doing it multiple times
        if ~isempty(sectionPolars)
            optim.htail.aero.sectionPolars = sectionPolars;
        end
        optim = LongitudinalTrim(optim);
        
        sectionPolars       = optim.htail.aero.sectionPolars;
        Y(1)                = optim.aircraft.aero.CMtrim;
        Y(2)                = dSh_m2;
        
        % Construct Jacobian
        % Compute dxCGdSh by moving the wing assembly
        optim.htail.Sref_m2 = X(2)*1.02;
        optim.htail.bref_m  = sqrt(optim.htail.AR * optim.htail.Sref_m2);
        optim.htail.c_m     = optim.htail.Sref_m2 / optim.htail.bref_m * [1 1];
        optim.wing.x_m      = optim.wing.x_m + optim.wing.dxDue2Deflections_m;
        optim               = UpdateMassProperties(optim, 'analytical');
        optim.wing.x_m      = optim.wing.x_m - optim.wing.dxDue2Deflections_m;
        optim               = SizeVerticalTail(optim);
        optim               = ShapeFuselage(optim);
        optim.wing.x_m      = optim.wing.x_m + optim.wing.dxDue2Deflections_m;
        optim               = UpdateMassProperties(optim, 'analytical');
        optim.wing.x_m      = optim.wing.x_m - optim.wing.dxDue2Deflections_m;
        dxCGdSh             = (optim.xCG_m - optim.xCG_m)/(optim.htail.Sref_m2 - X(2));
                
        dxCGdxw              = optim.wing.dxCGdw;
              
        lh = optim.htail.x_m - optim.xCG_m;
        dlwdSh = -dxCGdSh;
        dlhdSh = -dxCGdSh;
        dlwdxw = 1 - dxCGdxw;
        dlhdxw = -dxCGdxw;
        
        % Jacobian
        J(1, 1) = optim.wing.aero.dCMdl*dlwdxw + optim.htail.aero.dCMdl*dlhdxw; % dCMdxw  
        J(1, 2) = optim.htail.aero.dCMdS + optim.wing.aero.dCMdl*dlwdSh + optim.htail.aero.dCMdl*dlhdSh; % dCMdSh
        J(2, 1) = - optim.htail.VH * optim.wing.Sref_m2 * optim.wing.cref_m / lh^2 * dlhdxw; % dDeltaShdxw
        J(2, 2) = -1 - optim.htail.VH * optim.wing.Sref_m2 * optim.wing.cref_m / lh^2 * dlhdSh; % dDeltaShdSh
        
      	% Ensure that solving variables are properly set
     	optim.htail.Sref_m2 = X(2);
        optim.htail.bref_m  = sqrt(optim.htail.AR * optim.htail.Sref_m2);
        optim.htail.c_m     = optim.htail.Sref_m2 / optim.htail.bref_m * [1 1];
        optim.wing.x_m      = X(1);
	end

% Function to retrieve Jacobian
    function Jacobian = getJacobian(X)
        Jacobian = J;
    end

% Function to retrieve optim after solving. This is useful to avoid
% resolving after the Newton method was run.
    function optim1 = getOptim()
        optim1 = optim;
    end

end