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
  
function [EquivalentMass_kg, EquivalentMassXLocation_kgm] = GetMasses(massNames, type, optim, varargin)
% This function automates the return of the weight and location for given
% masses. It is used to compute overall mass and CG.

    % If extra arguments are declared, initial mass and massXLocation are
    % provided
    if ~isempty(varargin)
        EquivalentMass_kg           = varargin{1};
        EquivalentMassXLocation_kgm = varargin{2};
    else
        EquivalentMass_kg           = 0;
        EquivalentMassXLocation_kgm = 0;
    end
           
    for m = 1:length(massNames)
        
      	% If mass name contains a '.' this means it is nested in the optim
        % structure
        if strfind(massNames{m}, '.')
            index    = strfind(massNames{m}, '.');
            massPath = optim.(massNames{m}(1:index-1)).(massNames{m}(index+1:end));
        else
            massPath = optim.(massNames{m});
        end

        % Declare individual functions per type of mass
        BeamMass_kg             = @(massName) sum(massPath.structure.mass_kg .* massPath.N);
        BeamMassXLocation_kg	= @(massName) sum(massPath.structure.mass_kg .* massPath.N .* massPath.xCG_m);
        PointMass_kg            = @(massName) sum(massPath.mass_kg .* massPath.N);
        PoinMassXLocation_kg	= @(massName) sum(massPath.mass_kg .* massPath.N .* massPath.xCG_m);
    
        switch type
            case 'beam'
                EquivalentMass_kg           = EquivalentMass_kg     + BeamMass_kg(massNames{m});
                EquivalentMassXLocation_kgm = EquivalentMassXLocation_kgm + BeamMassXLocation_kg(massNames{m});
            case 'point-mass'
                EquivalentMass_kg           = EquivalentMass_kg     + PointMass_kg(massNames{m});
                EquivalentMassXLocation_kgm = EquivalentMassXLocation_kgm  + PoinMassXLocation_kg(massNames{m});
        end
    end
    
end
