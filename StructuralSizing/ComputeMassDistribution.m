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
 

function optim = ComputeMassDistribution(optim, varargin)
% This script computes the mass distribution of a given structural component using two methods:
% 1. Analytical: Provides updated mass distributions based on previously computed densities given a change in geometry
% 2. Co-blade: Calls coblade to provide mass distribution.
% The script is called by UpdateMassProperies. The analytical version is
% used for faster albeit less accurate evaluations.

for b = 1:length(optim.aircraft.beamNames)
    if ~max(strcmp(optim.aircraft.beamNames{b}, {'pod', 'boom'}))  % Update lifting surface and CG      
        optim = ComponentMassDistribution (optim,optim.aircraft.beamNames{b},varargin{1});
        optim.(optim.aircraft.beamNames{b}).xCG_m  = optim.(optim.aircraft.beamNames{b}).x_m + optim.(optim.aircraft.beamNames{b}).structure.xCG_m;
        
    else  % Update boom/pod CG
        optim = ComponentMassDistribution (optim,optim.aircraft.beamNames{b},varargin{1});
        optim.(optim.aircraft.beamNames{b}).xCG_m  = optim.(optim.aircraft.beamNames{b}).x_m + optim.(optim.aircraft.beamNames{b}).structure.yCG_m;
    end

end

end

function optim = ComponentMassDistribution (optim, entity, varargin)

switch varargin{1}
    
    case 'analytical'
        
        
        for idx = 1:length(optim.(entity).N)
            
        optim = UpdateGeometry (optim, entity, idx);    
        
        s = optim.(entity).structure;
        
        % Mass per unit length:
        massPerLength_kgm = s.mg_N_m2{idx}/9.81.*s.chord_m{idx}(:);
        
        if strcmp(entity, 'wing')
            length_m = optim.wing.bref_m*0.5/cosd(optim.wing.sweep_deg);
        else
            length_m = s.length_m(idx);
        end
        
        % Spanwise CG location:
        s.yCG_m(idx) = trapz(s.t{idx}(:)*length_m,s.t{idx}(:)*length_m.*massPerLength_kgm(:))/...
            trapz(s.t{idx}(:)*length_m, massPerLength_kgm(:));
        
        % Chordwise CG location:
        s.xCG_m(idx) = trapz(s.t{idx}(:)*length_m,(s.sRefX_m{idx}(:)+s.Ccg_m{idx}(:)).*massPerLength_kgm(:))/...
            trapz(s.t{idx}(:)*length_m, massPerLength_kgm(:));
        
        % Weight:
        s.mass_kg(idx) =  trapz(s.t{idx}(:)*length_m,massPerLength_kgm(:))*s.numInstances;
        end
        
        optim.(entity).structure = s;
        
    case 'co-blade'
        optim = CoBlade(optim, entity);
        
end
end