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
  
function optim = UpdateGeometry (optim, entity, idx)
% This script discretizes aero geometric properties (twist, chord) onto the
% refined structure grid

if ~optim.(entity).structure.init  
    % Proceed only if not operating in initialize mode
    
    switch entity
        
        case 'wing'
            optim.wing.structure.twist_deg{idx} 	= ...
            interp1(optim.wing.yobo2, optim.wing.twists_deg, optim.wing.structure.spanLocation);
            
        case 'htail'
            optim.htail.structure.twist_deg{idx} = ...
                interp1 ([0,1],optim.htail.twists_deg,linspace(0,1,optim.htail.structure.numStations));
            optim.htail.structure.length_m(idx) = sqrt(optim.htail.AR*optim.htail.Sref_m2)*0.5;
            optim.htail.structure.chord_m{idx} = ...
                ScaleTimeBulletProof (optim.htail.c_m,[0,1],linspace(0,1,optim.htail.structure.numStations));
            
        case 'vtail'
            optim.vtail.structure.length_m(idx) = sqrt(optim.vtail.AR*optim.vtail.Sref_m2)*0.5;
            optim.vtail.structure.chord_m{idx} = ...
                ScaleTimeBulletProof (optim.vtail.c_m,[0,1],linspace(0,1,optim.vtail.structure.numStations));
            
    end
      
end




