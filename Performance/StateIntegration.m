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
  
function [aircraft, freeStream, mission] = StateIntegration(aircraft, freeStream, mission)
% Integrate all states flagged for integration during a step computation
% e.g. energy remaining, ground position, altitude, etc...

    % State forward march
    for s = 1:length(aircraft.states.integration)
        eval([aircraft.states.integration(s).var '=' aircraft.states.integration(s).var '+' aircraft.states.integration(s).der '* mission.dt_s;']);
        
        % Enforce saturations
        if ~isempty(aircraft.states.integration(s).bounds)
            eval([aircraft.states.integration(s).var '= min(aircraft.states.integration(s).bounds(2), ' aircraft.states.integration(s).var ');']);
            eval([aircraft.states.integration(s).var '= max(aircraft.states.integration(s).bounds(1), ' aircraft.states.integration(s).var ');']);
        end
    end
       
end
