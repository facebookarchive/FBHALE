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
  
function [optim] = CheckRollRate(optim, iterCnt)
% If ailerons are used for roll control, size them per a target steady roll
% rate. Otherwise, find the required actuator deflection to achieve the
% target rate.

rollSurface = optim.aircraft.controlSurfaces.roll.surfaceName;
rollControlSurfaceNumber = optim.aircraft.controlSurfaces.roll.contolSurfaceIndex;

% If roll comes from wing control surfaces size ailerons otherwise just check roll.
if strcmp('wing', rollSurface)
    [optim, p] = SizeAilerons(optim, iterCnt);
else
    optim = RunASWINGRollRateCheck(optim);
    p     = optim.aircraft.controlSurfaces.roll.minRollRate ;
end

%Set Delta Roll Rate
optim.aircraft.controlSurfaces.roll.deltaRollRate = p - ...
        optim.aircraft.controlSurfaces.roll.requiredRollRate;  
end

function [optim, p] = SizeAilerons(optim, iterCnt)

% If this is the first iteration and ailerons are used for roll, then size.
if iterCnt == 1 
        % Find aileron starting steady roll rate for a few starting span locations
        yobob2 = [0.3 0.8];
        
        for b = 1:length(yobob2)
            [rollrate(b), ~] = SteadyRollRate(yobob2(b), optim);
        end
        
        % Find the value that will match requirement
        dp    = ( rollrate - optim.aircraft.controlSurfaces.roll.requiredRollRate );
        yobo2 = interp1(dp, yobob2, 0);
        if isnan(yobo2) && min(dp) >= 0
            yobo2 = 0.8;
        elseif isnan(yobo2) && max(dp) <= 0
            yobo2 = 0.3;
        end
        optim.wing.controlSurface(1).yobo2(1) = yobo2;

end

% Record
[p, optim] = SteadyRollRate(optim.wing.controlSurface(1).yobo2(1), optim);

% Make sure we are not too far out off
iter = 0;
dy   = 1;
while abs(p - optim.aircraft.controlSurfaces.roll.requiredRollRate ) > .1 * optim.aircraft.controlSurfaces.roll.requiredRollRate  && iter < 4 && dy > 1e-3
    iter = iter + 1;
    
    if iter == 1
        yobo2 = optim.wing.controlSurface(1).yobo2(1) * (1 + .1 * sign(p - optim.aircraft.controlSurfaces.roll.requiredRollRate ));
    else
        yobo2 = optim.wing.controlSurface(1).yobo2(1) + dpdy * (p - optim.aircraft.controlSurfaces.roll.requiredRollRate );
		if yobo2 < .3
			yobo2 = .3;
		end 
		if yobo2 > .9
            yobo2 = .9;
        end
    end
    
    [p1, optim1] = SteadyRollRate(yobo2, optim);
    dpdy = (p1 - p)/(yobo2 - optim.wing.controlSurface(1).yobo2(1));
    dy   = dpdy * (p - optim.aircraft.controlSurfaces.roll.requiredRollRate);
    p    = p1;
    optim = optim1;
        
end

end
 
% We estimate the rolling moment increase from the deflection
function [p, optim] = SteadyRollRate(X, optim)
    optim.wing.controlSurface(1).yobo2(1) = X;
    optim = RunASWINGRollRateCheck(optim);
    p     = optim.aircraft.controlSurfaces.roll.minRollRate;
end
