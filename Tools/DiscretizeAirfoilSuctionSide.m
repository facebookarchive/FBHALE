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
  
function [XSuction, YSuction] = DiscretizeAirfoilSuctionSide(xSuction, ySuction, N)
% This function discretizes the suction side of an airfoil given its
% non-dimentional coordinates and the requested number of points.
% To do so, it computes the curvilinear abscissa over the suction side and
% splits it evenly.

% Compute curvilinear absicssa
ds = sqrt(diff(xSuction).^2 + diff(ySuction).^2);
s0 = [0; cumsum(ds)];

% Split s into N segments
s = linspace(0, max(s0), N);

% interpolate airfoil on these new points
XSuction = interp1(s0, xSuction, s);
YSuction = interp1(s0, ySuction, s);

end
