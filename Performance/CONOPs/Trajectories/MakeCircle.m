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
  
% This function creates the data relative to the circle trajectory. At each
% point we need the local:
%   1) s curvilinear abscissa
%   2) the ground path angle
function trajectory = MakeCircle(radius_m)

% Fixed number of points: 1000
N = 1000;

% Generate curve
theta_rad     	= linspace(pi/2, -3/2*pi, N); % Leave the discontinuities at the edges
trajectory.x_m 	= radius_m * cos(theta_rad);
trajectory.y_m	= radius_m * sin(theta_rad);

% Curvilinear abscissa
ds_m            = sqrt(diff(trajectory.x_m).^2 + diff(trajectory.y_m).^2);
trajectory.s_m	= [0 cumsum(ds_m)];

% Local radius of curvature
xp = - radius_m * sin(theta_rad);
yp = radius_m * cos(theta_rad);
xpp = - trajectory.x_m;
ypp = - trajectory.y_m;
trajectory.radius_m = (xp.^2 + yp.^2).^(3/2) ./ (xp.*ypp - yp.*xpp);

% Ground path angle: 0 means the ground speed is aligned with ey
t = [xp; yp];
normt = sqrt(xp.^2 + yp.^2);
t = t./normt;
trajectory.groundTrackAngle_rad = atan2(t(2, :), t(1, :));

end
