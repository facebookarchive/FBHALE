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
  
% This function calls interp1q for fast interpolation while ensuring
% correct inputs defintion
function y = interp1f(u, v, x)

% Check inputs are column vectors
if size(u, 2) ~= 1
    u = u';
end
if size(v, 2) ~= 1
    v = v';
end
flipY = false;
if size(x, 2) ~= 1
    x = x';
    flipY = true;
end

% Check u is increasing on average
if mean(diff(u)) < 0
    u = u(end:-1:1);
    v = v(end:-1:1);
end

% Interpolate
y = interp1q(u, v, x);
if flipY
    y = y';
end
end