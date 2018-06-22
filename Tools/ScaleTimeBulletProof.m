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
  
function Y = ScaleTimeBulletProof(M, T, t)
% This function ensures the proper functioning of the ScaleTime routine. It
% bulletproofs against improper inputs that would yield a fatal crash.
% M is the matrix that is to be sliced. T is the 'time vector', and t is
% the time at which we want to slice.

% Bullet proof
if min(t) < min(T)
    t(t==min(t)) = min(T);
    warning('ScaleTime input saturated at min allowable value');
end
if max(t) > max(T)
    t(t==max(t)) = max(T);
    warning('ScaleTime input saturated at max allowable value');
end
if length(T) < 2 % Can't recover from that one
    error('Error in ScaleTime, the time vector was of size 1. We expect a vector of size > 1');
end
if sum(isnan(t) > 0)
    error('NaN input in ScaleTime. Cannot recover from that.');
end
if sum(isnan(T)) > 0
    warning('NaNs found in input time vector in ScaleTime, filling gaps...');
    N = length(T);
    indices = find(isnan(T));
    avgValues = (T(indices + 1) + T(indices - 1))/2;
    T(indices) = avgValues;
end

% Find indices for slicing
Ti = interp1f(T, 1:length(T), t);

% Slice
Y = ScaleTime(M, Ti);

end
