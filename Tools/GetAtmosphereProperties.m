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
  
% Given an altitude this function returns the appropriate air density and
% dynamic viscosity based on the international standard atmosphere
function [ rho_kgm3, dynamicViscosity_Nsm2, varargout ] = GetAtmosphereProperties(h_m)

   load('atmo_properties.mat')
   
   rho_kgm3               = interp1f(alt_m_vect, rho_kgm3_vect, h_m);
   dynamicViscosity_Nsm2  = interp1f(alt_m_vect, dynamicViscosity_vect, h_m);

   % Add csound_ms as an optional output. Won't break the current code or
   % make it slower
   if nargout > 2
       csound_ms = interp1f(alt_m_vect, csound_ms_vect, h_m);
       varargout{1} = csound_ms;
   end
end

