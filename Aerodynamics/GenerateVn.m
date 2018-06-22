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
  
function optim = GenerateVn(optim)
% This function generates the Vn diagram based on the wing aerodynamic
% quantities.

% Find TAS at max altitude using specified cruise CL
VCruise_EASms = sqrt(2*optim.MGTOW_kg*optim.constants.g_ms2/(optim.aircraft.Sref_m2*optim.constants.rhoSL_kgm3*optim.aircraft.aero.CLcruise));

% find critical speeds for Vn - Vstall, Vdive
VStall_EASms  = sqrt(optim.MGTOW_kg*optim.constants.g_ms2*2/(optim.constants.rhoSL_kgm3*optim.aircraft.Sref_m2*optim.wing.aero.CLmax));
VDive_EASms   = 1.4*VCruise_EASms;

% Find CL at dive speed
CLdive = optim.aircraft.aero.CLcruise * (VCruise_EASms/VDive_EASms)^2;

% Find dalpha gust at Vc and Vd
daVcgust_rad   = atan(optim.wing.vn.VzCruiseGustEAS_ms/VCruise_EASms);
daVdgust_rad   = atan(optim.wing.vn.VzDiveGustEAS_ms/VDive_EASms);

% Find CL gust then calculate gust loads
CLVcgust  = optim.aircraft.aero.CLcruise + daVcgust_rad*optim.wing.aero.CLalpha;
CLVdgust  = CLdive                   + daVdgust_rad*optim.wing.aero.CLalpha;
nVcgust   = 1/2*optim.constants.rhoSL_kgm3.*VCruise_EASms.^2*CLVcgust*optim.wing.Sref_m2/(optim.MGTOW_kg*optim.constants.g_ms2);
nVdgust   = 1/2*optim.constants.rhoSL_kgm3.*VDive_EASms.^2*CLVdgust*optim.wing.Sref_m2/(optim.MGTOW_kg*optim.constants.g_ms2);

% Create velocity vector from 0 to Vd
VEAS_ms = linspace(0,  VDive_EASms, 100);

% Create lift limit curves
n_pos  = 1/2*optim.constants.rhoSL_kgm3.*VEAS_ms.^2*optim.wing.aero.CLmax*optim.wing.Sref_m2/(optim.MGTOW_kg*optim.constants.g_ms2);
n_neg  = 1/2*optim.constants.rhoSL_kgm3.*VEAS_ms.^2*optim.wing.aero.CLmin*optim.wing.Sref_m2/(optim.MGTOW_kg*optim.constants.g_ms2);

% Check if load factor of lift limit curve at Vd is greater than nmax and
% set flag to indicate if aircraft can trim at nmax vd. 
if n_pos(end) <= optim.wing.vn.nmax
    optim.wing.vn.trimmedAtVneCLmax = 'true';
elseif n_pos(end) > optim.wing.vn.nmax
    optim.wing.vn.trimmedAtVneCLmax = 'false';
end

% if lift limit curve exceeds ncrit trim with ncrit value
n_pos_trimmed = n_pos;
n_neg_trimmed = n_neg;

% Get the upper right corner of the vn
if nVdgust > n_pos_trimmed(end) && nVdgust < n_pos(end)
    optim.wing.vn.nTopRightCorner    = n_pos_trimmed(end);
    optim.wing.vn.vEasTopRightCorner_ms = VDive_EASms;
else
    optim.wing.vn.nTopRightCorner       = n_pos_trimmed(end);
    optim.wing.vn.vEasTopRightCorner_ms = VDive_EASms;
end 

% Check if Vc gust load factor is higher than max normal force at Vc and if
% so set gust gust load factor to max normal force
limitNAtVc = 1/2*optim.constants.rhoSL_kgm3*VCruise_EASms^2* optim.wing.aero.CLmax*optim.wing.Sref_m2/(optim.MGTOW_kg*optim.constants.g_ms2);
nVcgust = min(limitNAtVc, nVcgust);

% Get the max load factor point from Vn
if nVcgust > optim.wing.vn.nTopRightCorner
    optim.wing.vn.nmax_realized 	= nVcgust;
    optim.wing.vn.vEasnmax_ms       = VCruise_EASms;
else
    optim.wing.vn.nmax_realized 	= optim.wing.vn.nTopRightCorner;
    optim.wing.vn.vEasnmax_ms       = optim.wing.vn.vEasTopRightCorner_ms;
end

optim.wing.vn.vCruise_ms  =  VCruise_EASms;

end