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
  
%Get average soalr intensity (W/m2) and length of night on winter solstice
%at specified latitude

 mission.starttime =  datetime(2016,12,20,0,00,0);
 
 mission.t_s     = 0;
 mission.leg.t_s = 0;
 mission.leg.h_m = 0;
 
 mission.lat = 25;
 mission.lon = 0;
 mission.timezone = 0;
 
 
 load('Solar/absorbed_intensity.mat');
 environment.solar.intensity_power       = intensity_power;
 environment.solar.intensity_altitudes   = intensity_altitudes;
 environment.solar.intensity_zeniths     = intensity_zeniths;

solar = [];
for i = 0:10:24*60*60
    
    mission.t_s = i;
    
    [ location, curTimeStruct ] = GetTimeLocation( mission );
    
    sunangle = sun_position(curTimeStruct, location);
    
    dim1 = round(location.altitude/500)+1;
    dim2 = round(sunangle.zenith/0.5)+1;
    [siz1, siz2] = size(environment.solar.intensity_power);
    if siz1 >= dim1 && siz2 >= dim2
        solarIntensity = environment.solar.intensity_power(dim1,dim2);
    else
        solarIntensity = 0;
    end
    
    solar = [solar solarIntensity];
    
end
