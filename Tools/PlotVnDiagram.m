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
  
figure
hold on 

plot(V*sqrt(rho_kgm3/rhoSL_kgm3), n_pos, 'color', 'b', 'LineWidth', 2)
plot(V*sqrt(rho_kgm3/rhoSL_kgm3), optim.wing.vn.nmax*ones(length(V)), '--' , 'color', 'r', 'LineWidth', .5)
plot([Vdive_ms Vdive_ms]*sqrt(rho_kgm3/rhoSL_kgm3),[nVdgust 1-(nVdgust-1)], '--' , 'color', 'k', 'LineWidth', .5)

[rho_kgm3, ~] = getAtmosphereProperties(optim.mission.maxh_m);
rhoSL_kgm3    = 1.225;

% plot n max lines
plot(V*sqrt(rho_kgm3/rhoSL_kgm3), optim.wing.vn.nmax*ones(length(V)), '--' , 'color', 'r', 'LineWidth', .5)
plot(V*sqrt(rho_kgm3/rhoSL_kgm3), optim.wing.vn.nmin*ones(length(V)), '--' , 'color', 'r', 'LineWidth', .5)

% plot npos
plot(V*sqrt(rho_kgm3/rhoSL_kgm3), n_pos, 'color', 'b', 'LineWidth', 2)
plot(V*sqrt(rho_kgm3/rhoSL_kgm3),n_neg_trimmed, 'color', 'b', 'LineWidth', 2)
plot([V(end)*sqrt(rho_kgm3/rhoSL_kgm3) V(end)*sqrt(rho_kgm3/rhoSL_kgm3)] , [n_neg_trimmed(end) n_pos(end)], 'color', 'b', 'LineWidth', 2)

plot([0 Vcruise_ms*sqrt(rho_kgm3/rhoSL_kgm3)] , [1 nVcgust_orig], '--' , 'color', 'k')
plot([0 Vcruise_ms*sqrt(rho_kgm3/rhoSL_kgm3)] , [1 1-(nVcgust_orig-1)], '--' , 'color', 'k')
plot(Vcruise_ms*sqrt(rho_kgm3/rhoSL_kgm3),nVcgust_orig, 'o', 'color', 'k')
plot(Vcruise_ms*sqrt(rho_kgm3/rhoSL_kgm3),1-(nVcgust_orig-1), 'o', 'color', 'k')


plot([Vcruise_ms Vdive_ms]*sqrt(rho_kgm3/rhoSL_kgm3) , [nVcgust_orig nVdgust], '--' , 'color', 'k', 'LineWidth', .5)
plot([Vcruise_ms Vdive_ms]*sqrt(rho_kgm3/rhoSL_kgm3) , [1-(nVcgust_orig-1) 1-(nVdgust-1)],'--' , 'color', 'k', 'LineWidth', .5)
plot(Vdive_ms*sqrt(rho_kgm3/rhoSL_kgm3),nVdgust, 'o', 'color', 'k')
plot(Vdive_ms*sqrt(rho_kgm3/rhoSL_kgm3),1-(nVdgust-1), 'o', 'color', 'k')

plot([Vdive_ms Vdive_ms]*sqrt(rho_kgm3/rhoSL_kgm3),[nVdgust 1-(nVdgust-1)], '--' , 'color', 'k', 'LineWidth', .5)

xlabel('EAS (m/s)')
ylabel('Load Factor (gs)')

legend('Vn Envelope','nmax/nmin','Vertical Gusts')
