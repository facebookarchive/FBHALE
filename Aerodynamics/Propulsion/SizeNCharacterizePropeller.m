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
  
function optim = SizeNCharacterizePropeller(optim)
% This function sizes the propeller for the modified cruise condition. The
% resulting design is then characterized in order to find the worst case
% torque required from the motor during climb to satisfy a climb time
% requirement. Lastly, the performance maps are put uner a form compatible
% with rapid look-ups for mission perfomance calculations.

% Find cruise drag
hCruise_km = optim.mission.minh_m/1e3;
CL  	   = optim.aircraft.aero.CLcruise;
[rho_kgm3, dynamicViscosity_Nsm2, csound_ms] = GetAtmosphereProperties(hCruise_km*1e3);
V_ms       = sqrt(2 * optim.MGTOW_kg * optim.constants.g_ms2 ./ (rho_kgm3 .* CL * optim.aircraft.Sref_m2)); 
Re         = V_ms * optim.aircraft.cref_m .* rho_kgm3 ./ dynamicViscosity_Nsm2;
CD         = interp2f(optim.aircraft.aero.polar.AoA_deg, optim.aircraft.aero.polar.Re, optim.aircraft.aero.polar.CD, optim.aircraft.aero.alphaCruise_deg, Re);
D_N        = 1/2 * rho_kgm3 .* V_ms.^2 .* CD * optim.aircraft.Sref_m2;

% Propeller radius based on target loading
Aprop_m2 = D_N / optim.propulsion.propeller.cruiseDiskLoading_Nm2;
optim.propulsion.propeller.r_m = sqrt(Aprop_m2 / (pi * sum(optim.propulsion.N)));

% Find maximum RPM based on maximum altitude condition
h_m = optim.mission.maxh_m;
[rhoCeiling_kgm3, ~, csound_ms] = GetAtmosphereProperties(h_m);
RPMmax = optim.propulsion.propeller.tipMach * csound_ms / (2*pi*optim.propulsion.propeller.r_m) * 60;

% Translate maximum RPM to design point RPM
RPMcruise = RPMmax*sqrt(rhoCeiling_kgm3/rho_kgm3) - optim.propulsion.propeller.RPMmargin;

% Size propeller for cruise conditions
% Sizing provided by Mtip, cruise Vinf, and thrust
cd(optim.propDir);
commandSeq = 'plop \n';
bladeNum   = optim.propulsion.propeller.Nb;
R_m        = optim.propulsion.propeller.r_m;
Rhub_m     = optim.propulsion.propeller.hubRelativeRadius*R_m;
RhubWake_m = Rhub_m*.2;
VTAS_ms    = V_ms;
T_N        = D_N/sum(optim.propulsion.N);
cl         = [optim.propulsion.propeller.CLroot  optim.propulsion.propeller.CLtip]; % linear cl distribution
addTwist_deg = 0;
rotorName  = 'AQ3.prop';
airfoilSectionFile = [optim.propulsion.propeller.airfoilName '.aero'];

% Read airfoil alpha0 as xrotor doesn't seem to be able to read it
% correctly
fid = fopen(airfoilSectionFile, 'r');
alpha0_deg = [];
while isempty(alpha0_deg)
    tline = fgetl(fid);
    if contains(tline,  'Zero-lift alpha (deg):')
        alpha0_deg = str2num(tline(25:38));
    end
end
fclose(fid);

commandSeq = [commandSeq 'G F \n \n'];
commandSeq = [commandSeq 'aero\n'];
commandSeq = [commandSeq 'read\n'];
commandSeq = [commandSeq airfoilSectionFile '\n'];
commandSeq = [commandSeq 'edit \n'];
commandSeq = [commandSeq '1 \n'];
commandSeq = [commandSeq num2str(alpha0_deg) '\n\n\n'];
commandSeq = [commandSeq 'desi\n'];
commandSeq = [commandSeq 'atmo ' num2str(hCruise_km) '\n'];
commandSeq = [commandSeq 'inpu \n'];
commandSeq = [commandSeq num2str(bladeNum) '\n'];
commandSeq = [commandSeq num2str(R_m) '\n'];
commandSeq = [commandSeq num2str(Rhub_m) '\n'];
commandSeq = [commandSeq num2str(RhubWake_m) '\n'];
commandSeq = [commandSeq num2str(VTAS_ms) '\n'];
commandSeq = [commandSeq '0 \n'];
commandSeq = [commandSeq num2str(RPMcruise) '\n'];
commandSeq = [commandSeq num2str(T_N) '\n'];
commandSeq = [commandSeq num2str(cl) '.5 \n'];
commandSeq = [commandSeq 'CL \n'];
commandSeq = [commandSeq num2str(cl(1)) ' \n'];
commandSeq = [commandSeq num2str(cl(2)) ' \n'];
commandSeq = [commandSeq '\n \n \n'];
commandSeq = [commandSeq 'modi \n'];
commandSeq = [commandSeq 'tlin \n'];
commandSeq = [commandSeq num2str(addTwist_deg) ' \n'];
commandSeq = [commandSeq '\n \n'];
commandSeq = [commandSeq 'save ' num2str(rotorName) ' \n'];
commandSeq = [commandSeq 'Y \n'];
commandSeq = [commandSeq 'quit \n'];
Xrotor(commandSeq);

%% Characterize Propeller

pitches_deg = linspace(-25, 10, 10)';
for a = 1:length(pitches_deg)
    [CT(:, a), CP(:, a), CQ(:, a), lambda, RPM(:, a), BetaTip_deg(:, a)] = CharacterizePropeller(rotorName, pitches_deg(a), 2*R_m);
    
    % Clear NaNs
    flagNaNs = isnan(CT(:, a));
    CT(flagNaNs, a) = interp1(lambda(~flagNaNs), CT(~flagNaNs, a), lambda(flagNaNs), 'linear', 'extrap');
    CP(flagNaNs, a) = interp1(lambda(~flagNaNs), CP(~flagNaNs, a), lambda(flagNaNs), 'linear', 'extrap');
    CQ(flagNaNs, a) = interp1(lambda(~flagNaNs), CQ(~flagNaNs, a), lambda(flagNaNs), 'linear', 'extrap');
    RPM(flagNaNs, a) = interp1(lambda(~flagNaNs), RPM(~flagNaNs, a), lambda(flagNaNs), 'linear', 'extrap');
    
    % Efficiency
    eta(:, a) = CT(:, a) .* lambda ./ (CP(:, a));
    
    % Power spent at CT = 0 (for descent)
    CPCT0(a) = interp1f(CT(:, a), CP(:, a), 0); 
	JCT0(a)  = interp1f(CT(:, a), lambda, 0); 
end

cd(optim.rootDir);

% Store map
optim.propulsion.propeller.map.CT   = CT;
optim.propulsion.propeller.map.CP   = CP;
optim.propulsion.propeller.map.CQ   = CQ;
optim.propulsion.propeller.map.eta  = eta;
optim.propulsion.propeller.map.J    = lambda;
optim.propulsion.propeller.map.pitch_deg = pitches_deg;

% Store CP @ CT = 0 (helpful for descent power calculation)
optim.propulsion.propeller.map.CPCT0 = CPCT0;
optim.propulsion.propeller.map.JCT0 = JCT0;
    
%% Size propulsion system for climb mission

% Target climb rate: 
climbTime_s = optim.mission.climb.tclimb_min*60;

% Climb at CL = 1 for margin
CL       = 1;
V_ms     = @(h_m) sqrt( 2 * optim.MGTOW_kg * optim.constants.g_ms2 ./ (CL .* GetAtmosphereProperties(h_m) * optim.aircraft.Sref_m2));
Re       = @(h_m) GetRe(h_m, V_ms(h_m), optim.wing.cref_m);
CLvsA    = @(h_m) ScaleTimeBulletProof(optim.aircraft.aero.polar.CL, optim.aircraft.aero.polar.Re, Re(h_m));
AoA_deg  = @(h_m) interp1f(CLvsA(h_m), optim.aircraft.aero.polar.AoA_deg, CL);
CD       = @(h_m) interp2f(optim.aircraft.aero.polar.AoA_deg, optim.aircraft.aero.polar.Re, optim.aircraft.aero.polar.CD, AoA_deg(h_m), Re(h_m));

hVec_m      = linspace(0, optim.mission.climb.minh_m, 100);
for h = 1:length(hVec_m)
    PDrag_W(h) = CD(hVec_m(h)) * 1/2 * optim.aircraft.Sref_m2 .* GetAtmosphereProperties(hVec_m(h)) .* V_ms(hVec_m(h)).^3;
end
specificExcessPower_ms = @(h_m, Prop_W) (Prop_W - interp1(hVec_m, PDrag_W, h_m))/(optim.MGTOW_kg * optim.constants.g_ms2);
h_m   = linspace(0, optim.mission.climb.minh_m, 20);
    
if ~isfield(optim.mission.climb, 'TSLreq_N')
    
    % Compute required propulsive energy required for climb
    tau_s = @(Prop_W) trapz(h_m, 1./specificExcessPower_ms(h_m, Prop_W));
    dtau_s = @(Prop_W) tau_s(Prop_W) - climbTime_s;
    PpropVec_W = linspace(8e3, 1e3, 10);
    for p = 1:length(PpropVec_W)
        tauError_s(p) = dtau_s(PpropVec_W(p));
        if tauError_s(p) > 0
            PpropVec_W = PpropVec_W(1:p);
            Prop_W     = interp1f(tauError_s, PpropVec_W, 0);
            break
        end
    end

    % Gather first guess at sea level thrust requirement to satisfy climb
    % requirement
    PpropMax_W = Prop_W;
    TSLreq_N = PpropMax_W / V_ms(0);
    TSLreq_N = [1.2 .6] * TSLreq_N;
else
    TSLreq_N = optim.mission.climb.TSLreq_N;
    TSLreq_N = [1.05 .95] * TSLreq_N;
end

%% Close the loop on climb mission

% Estimate time to climb using the resulting power from using [1 .85] of 
% the SL thrust estimate. Evaluate from linear interpolation thrust
% requirement and loop back for a third estimate. A final interpolation is
% carried to compute the actual SL thrust requirement.
for t = 1:3

    if t == 3 % use two previous data points to inform the last one
        TSLreq_N(t) = interp1(tClimb_s, TSLreq_N(1:2), climbTime_s, 'linear', 'extrap');
    end
        
    % Span altitudes and find maximum thrust @ constant torque and power
    for h = 1:length(h_m)

        rho_kgm3 = GetAtmosphereProperties(h_m(h));
        VTAS_ms  = V_ms(h_m(h));
        T_N = @(rpm, theta) interp2f(pitches_deg*pi/180, lambda, CT, theta*pi/180, VTAS_ms./(rpm*2*R_m)*60)' * rho_kgm3 .* (rpm/60).^2*(2*R_m)^4;
        Q_Nm = @(rpm, theta) interp2f(pitches_deg*pi/180, lambda, CQ, theta*pi/180, VTAS_ms./(rpm*2*R_m)*60)' * rho_kgm3 .* (rpm/60).^2*(2*R_m)^5;
        P_W = @(rpm, theta) Q_Nm(rpm, theta) .* rpm*2*pi/60;

        if h == 1 % Size for SL thrust

            pitch_deg = pitches_deg(max(CT) > 0);
            pitch_deg = min(pitch_deg):1:max(pitch_deg);

            for p = 1:length(pitch_deg)
               
                F  = @(X) T_N(X(1), pitch_deg(p)) - TSLreq_N(t)/sum(optim.propulsion.N);
                
                % Check if the propulsion system can achieve the target
                % thrust within the RPM range
                RPMmax0 = V_ms(h_m(h)) / ((2*R_m)*min(lambda))*60 - 1e3*eps;
                
                if F(RPMmax0) > 0
                    
                    % Initial guess
                    b = (F(RPMmax0) - F(RPMmax0/2)) / (RPMmax0 - RPMmax0/2);
                   	a = F(RPMmax0) - b * RPMmax0;
                    RPMguess = -a/b;
                    
                    RPM0(p) = Newton(F, RPMguess, 'relax', .8, 'Nmax', 100);
                    Q(p) = Q_Nm(RPM0(p), pitch_deg(p));
                else
                    Q(p) = NaN;
                    RPM0(p) = NaN;
            	end
            end
            [Qmax_Nm(t), p] = min(Q);
            RPMQMax(t)      = RPM0(p);
            PMax_W(t)       = RPMmax*2*pi/60.*Qmax_Nm(t);
            X1              = [RPMQMax(t), pitch_deg(p)];
        else

            F = @(X) [P_W(X(1), X(2)) - PMax_W; Q_Nm(X(1), X(2)) - Qmax_Nm];

            if h > 1
                X0 = X1;
            else
                X0 = [RPMQMax, pitch_deg(a)];
            end
            LB = [VTAS_ms/(max(lambda)*(2*R_m))*60; min(pitch_deg)];
            UB = [min(VTAS_ms/(min(lambda)*(2*R_m))*60, RPMmax); max(pitch_deg)];
            options = optimset('Display', 'off');
            X1 = fmincon(@(X) -T_N(X(1), X(2)), X0, [],[],[],[],LB,UB, @(x) mycon(x,Q_Nm,Qmax_Nm(t),P_W, PMax_W(t)), options);

        end

        thrustClimb_N(h)    = T_N(X1(1), X1(2));
        pitchClimb_deg(h)   = X1(2);
        RPMClimb(h)         = X1(1);
        powerClimb_W(h)      = P_W(X1(1), X1(2));
        torqueClimb_Nm(h)    = Q_Nm(X1(1), X1(2));

        % Now that thrust is known, compute actual specific excess power
        hdotClimb_ms(h) = specificExcessPower_ms(h_m(h), thrustClimb_N(h)*VTAS_ms*sum(optim.propulsion.N));
    end
    
    dt_s        = (h_m(2) - h_m(1))./hdotClimb_ms;
    t_s         = [0 cumsum(dt_s)];
    tClimb_s(t) = t_s(end-1);
    hdotSL_ms(t) = hdotClimb_ms(1);
    
    % For SOC check
    totalClimbEnergy_J(t) = trapz(t_s(1:end-1), powerClimb_W)/optim.propulsion.motor.efficiency/optim.propulsion.controller.efficiency;
end

% Interpolate to solve for actual thrust requirement
[tClimb_s, I] = sort(tClimb_s);
TSLreq_N = interp1(tClimb_s, TSLreq_N(I), climbTime_s, 'linear', 'extrap');
optim.propulsion.propeller.Qmax_Nm  = interp1(tClimb_s, Qmax_Nm(I), climbTime_s, 'linear', 'extrap');
optim.propulsion.totalClimbEnergy_J = interp1(tClimb_s, totalClimbEnergy_J(I), climbTime_s, 'linear', 'extrap') * sum(optim.propulsion.N);
optim.mission.climb.totalClimbEnergy_J = (1 + optim.margin.power_percent) * (optim.propulsion.totalClimbEnergy_J/optim.harness.efficiency.mpptToMotorsEff + ...
                                            + climbTime_s * optim.avionics.powerDraw_W.climb / optim.avionics.powerConversionEfficiency / optim.harness.efficiency.avionicsHarnessEff);
optim.propulsion.propeller.RPMmax   = RPMmax;
optim.mission.climb.TSLreq_N        = TSLreq_N;
optim.mission.climb.DSL_N           = PDrag_W(1)/V_ms(0);
optim.mission.climb.VSL_ms          = V_ms(0);
optim.mission.climb.hdotSL_ms       = interp1(tClimb_s, hdotSL_ms(I), climbTime_s, 'linear', 'extrap');

% Power limit set as the product of max torque and max RPM
optim.propulsion.propeller.PMax_W   = 2 * pi * RPMmax / 60 * optim.propulsion.propeller.Qmax_Nm; % shaft power

%% To accelerate power and torque computation, at known airspeed and thrust
% for performance calculations, inverse CTv = T/(rho V^2 d^2) = f(theta, J)
% into [theta, CTv] -> J. This is repeated for CQv and CPv.

CTv = CT./(lambda*ones(1, size(CT, 2))).^2;
CQv = CQ./(lambda*ones(1, size(CQ, 2))).^2;
CPv = CP./(lambda*ones(1, size(CP, 2))).^3;

CTvs = linspace(min(min(CTv)), max(max(CTv)), 100);
CQvs = linspace(min(min(CQv)), max(max(CQv)), 100);
CPvs = linspace(min(min(CPv)), max(max(CPv)), 100);

JCTv = zeros(length(pitches_deg), length(CTvs));
JCQv = zeros(length(pitches_deg), length(CQvs));
JCPv = zeros(length(pitches_deg), length(CPvs));

for thetat = 1:length(pitches_deg)
    for q = 1:length(CTvs)
        
        % J = f(theta, CTv)
        F = @(j) CTvs(q).*j.^2 - interp2(lambda, pitches_deg, CT', j, pitches_deg(thetat));
        
        if CTvs(q) <= min(CTv(:, thetat)) || CTvs(q) >= max(CTv(:, thetat))
            JCTv(thetat, q) = NaN;
        else
            % Find J values
            JCTv(thetat, q) = Newton(F, interp1(CTv(:, thetat), lambda, CTvs(q)));
        end
        
        % J = f(theta, CQv)
        F = @(j) CQvs(q).*j.^2 - interp2(lambda, pitches_deg, CQ', j, pitches_deg(thetat));
        
        if CQvs(q) <= min(CQv(:, thetat)) || CQvs(q) >= max(CQv(:, thetat))
            JCQv(thetat, q) = NaN;
        else
            % Find J values
            [~, i] = min(abs(CQv(:, thetat)- CQvs(q)));
            JCQv(thetat, q) = Newton(F, lambda(i));
        end
        
        % J = f(theta, CPv)
        F = @(j) CPvs(q).*j.^3 - interp2(lambda, pitches_deg, CP', j, pitches_deg(thetat));
        
        if CPvs(q) <= min(CPv(:, thetat)) || CPvs(q) >= max(CPv(:, thetat))
            JCPv(thetat, q) = NaN;
        else
            % Find the values for J
            [~, i] = min(abs(CPv(:, thetat)- CPvs(q)));
            JCPv(thetat, q) = Newton(F, lambda(i));
        end
    end
end

% Re-discretize pitch angles to insure proper interpolations
pitch_deg = linspace(min(pitches_deg), max(pitches_deg), 100);
JCTv1 = zeros(length(pitch_deg), length(CTvs));
JCQv1 = zeros(length(pitch_deg), length(CTvs));
JCPv1 = zeros(length(pitch_deg), length(CTvs));
for p = 1:length(pitch_deg)
    JCTv1(p, :) = interp2(CTvs, pitches_deg, JCTv, CTvs, pitch_deg(p), 'cubic');
    JCQv1(p, :) = interp2(CQvs, pitches_deg, JCQv, CQvs, pitch_deg(p), 'cubic');
    JCPv1(p, :) = interp2(CPvs, pitches_deg, JCPv, CPvs, pitch_deg(p), 'cubic');
end
optim.propulsion.propeller.invertedMaps.JCTv = JCTv1;
optim.propulsion.propeller.invertedMaps.JCQv = JCQv1;
optim.propulsion.propeller.invertedMaps.JCPv = JCPv1;
optim.propulsion.propeller.invertedMaps.CTv  = CTvs;
optim.propulsion.propeller.invertedMaps.CQv  = CQvs;
optim.propulsion.propeller.invertedMaps.CPv  = CPvs;
optim.propulsion.propeller.invertedMaps.pitch_deg = pitch_deg;

% Clean up propulsion
delete([ optim.propDir filesep 'xrotorlog']);
delete([ optim.propDir filesep 'propMap.dat']);
delete([ optim.propDir filesep 'AQ3.prop']);

end

function [CT, CP, CQ, lambda, RPM, BetaTip_deg] = CharacterizePropeller(filename, deltaPitch_deg, d_m)

% Run XROTOR
commandSeq = 'plop \n';
commandSeq = [commandSeq 'G F \n \n'];
commandSeq = [commandSeq 'load ' filename ' \n30\n'];
commandSeq = [commandSeq 'oper\n'];
commandSeq = [commandSeq 'angl ' num2str(deltaPitch_deg) '\n'];
commandSeq = [commandSeq  'aseq \n.1 \n.2 \n.01 \n '];
commandSeq = [commandSeq  'aseq \n.205 \n1 \n.05 \n '];
commandSeq = [commandSeq  'cput\npropMap.dat \n Y \n \nquit \n'];
Xrotor(commandSeq);

% Parse output
[CT, CP, CQ, lambda, RPM, BetaTip_deg] = ParseXROTOROutput('propMap.dat', d_m);
end


function [CT, CP, CQ, lambda, RPM, BetaTip_deg] = ParseXROTOROutput(filename, d_m)

startRow = 4;
delimiter = ' ';
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

% Open the text file.
fileID = fopen(filename,'r');

% Read columns of data according to the format.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'TextType', 'string', 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

% Close the text file.
fclose(fileID);

% Create output variables
lambda = dataArray{2}*pi;
rho_kgm3 = dataArray{6};
BetaTip_deg = dataArray{3};
RPM = dataArray{5};
P_kW = dataArray{10};
T_N = dataArray{11};
Q_Nm = dataArray{12};

n   = RPM/60;
K   = rho_kgm3 .* n.^2*d_m^4;
CT  = T_N./K;
CQ  = Q_Nm./(K.*d_m);
CP  = P_kW*1e3./(K.*d_m.*n);
end

function [c,ceq] = mycon(x,Q_Nm,Qmax_Nm,P_W, Pmax_W)
    c(1) = Q_Nm(x(1), x(2)) - Qmax_Nm;
    c(2) = P_W(x(1), x(2)) - Pmax_W;
    ceq = [];
end

function Re = GetRe(h_m, V_ms, c_m)

[rho_kgm3, mu] = GetAtmosphereProperties(h_m);
Re = c_m * V_ms/(mu/rho_kgm3);

end