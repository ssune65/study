clear all

%%%%%%%%%% Load Data %%%%%%%%%%
load CYC_FTP.mat
% load CYC_UDDS.mat
% load CYC_NEDC.mat
load EREV_20250825.mat

bat_cap = 75;

%%%%%%%%%% Initlal Setting %%%%%%%%%%
cyc_mph = [cyc_mph; cyc_mph]; % Two drive cycle (UDDS, NEDC)
cyc_length = length(cyc_mph);
g = 9.8;
grade = 0; % Grade
veh_gb_eff = 1; % Gear Efficiency
Ts = 1; % Sampling Time (sec)
a = zeros(cyc_length, 1); % Acceleration
F_wh = zeros(cyc_length, 1); % Wheel Force
P_wh = zeros(cyc_length, 1); % Wheel Power
T_mot = zeros(cyc_length, 1); % Motor Torque
w_mot = zeros(cyc_length, 1); % Motor angular speed
P_d = zeros(cyc_length, 1); % Power demand
SOC = zeros(cyc_length, 1); % Battery SOC
Sg = zeros(cyc_length, 1); % Engine On/Off State
mc_eff = zeros(cyc_length, 1); % motor efficiency
% SOC0 = 0.5; % SOC initial value (Series HEV)
SOC0 = 0.3; % SOC initial value (EREV)
SOC = zeros(cyc_length+1, 1); % SOC
SOC(1) = SOC0; % SOC initialization
% SOC_min = 0.4; % SOC lower limit (Series HEV)
% SOC_max = 0.8; % SOC upper limit (Series HEV)
SOC_min = 0.25; % SOC lower limit (EREV)
SOC_max = 0.35; % SOC upper limit (EREV)
Voc = zeros(cyc_length, 1); % Open Circuit Voltage
Rin = zeros(cyc_length, 1); % Internal Resistance
w_eng = 3250 * (2*pi)/60 * zeros(cyc_length, 1); % Optimal Engine Speed (rad/s)
T_eng = 54*zeros(cyc_length, 1); % Optimal Engine Output Torque (N.m)
P_eng = w_eng .* T_eng * 2 * pi / 60; % Optimal Engine Power (W)
P_eng_in = zeros(cyc_length, 1); % Engine Power input = Fuel Power (W)
P_gen = P_eng * gen_eff; % Optimel Generator Power (kW)
P_egs = zeros(cyc_length, 1); % Engine-Generator set
P_bat = zeros(cyc_length, 1); % Battery Power (W)
I_bat = zeros(cyc_length, 1); % Battery Current (A)

%%%%%%%%%% velocity mph -> m/s %%%%%%%%%%
cyc_kph = [cyc_mph(:,1), cyc_mph(:,2) * 1.6093];
cyc_mps = [cyc_kph(:,1), cyc_kph(:,2) * 1000 / 3600];

%%%%%%%%%% Motor Efficiency Map transformation (Input based -> Output based) %%%%%%%%%%
[mc_spd, mc_trq] = meshgrid(mc_map_spd, mc_map_trq);
mc_trq_out = mc_eff_map .* mc_trq;
F_mc_eff = scatteredInterpolant(mc_spd(:), mc_trq_out(:), mc_eff_map(:), 'natural', 'nearest');

%%%%%%%%%% Engine Efficiency Map %%%%%%%%%%
[eng_spd, eng_trq] = meshgrid(eng_map_spd*(60/(2*pi)), eng_map_trq);
eng_eff_map_tr = eng_eff_map';

%%%%%%%%%% Thermostat Control %%%%%%%%%%
for i = 1:cyc_length
    %%%%% Power Demand %%%%%
    F_R = veh_Cr * veh_mass * g * cos(grade); % Roling resistance force
    F_D = 0.5 * veh_air_den * veh_Cd * veh_FA * cyc_mps(i,2)^2; % Aerodynamic drag force
    F_C = veh_mass * g * sin(grade); % Climbing force

    %%% Keep acceleration value at final step %%%
    if i == cyc_length
        a(i) = a(i-1);
    else
        a(i) = (cyc_mps(i+1,2) - cyc_mps(i,2)) / Ts;
    end

    %%% Motor efficiency map Interpolation %%%
    F_wh(i) = veh_mass*a(i) + F_R + F_D + F_C;
    P_wh(i) = F_wh(i) * cyc_mps(i,2);
    T_mot(i) = (F_wh(i) * veh_wh_rad*0.001) / (veh_gb_rat*veh_gb_eff);
    w_mot(i) = (cyc_mps(i,2) * veh_gb_rat) / (veh_wh_rad*0.001);
    mc_eff(i) = F_mc_eff(w_mot(i), T_mot(i));

    %%% Calculate Power demand %%%
    if P_wh(i) >= 0
        P_d(i) = P_wh(i) / (veh_gb_eff * mc_eff(i));
    else
        P_d(i) = P_wh(i) * veh_gb_eff * mc_eff(i);
    end

    %%%%% Rule %%%%%
    %%% Engine On/Off State %%%
    if i == 1
        Sg(i) = 0;
    else
        if SOC(i) >= SOC_max || (SOC(i) > SOC_min && SOC(i) < SOC_max && Sg(i-1) == 0)
            Sg(i) = 0;
        elseif SOC(i) <= SOC_min || (SOC(i) > SOC_min && SOC(i) < SOC_max && Sg(i-1) == 1)
            Sg(i) = 1;
        else
            Sg(i) = Sg(i-1); % Keep previous state
        end
    end

    %%% Battery Power %%%
    if Sg(i) == 1
        w_eng(i) = 3250;
        T_eng(i) = 54;
        P_eng(i) = w_eng(i) * T_eng(i) * ((2*pi)/60); % Optimal Engine Power (W)
        P_gen(i) = P_eng(i) * gen_eff; % Optimal Generator Power (kW)
        P_egs(i) = P_gen(i);
    else
        w_eng(i) = 0;
        T_eng(i) = 0;
        P_eng(i) = w_eng(i) * T_eng(i) * ((2*pi)/60); % Optimal Engine Power (W)
        P_gen(i) = P_eng(i) * gen_eff; % Optimal Generator Power (kW)
        P_egs(i) = P_gen(i);
    end
    P_bat(i) = P_d(i) - P_egs(i);
    P_eng_in(i) = P_eng(i)/interpn(eng_spd', eng_trq', eng_eff_map_tr', w_eng(i)*60/(2*pi), T_eng(i), 'spline', 0.3);
    % P_eng_in(i) = interpn(eng_map_spd_rpm, eng_map_trq, eng_eff_map_tr, w_eng(i)*60/(2*pi), T_eng(i), 'spline', 0.3);

    %%%%% SOC %%%%%
    %%% Open Circuit Voltage Interpolation %%%
    Voc(i) = interpn(bat_soc, bat_voc, SOC(i), 'spline') * bat_module_num;

    %%% Internal Resistance Interpolation %%%
    if P_bat(i) >= 0 % Discharge
        Rin(i) = interpn(bat_soc, bat_rin_dis, SOC(i), 'spline');
    else % Charge
        Rin(i) = interpn(bat_soc, bat_rin_chg, SOC(i), 'spline');
    end

    %%% Output Current %%%
    I_bat(i) = (Voc(i) - sqrt(Voc(i)^2 - 4*P_bat(i)*Rin(i))) / (2*Rin(i));

    %%% Calculate SOC %%%
    if P_bat(i) >= 0 % Discharge
        SOC(i+1) = SOC(i) - (I_bat(i)/bat_cap)*(Ts/3600);
    else % Charge
        SOC(i+1) = SOC(i) - (I_bat(i)/bat_cap)*(Ts/3600)*bat_coulombic_eff;
    end
    SOC(i+1) = min(max(SOC(i+1), 0), 1); % SOC boundary consideration

end