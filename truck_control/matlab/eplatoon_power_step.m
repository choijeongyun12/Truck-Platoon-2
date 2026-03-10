function p_batt_w = eplatoon_power_step(v_ms, a_ms2, pitch_deg, mass_kg, c_rr, cd_a, rho_air, eta_drive, eta_regen, p_aux_w)
%EPLATOON_POWER_STEP Single-step battery power estimate used by ROS2 bridge.
g = 9.81;
pitch_rad = deg2rad(pitch_deg);

f_grade = mass_kg * g * sin(pitch_rad);
f_roll = mass_kg * g * c_rr;
f_aero = 0.5 * rho_air * cd_a * v_ms^2;
f_inertial = mass_kg * a_ms2;
f_total = f_grade + f_roll + f_aero + f_inertial;

p_wheel = f_total * v_ms;
if p_wheel >= 0
    p_batt_w = p_wheel / max(eta_drive, 0.05) + p_aux_w;
else
    p_batt_w = p_wheel * min(max(eta_regen, 0.0), 1.0) + p_aux_w;
end
end
