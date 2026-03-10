function [state, out] = step_ev_model(state, in, params)
%STEP_EV_MODEL Update SOC/energy from one simulation step.
% in fields: v_ms, a_ms2, pitch_deg, dt
% params fields: mass_kg, c_rr, cd_a, rho_air, eta_drive, eta_regen, p_aux_w, battery_capacity_kwh

p_batt = eplatoon_power_step( ...
    in.v_ms, in.a_ms2, in.pitch_deg, ...
    params.mass_kg, params.c_rr, params.cd_a, params.rho_air, ...
    params.eta_drive, params.eta_regen, params.p_aux_w);

state.battery_power_w = p_batt;
delta_wh = max(p_batt, 0.0) * (in.dt / 3600.0);
state.energy_wh = state.energy_wh + delta_wh;
state.distance_m = state.distance_m + max(in.v_ms, 0.0) * in.dt;

if params.battery_capacity_kwh > 0
    delta_soc = (delta_wh / 1000.0) / params.battery_capacity_kwh * 100.0;
    state.soc = max(0.0, min(100.0, state.soc - delta_soc));
end

if state.distance_m > 1.0
    state.wh_per_km = state.energy_wh / (state.distance_m / 1000.0);
else
    state.wh_per_km = 0.0;
end

out = state;
end
