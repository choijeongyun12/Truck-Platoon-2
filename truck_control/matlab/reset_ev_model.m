function state = reset_ev_model(state, initial_soc)
%RESET_EV_MODEL Reset runtime accumulators while keeping structure.
if nargin < 2
    initial_soc = 100.0;
end
state.soc = initial_soc;
state.energy_wh = 0.0;
state.distance_m = 0.0;
state.wh_per_km = 0.0;
state.battery_power_w = 0.0;
end
