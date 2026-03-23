function state = init_ev_model(initial_soc)
%INIT_EV_MODEL Initialize state for EV battery estimation loop.
if nargin < 1
    initial_soc = 100.0;
end

state.soc = initial_soc;
state.energy_wh = 0.0;
state.distance_m = 0.0;
state.wh_per_km = 0.0;
state.battery_power_w = 0.0;
end
