# gravitropism-controls-optimization

or our 1st order model with time delay:

Run the matlab file Optimal_Parameter.m. This uses the function MSE_error.m to minimize the error between generated and experimental gain and phase values. The simulated gains and phases are generated using the created function sim_system.m to run simulink file Simulation_nodeadzone2.slx

For our 1st order model with no delay:

Similar architecture. Run fule Optimal_Parameter_nodelay.m. This uses MSE_error_nodelay.m and sim_system_nodelay.m to interface with Simulation_nodeadzone_nodelay.slx.

For our 2nd order model with no delay:

Again, similar. Run file Optimal_Parameter_nodelay_secondorder.m. It uses MSE_error_nodelay_2ndorder.m and sim_sytem_nodelay_secondorder.m to also interface with Simulation_nodeadzone_nodelay.slx (the sim_system function just passes in more parameters to fit). 
