function [FRF_sim] = sim_system(tf_num, tf_pole, delay )
    %set parameters for system

    %transfer function    
    tf_string= sprintf('tf([%g],[1  %d])', tf_num, tf_pole);
    set_param('Simulation_nodeadzone2/LTI System', 'sys', tf_string)

% %     nodeadzone
% %     set_param('Simulation_nodeadzone2/Constant1', 'Value', num2str(nodeadzone_slope))
% %     set_param('Simulation_nodeadzone2/Constant', 'Value', num2str(nodeadzone_thresh))

    %delay
       set_param('Simulation_nodeadzone2/Transport Delay', 'DelayTime', sprintf('%.15g', delay))


    % Initialize FRF
    FRF_sim=[];

    %stimulus periods for simulation
    periods=[30 20 13 8 4 3 2 1.5 1];

    %sample time
    delta_t = 2.5 * 60;

    for i=1:length(periods)
        period=periods(i);
        %calculate reference input and set simulation stop time
        if period == 30
            t = 0 : delta_t : 1* period * 60 * 60 - delta_t;
             set_param('Simulation_nodeadzone2', 'StopTime', '30*60*60');
        elseif period == 20
            t = 0 : delta_t : 1* period * 60 * 60 - delta_t;
            set_param('Simulation_nodeadzone2', 'StopTime', '20*60*60');
        elseif period == 13
            t = 0 : delta_t : 2* period * 60 * 60 - delta_t;
            set_param('Simulation_nodeadzone2', 'StopTime', '26*60*60');
        elseif period == 8
            t = 0 : delta_t : 3* period * 60 * 60 - delta_t;
            set_param('Simulation_nodeadzone2', 'StopTime', '24*60*60');
        elseif period == 4
            t = 0 : delta_t : 6* period * 60 * 60 - delta_t;
            set_param('Simulation_nodeadzone2', 'StopTime', '24*60*60');
        elseif period == 3
            t = 0 : delta_t : 8* period * 60 * 60 - delta_t;
            set_param('Simulation_nodeadzone2', 'StopTime', '24*60*60');
        elseif period == 2
            t = 0 : delta_t : 12* period * 60 * 60 - delta_t;
            set_param('Simulation_nodeadzone2', 'StopTime', '24*60*60');
        elseif period == 1.5
            t = 0 : delta_t : 16* period * 60 * 60 - delta_t;
            set_param('Simulation_nodeadzone2', 'StopTime', '24*60*60');
        else
            t = 0 : delta_t : 24* period * 60 * 60 - delta_t;
            set_param('Simulation_nodeadzone2', 'StopTime', '24*60*60');
        end
        w = 2 * pi /(period * 60 * 60);
        r = -40 * sin(w * t);
        r2 = [transpose(t),transpose(r)];

        %define simulation and pass in r2
        simulation = Simulink.SimulationInput('Simulation_nodeadzone2');
        simulation = simulation.setVariable('r2', r2);

        %run simulation
        results=sim(simulation);

        %get results
        y = results.simout.Data(1:end-1);

        %calculate FRF
        delta_f = 1/(delta_t)/length(r);
        f = 0: delta_f:1/(delta_t)-delta_f;
        R = fft(r);
        Y = fft(y);

        sidx = 1/(period * 3600)/delta_f + 1;
        FRF = Y(round(sidx))/(R(round(sidx)));
        FRF_sim = [FRF_sim FRF];

    end
    %make figure showing calculated frf
    f_all = 1 ./(3600 * [30 20 13 8 4 3 2 1.5 1]);
    
%     figure,
%     h2 = axes('position',[0.1 0.55 0.88 0.4]);
%     hold on
%     grid on
%     semilogx(f_all, abs(FRF_sim), 'b','LineWidth',2,'MarkerSize', 15)
%     set(h2,'xScale','log');
%     set(h2,'yScale','log');
%     set(h2,'box','on');
%     % xlim([1e-5 1e-4]);
%     ylabel('Gain')
%     title('Root tip frequency response (G(s))');
%     
%     
%     h1 = axes('position',[0.1 0.1 0.88 0.4]);
%     hold on
%     grid on    
%     semilogx(f_all, rad2deg(unwrap(angle(FRF_sim))),'b','LineWidth',2,'MarkerSize', 15)
%     set(h1,'xScale','log');
%     set(h1,'box','on');
%     % xlim([1e-5 1e-4]);
%     xlabel('Frequency (Hz)')
%     ylabel('Phase (deg)')

    %sample time
delta_t = 2.5 * 60;

t = 0 : delta_t : 20*60*60;
r=90*ones(1, length(t));

r2 = [transpose(t),transpose(r)];
%define simulation and pass in r2
simulation = Simulink.SimulationInput('Simulation_nodeadzone2');
simulation = simulation.setVariable('r2', r2);

%run simulation
results=sim(simulation);
% 
% figure
% plot(results.simout.time/60/60, results.simout.data)
% step_sim=results.simout.data;
% steptime=results.simout.time/60/60;
% 
% hold on
% load singleturn_paper_data.mat
% plot((5/60)*(1:length(data)),-data)
% axis([0 20 0 100])
% ylabel('Root Tip Angle (degrees)')
% xlabel('Time (hrs)')

end