% load FRF_data;
FRF_data = [-0.1754 - 0.0556i ...
            -0.0722 - 0.3314i ...
             0.0913 - 0.4007i ...
             0.1823 - 0.2730i ...
             0.2285 - 0.2614i ... 
             0.4253 - 0.1972i ...
             0.4666 - 0.2085i ...
             0.6187 - 0.1176i ...
             0.4481 - 0.2008i];

FRF_data = flip(FRF_data);
% Set the max iteration
maxiter = 5;

% Set number of parameters
para_n = 3;

% Set a set of random values for inital condition
% rand_set = [1.5 * rand(maxiter,1) 10 * rand(maxiter,1) 0.0004* rand(maxiter,1) 0.0004*rand(maxiter,1) 500 * rand(maxiter,1)];
rand_set = [0.0007 * rand(maxiter,1) 0.0007*rand(maxiter,1) 1000 * rand(maxiter,1)];

% Initialize the 
fval_Set = zeros(maxiter,1);
Optimal_para_Set = zeros(maxiter,para_n);

for i = 1:maxiter


% tf_num = All_para(1);
% tf_pole = All_para(2);
% delay = All_para(3);

% Set initial conditions for each model parameter:
All_para_0 = rand_set(i,:);

lower_bound = zeros(1,length(All_para_0));
higher_bound = [1e6 1e6 3600];


fun = @(All_para) MSE_error(All_para,FRF_data);
options = optimoptions('fmincon','MaxIterations',1e10,'MaxFunctionEvaluations',1e5,'ConstraintTolerance',1e-8);
[Optimal_para_temp,fval_temp] = fmincon(fun,All_para_0,[],[],[],[],lower_bound,higher_bound,[],options);
fval_Set(i) = fval_temp;
Optimal_para_Set(i,:) = Optimal_para_temp;
end

[fval_min,index] = min(fval_Set);
%%
Optimal_para = Optimal_para_Set(index,:);

tf_num = Optimal_para(1);
tf_pole = Optimal_para(2);
delay = Optimal_para(3);
%% 
FRF_sim = sim_system(tf_num, tf_pole, delay);

 % make figure showing calculated frf
    f_all = 1 ./(3600 * [30 20 13 8 4 3 2 1.5 1]);

    figure,
    h2 = axes('position',[0.1 0.55 0.88 0.4]);
    hold on
    grid on
    semilogx(f_all, abs((FRF_data)), 'b','LineWidth',2,'MarkerSize', 15)
    semilogx(f_all, abs(FRF_sim), 'c--','LineWidth',2)
    set(h2,'xScale','log');
    set(h2,'yScale','log');
    set(h2,'box','on');
    % xlim([1e-5 1e-4]);
    ylabel('Gain')
    title('Root tip frequency response (G(s))');


    h1 = axes('position',[0.1 0.1 0.88 0.4]);
    hold on
    grid on    
    semilogx(f_all, rad2deg(unwrap(angle((FRF_data)))),'b','LineWidth',2)
    semilogx(f_all, rad2deg(unwrap(angle(FRF_sim))),'c--','LineWidth',2,'MarkerSize', 15)
    set(h1,'xScale','log');
    set(h1,'box','on');
    % xlim([1e-5 1e-4]);
    xlabel('Frequency (Hz)')
    ylabel('Phase (deg)')
