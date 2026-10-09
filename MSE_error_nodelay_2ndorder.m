function e = MSE_error_nodelay_2ndorder(All_para,FRF_data)


tf_num = All_para(1);
tf_pole1 = All_para(2);
tf_pole2 = All_para(3);

periods=[30 20 13 8 4 3 2 1.5 1];
w = 2 * pi ./(periods * 60 * 60);
[FRF_sim] = sim_system_nodelay_2ndorder(tf_num, tf_pole1,tf_pole2);
%e = 1/(length(periods)) * (norm(1e15 * w .*(FRF_data - FRF_sim),'fro'))^2;
e = 1/(length(periods)) * (norm(1e8 * w .*(FRF_data - FRF_sim),'fro'))^2;

end