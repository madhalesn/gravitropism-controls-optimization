function e = MSE_error(All_para,FRF_data)


tf_num = All_para(1);
tf_pole = All_para(2);
delay = All_para(3);

periods=[30 20 13 8 4 3 2 1.5 1];
w = 2 * pi ./(periods * 60 * 60);
[FRF_sim] = sim_system(tf_num, tf_pole, delay);
%e = 1/(length(periods)) * (norm(1e15 * w .*(FRF_data - FRF_sim),'fro'))^2;
e = 1/(length(periods)) * (norm(1e8 * w .*(FRF_data - FRF_sim),'fro'))^2;

end