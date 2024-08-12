% clear
% clc

load("FlightData.mat")

%save('myfile.mat','-v7.3')

theta_a = atan2(-Acc_x.signals.values,sqrt(Acc_y.signals.values.^2+Acc_z.signals.values.^2));

phi_a = atan2(Acc_y.signals.values,Acc_z.signals.values);

gamma = 0.1;

h = 0.01;

alpha = (gamma * h) / (1 - gamma); 


theta_k = zeros(length(Acc_x.time),1);

theta_k(1) = theta_a(1);

for i = 2:length(Acc_x.time)
    theta_k(i) = (1-gamma)*(theta_a(i)) + gamma*(theta_k(i-1)+h*Gyro_y.signals.values(i));
end

phi_k = zeros(length(Acc_x.time),1);

phi_k(1) = phi_a(1);

for i = 2:length(Acc_x.time)
    phi_k(i) = (1-gamma)*(phi_a(i)) + gamma*(phi_k(i-1)+h*Gyro_x.signals.values(i));
end


plot(theta_k)
hold on
plot(phi_k)
legend('theta (pitch)','phi (roll)')
% %save('myfile.mat','-v7.3')
% 
% theta_a = atan2(-Acc_x.signals.values,sqrt(Acc_y.signals.values.^2+Acc_z.signals.values.^2));
% 
% phi_a = atan2(Acc_y.signals.values,Acc_z.signals.values);
% 
% gamma = 0.001;
% 
% h = 0.01;
% 
% theta_k = zeros(length(Acc_x.time),1);
% 
% theta_k(1) = theta_a(1);
% 
% for i = 2:length(Acc_x.time)
%     theta_k(i) = (1-gamma)*(theta_a(i)) + gamma*(theta_k(i-1)+h*Gyro_y.signals.values(i));
% end
% 
% plot(theta_k)
crazyflie
