
load logs/Ft6ESC_34_log.txt
load logs/Ft6ESC_35_log.txt

x = Ft6ESC_34_log(:,1);
right_foot_f = [Ft6ESC_34_log(:,2),Ft6ESC_34_log(:,3),Ft6ESC_34_log(:,4)];
left_foot_f = [Ft6ESC_35_log(:,2),Ft6ESC_35_log(:,3),Ft6ESC_35_log(:,4)];
right_foot_tz = Ft6ESC_34_log(:,7);
left_foot_tz = Ft6ESC_35_log(:,7);

figure(1);
plot(x,right_foot_f(:,3),x,left_foot_f(:,3)); grid on;
figure(2);
plot(x,right_foot_tz,x,left_foot_tz); grid on;
figure(3);
plot(x,Ft6ESC_34_log(:,9),x,Ft6ESC_35_log(:,9)); grid on;
