
load /tmp/Ft6ESC_81_log.txt

Ft6_log = Ft6ESC_81_log;

t =  Ft6_log(:,1);
fx = Ft6_log(:,2);
fy = Ft6_log(:,3);
fz = Ft6_log(:,4);
tx = Ft6_log(:,5);
ty = Ft6_log(:,6);
tz = Ft6_log(:,7);

figure(1);
plot(t,fx,t,fy,t,fz); grid on;
figure(2);
plot(t,tx,t,ty,t,tz); grid on;
