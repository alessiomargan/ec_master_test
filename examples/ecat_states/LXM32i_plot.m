# ts, pRef, pAct, pDiff, tqAct

#load /tmp/LXM32iESC_pos_1_log.txt;
#load /tmp/LXM32iESC_pos_2_log.txt
load /tmp/LXM32iESC_pos_3_log.txt

data = LXM32iESC_pos_3_log;

ts = data(:,1);
pRef = data(:,2);
pAct = data(:,3);
#pDiff = LXM32iESC_pos_1_log(:,4);

figure(1);
plot(ts,pRef,"r;pRef;",ts,pAct,"b;pAct;"); grid on;
