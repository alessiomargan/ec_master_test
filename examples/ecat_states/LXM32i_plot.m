# ts, pRef, pAct, pDiff, tqAct

load logss/LXM32iESC_pos_1_log.txt
load logss/LXM32iESC_pos_2_log.txt
load logss/LXM32iESC_pos_3_log.txt

ts = LXM32iESC_pos_1_log(:,1);
pRef = LXM32iESC_pos_1_log(:,2);
pAct = LXM32iESC_pos_1_log(:,3);
pDiff = LXM32iESC_pos_1_log(:,4);

figure(1);
plot(ts,pRef,"+r",ts,pAct,"ob"); grid on;
