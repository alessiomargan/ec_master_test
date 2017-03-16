#! /usr/bin/env gnuplot

plot "LpESC_1_log.txt" using 1:2 title 'link_pos',\
"LpESC_1_log.txt" using 1:3 title 'motor_pos',\
"LpESC_1_log.txt" using 1:4 title 'pos_fb'


pause -1 "Hit any key to continue"
