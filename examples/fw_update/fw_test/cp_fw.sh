#! /bin/sh

#CCSVX=/home/amargan/workspace_v6
CCSVX=/home/amargan/workspace_v6tov7

#cp $CCSVX/soes_test_dc_m3/Flash-Standalone/soes_test_dc_m3.bin soes_m3.bin
cp $CCSVX/soes_test_dc_m3/Flash-AllCores/soes_test_dc_m3.bin soes_m3.bin
cp $CCSVX/soes_test_dc_c28/Flash/soes_test_dc_c28.bin soes_c28.bin

cp $CCSVX/centauro_AC_m3/Flash-AllCores/cent_AC_m3.bin .
#cp $CCSVX/centauro_AC_m3/Flash-AllCores_Release/cent_AC_m3.bin .
cp $CCSVX/centauro_AC_c28/Flash/cent_AC_c28.bin .

cp $CCSVX/f28m36_power_board_m3/Flash-Standalone/pow_m3.bin .
cp $CCSVX/f28m36_power_board_c28/Flash/pow_c28.bin .

cp $CCSVX/Walk_Man_Joint_AC/Release/BigMotor.bin .
if [ ! -e "MedMotor.bin" ]; then
  ln -s BigMotor.bin MedMotor.bin
fi
