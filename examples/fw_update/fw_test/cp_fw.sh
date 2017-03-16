#! /bin/sh

#cp /home/amargan/workspace_v6/soes_test_dc_m3/Flash-Standalone/soes_test_dc_m3.bin soes_m3.bin
#cp /home/amargan/workspace_v6/soes_test_dc_m3/Flash-AllCores/soes_test_dc_m3.bin soes_m3.bin
#cp /home/amargan/workspace_v6/soes_test_dc_c28/Flash/soes_test_dc_c28.bin soes_c28.bin

cp /home/amargan/workspace_v6/centauro_AC_m3/Flash-AllCores/cent_AC_m3.bin .
#cp /home/amargan/workspace_v6/centauro_AC_m3/Flash-AllCores_Release/cent_AC_m3.bin .
cp /home/amargan/workspace_v6/centauro_AC_c28/Flash/cent_AC_c28.bin .

cp /home/amargan/workspace_v6/Walk_Man_Joint_AC/Release/BigMotor.bin .
if [ ! -e "MedMotor.bin" ]; then
  ln -s BigMotor.bin MedMotor.bin
fi
