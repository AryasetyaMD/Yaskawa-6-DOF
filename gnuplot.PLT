set terminal png large size 1600,900
cd 'C:\2023\project_esp_vscode\YaskawaRobot' # ubah sesuai letak file
RTD=180/3.14
set grid 

set xlabel "waktu [s]"
set title "Joint"
set ylabel "Derajat"
set output "remote_joint.png"
set autoscale
#set xrange [1:20] 
plot "logremote.csv" usi 1:($3*RTD) ti "J1cmd" w l, "" usi 1:($4*RTD) ti "J1" w l, "" usi 1:($5*RTD) ti "J2cmd" w l, "" usi 1:($6*RTD) ti "J2" w l

set xlabel "X"
set title "EndEffector"
set ylabel "Y"
set output "remote_xy.png"
set autoscale
#set xrange [1:20] 
plot "logremote.csv" usi 7:8 ti "Endeffector" w l

set xlabel "sampling - k"
set title "Joint"
set ylabel "Derajat"
set output "local_joint.png"
set autoscale
#set xrange [1:20] 
plot "loglocal.csv" usi 1:($2*RTD) ti "J1" w l, "" usi 1:($3*RTD) ti "J2" w l

set xlabel "X"
set title "EndEffector"
set ylabel "Y"
set output "local_xy.png"
set autoscale
#set xrange [1:20] 
plot "loglocal.csv" usi 4:5 ti "Endeffector" w l

set output "dummy.png“ # agar gambar yang sebelumnya sudah dibuat bisa di buka