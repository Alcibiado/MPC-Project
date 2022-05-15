reset

set multiplot layout 3,3


set xlabel "time"
set ylabel "car_x"
plot "nextxout.txt" w lp title"cakao_x","mpcxout.txt" w lp title "mpc_x"

set xlabel "time"
set ylabel "car_y"
plot "nextyout.txt" w lp title"cakao_y","mpcyout.txt" w lp title "mpc_y"

set xlabel "time"
set ylabel "psi"
plot "psiout.txt" u 1:2 w lp pt 5 title "mpc_psi","upsiout.txt" u 1:2 w lp pt 7 title "re_psi"
unset multiplot

