reset
set multiplot layout 3,3
plot "mpcxout.txt" 
plot "mpcyout.txt"
plot "nextxout.txt"
plot "nextyout.txt"
plot "msaout.txt"
plot "mtout.txt"
plot "ptsxout.txt"
plot "ptsyout.txt"
plot "upsiout.txt"
plot "psiout.txt"
plot "xout.txt"
plot "yout.txt"
plot "saout.txt"
plot "tout.txt"
plot "speedout.txt"
plot "mpcout.txt"
plot "nextout.txt"
plot "ptsout.txt"
#plot "throttleop.txt"
set xrange [0:1000]
set yrange  [0:200]

#plot  "speedout.txt" 
unset multiplot

