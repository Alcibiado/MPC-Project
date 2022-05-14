reset
#set terminal pngcairo size 1000,1000 font 'Times New Roman,10'
#set output "Mpcplot.png"

#sx = "set xrange"

#left=0.1
#right=0.95
#bottom=0.1
#top="0.95"
#hspace = "0.1"
#wspace = "0.15"
#set multiplot layout 2,2 spacing @hspace,@wspace margins left,right,bottom,@top

#set label"(1)"at graph 0.02,0.03 font ',20' textcolor rgb 'red'
set title "mpc" 
set xlabel "mpc_x"
set ylabel "mpc_y"
set key top right Left reverse font 'Time New Roman,15'
set xrange[0:30]
set yrange[-5:10]
set xtics scale 3
set mxtics 10
set mytics 5
#set log x
#set style fill pattern 1

plot "output.txt" with linespoints # 2 

