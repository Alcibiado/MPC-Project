reset
set xlabel "时间步"
set ylabel "横摆角"
plot "psiout.txt" u 1:2 w lp pt title "预测横摆角",\
"upsiout.txt" u 1:2 w lp pt title "实际横摆角"

