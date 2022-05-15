reset
set multiplot layout 3,1
set xlabel "time"
set ylabel "carCte_5"
plot "5ctere_cte.txt" w lp title"5ctere_cte","5ure_cte.txt" w lp title"5ure_cte",\
"5wre_cte.txt" w lp title"5wre_cte","stre_cte.txt" w lp title"stre_cte"

set xlabel "time"
set ylabel "carCte_0"
plot "0ctere_cte.txt" w lp title"0ctere_cte","001ctere_cte.txt" w lp title"001ctere_cte",\
"0ure_cte.txt" w lp title"0ure_cte","0wre_cte.txt" w lp title"0wre_cte","stre_cte.txt" w lp title"stre_cte"

set xlabel "time"
set ylabel "carepsi_5"
plot "5ctere_epsi.txt" w lp title"5ctere_epsi","5ure_epsi.txt" w lp title"5ure_epsi",\
"5wre_epsi.txt" w lp title"5wre_epsi","stre_epsi.txt" w lp title"stre_epsi"

set xlabel "time"
set ylabel "carepsi_0"
plot "0ctere_epsi.txt" w lp title"0ctere_epsi","001ctere_epsi.txt" w lp title"001ctere_epsi",\
"0ure_epsi.txt" w lp title"0ure_epsi","0wre_epsi.txt" w lp title"0wre_epsi","stre_epsi.txt" w lp title"stre_epsi"

set xlabel "time"
set ylabel "carespeedout5"
plot "5ctespeedout.txt" w lp title"5ctespeedout","5uspeedout.txt" w lp title"5uspeedout",\
"5wspeedout.txt" w lp title"5wspeedout","stspeedout.txt" w lp title"stspeedout"

set xlabel "time"
set ylabel "carespeedout0"
plot "0ctespeedout.txt" w lp title"0ctespeedout","001ctespeedout.txt" w lp title"001ctespeedout",\
"0uspeedout.txt" w lp title"0uspeedout","0wspeedout.txt" w lp title"0wspeedout","stspeedout.txt" w lp title"stspeedout"

set xlabel "time"
set ylabel "carexyout5"
plot "5ctexyout.txt" w lp title"5ctexyout","5uxyout.txt" w lp title"5uxyout",\
"5wxyout.txt" w lp title"5wxyout","stxyout.txt" w lp title"stxyout"

set xlabel "time"
set ylabel "carexyout0"
plot "0ctexyout.txt" w lp title"0ctexyout","001ctexyout.txt" w lp title"001ctexyout",\
"0uxyout.txt" w lp title"0uxyout","0wxyout.txt" w lp title"0wxyout","stxyout.txt" w lp title"stxyout"

unset multiplot