reset
set multiplot layout 3,2
set xlabel "time"
set label "p1"
set ylabel "car_psi"
plot "dt005psiout.txt" w lp title"dt005psi","dt008psiout.txt" w lp title"dt008psi",\
"dt010psiout.txt" w lp title"dt010psi","dt015psiout.txt" w lp title"dt015psi","dt020psiout.txt" w lp title"dt020psi",\
"dt030psiout.txt" w lp title"dt030psi"

set xlabel "time"
set ylabel "car_cte"
plot "dt005re_cte.txt" w lp title"dt005re_cte","dt008re_cte.txt" w lp title"dt008re_cte",\
"dt010re_cte.txt" w lp title"dt010re_cte","dt015re_cte.txt" w lp title"dt015re_cte","dt020re_cte.txt" w lp title"dt020re_cte",\
"dt030re_cte.txt" w lp title"dt030re_cte"

set xlabel "time"
set ylabel "car_epsi"
plot "dt005re_epsi.txt" w lp title"dt005re_epsi","dt008re_epsi.txt" w lp title"dt008re_epsi",\
"dt010re_epsi.txt" w lp title"dt010re_epsi","dt015re_epsi.txt" w lp title"dt015re_epsi","dt020re_epsi.txt" w lp title"dt020re_epsi",\
"dt030re_epsi.txt" w lp title"dt030re_epsi"

set xlabel "time"
set ylabel "car_speed"
plot "dt005speedout.txt" w lp title"dt005speed","dt008speedout.txt" w lp title"dt008speed",\
"dt010speedout.txt" w lp title"dt010speed","dt015speedout.txt" w lp title"dt015speed","dt020speedout.txt" w lp title"dt020speed",\
"dt030speedout.txt" w lp title"dt030speed"

set xlabel "car_x"
set ylabel "car_y"
plot "dt005xyout.txt" w lp title"dt005","dt008xyout.txt" w lp title"dt008",\
"dt010xyout.txt" w lp title"dt010","dt015xyout.txt" w lp title"dt015","dt020xyout.txt" w lp title"dt020",\
"dt030xyout.txt" w lp title"dt030"

unset multiplot