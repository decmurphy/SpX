set key off
set xrange[-100:2300]
set yrange[-1000:1000]
set xlabel "Downrange (km)"
set ylabel "Altitude (km)"
p "../Stage1.dat" u 2:3 w l, "../Stage2.dat" u 2:3 w l, \
	"../Stage1_Points.dat" u 2:3 pt 7 notitle, "" u 2:3:8 w labels offset 3,0 notitle, \
	"../Stage2_Points.dat" u 2:3 pt 7 notitle, "" u 2:3:8 w labels offset 3,0 notitle, \
	"../earth.dat" u 1:2 w l
pause -1
