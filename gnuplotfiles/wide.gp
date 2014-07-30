set key off
set xrange[-10000:10000]
set xlabel "Downrange (km)"
set ylabel "Altitude (km)"
p "Stage1.dat" u 2:3 w l, "Stage2.dat" u 2:3 w l, \
	"earth.dat" u 1:2 w l
pause -1
