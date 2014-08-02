set key off
set xrange[0:2000]
set yrange[0:300]
set xlabel "Velocity (m/s)"
set ylabel "Altitude (km)"
p "Stage1.dat" u 5:4 w l, \
	"Points.dat" u 5:4 pt 7 notitle, "" u 5:4:8 w labels offset 1,1 notitle
pause -1
