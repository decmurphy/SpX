set key off
set xlabel "Velocity (m/s)"
set ylabel "Altitude (km)"
p "../Stage1_launch.dat" u 5:4 w l, "../Stage1_retro.dat" u 5:4 w l, \
	"../Stage1_Points.dat" u 5:4 pt 7 notitle, "" u 5:4:8 w labels offset 1,1 notitle
pause -1
