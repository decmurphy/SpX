set key off
set xlabel "Time (s)"
set ylabel "Altitude (km)"
set title "OG2 Re-entry/Landing burns"
p "Stage1.dat" u 1:4 w l, \
	"Points.dat" u 1:4 pt 7 notitle
pause -1
