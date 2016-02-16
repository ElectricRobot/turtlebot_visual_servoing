 # -----------------------------------#
 # Global settings                    #
 # -----------------------------------#

 set grid
 set key left
 set autoscale
 set style data lines 
 set key on inside right top
 set terminal jpeg
 set output "/home/matheus/Pictures/graph_20160215_1743.jpg"


 debut  = 10 # first sample line
 fin    = 365 # last salpe line
 deltaT = 1   # time step for figure related to time
 T0     = 681198 # timestamp at the beginning of the expo

 # -----------------------------------#
 # Graph                              #
 # -----------------------------------#
 set title "Comparison Vx(m/s)"
 #set xtics 0,deltaT,8000
 #set ytics 0,0.5,5
 set grid ytics lc rgb "#bbbbbb" lw 1 lt 0
 set grid xtics lc rgb "#bbbbbb" lw 1 lt 0
 plot  "20160215_1743leader.txt" every 8 using 3 title 'Turtle 1',\
       "20160215_ 1743result.txt" every 1 using 3 title 'Turtle 2'

