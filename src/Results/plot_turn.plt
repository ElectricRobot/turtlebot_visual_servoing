 # -----------------------------------#
 # Global settings                    #
 # -----------------------------------#

 set grid
 set key left
 set autoscale
 set style data lines 
 set key on inside right top


 debut  = 10 # first sample line
 fin    = 200 # last salpe line
 deltaT = 1   # time step for figure related to time
 T0     = 718022 # timestamp at the beginning of the expo

 # -----------------------------------#
 # Error Norm                         #
 # -----------------------------------#
 set title "Error Norm"
 set xtics 0,deltaT,8000
 set ytics 0,0.5,5
 plot  "turtle_turn_ok_160205_ex3.txt" every ::debut::fin using ($2-T0):3 title 'Angle',\
       "turtle_turn_ok_160205_ex3.txt" every ::debut::fin using ($2-T0):4 title 'Commande'
 set terminal postscript eps color lw 3 "Helvetica" 20
 set out "/home/matheus/Pictures/graph1.ps"
 replot
 set term pop
