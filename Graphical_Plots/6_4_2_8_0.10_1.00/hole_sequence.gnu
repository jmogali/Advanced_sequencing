reset
set xlabel "Breadth (In meters)"
set ylabel "Diameter (In meters)"

set xrange [-0.5:100]
set yrange [-0.5:2.5]

set term png

set output "Alternating_Graph.png"
plot 'collisions.dat' using 1:2:3:4 with vectors nohead , 'enabling.dat' using 1:2:3:4 with vectors head filled lt 2
