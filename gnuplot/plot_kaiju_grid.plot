reset



field = 'COSMOS'
field = 'XMM-LSS'

infilename = "~/scratch/kaiju/kaiju_".field.".log"

infile_targets = "< gump ~/scratch/kaiju/kaiju_grid_out.fits+2 all yes dhigh"

infile_robots0 = "< gawk '$1!~/^#/ && $5~/BA|BOSS/ {print $0}' ~/SDSSV/gitwork/kaiju/etc/fps_filledHex.txt"
infile_robots1 = "< gawk '$1~/^ROBOTSTATS/ && $7 == 0 {print $0}' ".infilename
infile_robots2 = "< gawk '$1~/^ROBOTSTATS/ && $13 == 0 {print $0}' ".infilename
infile_robots3 = "< gawk '$1~/^ROBOTSTATS/ && $19 == 0 {print $0}' ".infilename
infile_robots4 = "< gawk '$1~/^ROBOTSTATS/ && $19 > 0 {print $0}' ".infilename
infile_robots5 = "< gawk '$1~/^ROBOTSTATS/ &&  $7 == 0 || $13 == 0 || $19 == 0 {print $0}' ".infilename

######infile_standards = "< ~/SDSSV/BHM/targetdb/ebosstarget-v0005-std_COSMOS.fits 'ra dec'"
set terminal pngcairo size 1200,1200 font ",16"
set out "~/scratch/kaiju/kaiju_grid.png"

radius2= 22.6
radius1= 7.6

diam1 = radius1*2.
diam2 = radius2*2.

set xlabel "X [mm]"
set ylabel "Y [mm]"
set mxtics 5
set mytics 5
plot \
     infile_targets using 2 : 3 with points pt 1 ps 1.0 lc 9 t "targets",\
     infile_robots0 using 3 : 4 with points pt 2 ps 1.0 lc 2  t "",\
     infile_robots1 using 4 : 5 with points pt 7 ps 1.0 lc 1  t "No targets within annulus",\
     infile_robots2 using 4 : 5 with points pt 7 ps 1.2 lc -1 t "No unassigned targets within annulus",\
     infile_robots2 using 4 : 5 : (diam1) : (diam1) with ellipses lc -1 lw 0.8  t "",\
     infile_robots2 using 4 : 5 : (diam2) : (diam2) with ellipses lc -1 lw 0.8  t "",\
     infile_robots3 using 4 : 5 with points pt 7 ps 1.5 lc  3 t "All targets in annulus are collided",\
     infile_robots3 using 4 : 5 : (diam1) : (diam1) with ellipses lc 3 lw 0.8  t "",\
     infile_robots3 using 4 : 5 : (diam2) : (diam2) with ellipses lc 3 lw 0.8  t "",\
     infile_robots4 using 4 : 5 with points pt 7 ps 1.0 lc  4 t "good"


set out