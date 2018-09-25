import pstats
p = pstats.Stats("pyProfile.txt")
p.strip_dirs().sort_stats("cumulative").print_stats(50)
