#!/usr/bin/env python

import sys
from travel import travel
from edge_interpreter import edge_interpreter


def main(args):
    edge=5
    t= travel()
    t.set_up_for_test()
    t.load_path_times()
    t.current=0
    t.graph_path_times(False)
    t.graph_edge(edge)
    e = edge_interpreter()
    times = e.interpret(t.path_times_list[edge],4)
    t.graph_range(times[0],times[1],'exponential method',True)
    
    times = e.interpret(t.path_times_list[edge],3)
    t.graph_range(times[0],times[1],'clustering method',True)
    #t.graph_range(times[0],times[2],True)
    



if __name__ == '__main__':
    main(sys.argv)