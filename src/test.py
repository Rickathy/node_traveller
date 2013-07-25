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
    e = edge_interpreter()
    times = e.interpret(t.path_times_list[edge],3)
    print times
    t.graph_edge(edge)
    t.graph_range(times[0],times[1],True)
    



if __name__ == '__main__':
    main(sys.argv)