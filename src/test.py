#!/usr/bin/env python

import sys
from travel import travel
from edge_interpreter import edge_interpreter
from datetime import datetime

def main(args):
    #current_time = 10*int((datetime.today().hour*60+datetime.today().minute)/10)
    #print current_time
    edge=16
    t= travel()
    t.set_up_for_test()
    t.load_path_times()
    t.current=0
    route = t.generate_route_entropy_clusters([],570)
    print route
    print len(route)
    #for i in range(len(t.path_times_list)):
     #   print i
      #  print t.path_times_list[i].edge.A.node_num, t.path_times_list[i].edge.B.node_num
    #print t.path_times_list
    #t.graph_path_times(True)
   # ent = t.generate_entropies()
    #t.current=0
    #t.graph_path_times(False)
   # t.graph_path_dates(True)
    #t.graph_edge(edge,True)
    #e = edge_interpreter()
    #times = e.interpret(t.path_times_list[edge],4)
    #t.graph_range(times[0],times[1],'exponential method',True)
    
   # times = e.interpret(t.path_times_list[edge],3)
   # t.graph_range(times[0],times[1],'clustering method',True)
    #print(times[2])
    #print(times[0])
    #t.graph_range(times[0],times[2][0:143], 'entropies',True)
    #t.graph_range(times[0],times[2],True)
    



if __name__ == '__main__':
    main(sys.argv)