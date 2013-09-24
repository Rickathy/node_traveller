#!/usr/bin/env python

import sys
from travel import travel
from clustering import clustering
from edge_interpreter import edge_interpreter
from datetime import datetime
import matplotlib
import matplotlib.pyplot

def print_after_run():
    t=travel()
    t.set_up_for_test()
    t.load_path_times()
    t.graph_path_times(False)
    t.graph_path_dates(True)

def main(args):
    
    e = edge_interpreter()
    #edge = 29
    t= travel()
    t.set_up_for_test()
    #t.load_path_times()
    #print t.euler_tour(0,[],[])
    '''for i in [0,3,len(t.path_times_list)-5]:
    
        print i
        t.graph_edge(i)
        e = edge_interpreter()
        time = e.interpret(t.path_times_list[i],3)
        t.graph_range(time[0],time[1],'timing estimation - clustering - edge {0}'.format(i))
        time = e.interpret(t.path_times_list[i],4)
        t.graph_range(time[0],time[1],'timing estimation - exponential distribution - edge {0}'.format(i))'''
    #t.graph_path_dates(False)
    #for pt in t.path_times_list:
        #print pt
        #
    #t.graph_edge(26,True)
    #for j in [(0,28),(3,40),(0,40),(10,0),(10,43)]:
     #   print 'between', j
      #  for i in [600,610,620,630,640,650,660,670,680,690,700,710,720]:
       #    print 'time is', i
        #   print 'clust'
         #  route = t.a_star_route_advanced(i,3,j[0],j[1])
          # 
           #temp =[]
           #for n in route:
            #   temp.append(n.node_num)
           #print temp
           #print 'expo'
           #route = t.a_star_route_advanced(i,4,j[0],j[1])
          
           #temp =[]
           #for n in route:
           #    temp.append(n.node_num)
           #print temp
       #t.graph_edge(i)
       #times = e.interpret(t.path_times_list[i],3)
       #t.graph_range(times[0],times[2][0:143],'entropy of edge {0}'.format(i))
       #times = e.interpret(t.path_times_list[i],4)
       #t.graph_range(times[0],times[2][0:143],'confidence intervals over edge {0}'.format(i))
        
      #  t.graph_range(times[0],times[1],'edge traversal estimations - exponential method on edge {0}'.format(i))
    #t.graph_path_dates(True)    
    #for i in [61,68,71]:
     #   t.graph_each_edge_confidence_at_time(i,False)
      #  t.graph_each_edge_entropy_at_time(i,False)
   # for i in range(len(t.path_times_list)):
        #times = e.interpret(t.path_times_list[i],3)
      #  t.graph_edge(i,False)
      #  print i
    #matplotlib.pyplot.show()
    
    #matplotlib.pyplot.show()
    #t.graph_path_times(True)
    #times = e.interpret(t.path_times_list[edge],4)
    #print times[2]
    #times = e.interpret(t.path_times_list[45],4)
    #print times[2]
    #t.graph_edge(45,False)
    #times = e.interpret(t.path_times_list[47],4)
    #print times[2]
    #t.graph_edge(47,True)
    
    #print 'times', times[2]
    #for i in range(len( times[2])):
    #    if times[2][i] < sys.maxint:
    #        print i ,times[2][i], times[1][i]
    #print times[55],times[56],times[57], times[58], times[59], times[60],times[85],times[86],times[87],times[88],times[89]
    
    
    #current_time = 10*int((datetime.today().hour*60+datetime.today().minute)/10)
    #print current_time
   # t=travel()
    #t.set_up_for_test()
    #t.load_path_times()
    #t.graph_path_times(False)
    #t.graph_path_dates(True)
  #  for i in range(len(t.path_times_list)):
  #      print i, t.path_times_list[i].edge.A, t.path_times_list[i].edge.B
    #edge=26
    e = edge_interpreter()
    #times = e.interpret_clustering_ten_minutes(t.path_times_list[5].recordings,True,2,10,True)
    #t.graph_range(times[0],times[2][0:(144*7)-1], 'entropies',True)
    #print times
    #t.temp_remove_records_after(30,7,2013)
  #  print t.path_times_list[26]
    #print len(t.path_times_list[0].recordings)
   # print(t.system_entropy_between_times(55,56))
    #t.graph_each_edge_entropy_at_time(55,False)
    #t.graph_each_edge_confidence_at_time(55,True)
   # t.graph_edge_entropy(edge,True)
    #t.load_path_times()
    #t.temp_remove_records_after(1,8,2013)
   # print t.path_times_list[26]
   # print len(t.path_times_list[0].recordings)
   # print(t.system_entropy_between_times(55,56))
    #t.graph_each_edge_entropy_at_time(55,False)
    #t.graph_each_edge_confidence_at_time(55,True)
   # t.graph_edge_entropy(edge,True)
    #t.load_path_times()
    #t.temp_remove_records_after(2,8,2013)
    
   # print t.path_times_list[26]
    
  #  print(t.system_entropy_between_times(55,56))
    #t.graph_each_edge_entropy_at_time(55,False)
    #t.graph_each_edge_confidence_at_time(55,True)
  #  t.graph_edge_entropy(edge,True)
    #t.load_path_times()
    #t.temp_remove_records_after(3,8,2013)
    
  #  print len(t.path_times_list[0].recordings)
   # print(t.system_entropy_between_times(55,56))
    #t.graph_each_edge_entropy_at_time(55,False)
    #t.graph_each_edge_confidence_at_time(55,True)
  #  t.graph_edge_entropy(edge,True)
   # t.load_path_times()
    #t.temp_remove_records_after(5,8,2013)
  #  print t.path_times_list[26]
   # for i in range(len(t.path_times_list)):
   #     print '{0} : {1}'.format(i, t.path_times_list[i])
    
   # print len(t.path_times_list[0].recordings)
   # print(t.system_entropy_between_times(55,56))
 #   t.graph_each_edge_entropy_at_time(55,False)
  #  t.graph_each_edge_confidence_at_time(55,True)
   # t.graph_edge_entropy(edge,True)'''
    
    '''print 'len',len(t.path_times_list[0].recordings)
    print t.path_times_list[0]
    t.temp_remove_records_after(4,8,2013)
    print('removed')
    print 'len',len(t.path_times_list[0].recordings)
    print t.path_times_list[0]'''
    '''time=580
    edge=16
    t= travel()
    t.set_up_for_test()
    t.load_path_times()
    t.current=0
    list1 =t.a_star_route_setup(time,4)
    list2 =t.a_star_route_setup(time,3)
    print('expected value: exponential--------clustering')
    print(len(list1))
    for i in range(len(list1)):
        print( '{0} -- {1} -- diff:{2}'.format(list1[i], list2[i],(list1[i]-list2[i])))
    start=15
    end =6
    route1= t.a_star_route_advanced(time,3,start,end,[])
    route2 = t.a_star_route_advanced(time,4,start,end,[])
    print('route: clustering - exponential')
    if (len(route1)>len(route2)):
        for i in range(len(route1)):
            if len(route2)<i:
                print '{0}  , -'.format(route1[i].node_num)
            else:
                print '{0} , {1}'.format(route1[i].node_num,route2[i].node_num)
    else:
        for i in range(len(route2)):
            if len(route2)<i:
                print '- , {0}'.format(route2[i].node_num)
            else:
                print '{0} , {1}'.format(route1[i].node_num,route2[i].node_num)
                '''
    #route = t.generate_route_entropy_clusters([],570)
    #print route
    #print len(route)
    #for i in range(len(t.path_times_list)):
     #   print i
      #  print t.path_times_list[i].edge.A.node_num, t.path_times_list[i].edge.B.node_num
    #print t.path_times_list
    #t.graph_path_times(True)end
    #ent = t.generate_entropies()
    #for i in range(len(ent)):
     #   print ent[i][55]
   # print ent[55]
    #t.current=0
    #t.graph_path_times(False)
   # t.graph_path_dates(True)
  #  for i in [1,7,8]:
   #     t.graph_edge(i,False)
    #    times = e.interpret(t.path_times_list[i],3)
     #   t.graph_range(times[0][60:80],times[1][60:79],'edge traversal estimations - clustering method on edge {0}'.format(i))
      #  times = e.interpret(t.path_times_list[i],4)
       # t.graph_range(times[0][60:80],times[1][60:79],'edge traversal estimations - exponential method on edge {0}'.format(i))
    matplotlib.pyplot.show()
  
   # print t.path_times_list[5]
   # c = clustering()
   # print c.cluster_path_times(t.path_times_list[5].recordings)
    #print c.cluster_path_timse(t.path_times)
    #e = edge_interpreter()
    #times = e.interpret(t.path_times_list[10],3)
    #print times
    #t.graph_range(times[0],times[1],'exponential method',True)
    
    #times = e.interpret(t.path_times_list[edge],3)
    #t.graph_range(times[0],times[1],'clustering method',True)
    #print(times[2])
    #print(times[0])
    #print times[2][0:143]
    #t.graph_range(times[0],times[2][0:143], 'entropies',True)
    #t.graph_range(times[0],times[2],True)
    



if __name__ == '__main__':
    main(sys.argv)