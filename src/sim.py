#!/usr/bin/env python

from travel import travel
import rospy
import sys
import math
from travel_node import travel_node
from travel_edge import travel_edge
from datetime import datetime
import random
from edge_interpreter import edge_interpreter
from datetime import timedelta
import matplotlib
import matplotlib.pyplot as plt
import pickle

class sim:
    def __init__(self,mode):
        self.t= travel()
        self.setup_map()
        self.t.initialize_path_times_from_nodes()
        self.mode = mode
    def setup_map(self):
        node0 = travel_node(0,0,0)
        node1 = travel_node(0,1,1)
        node2 = travel_node(0,2,2)
        node3 = travel_node(0,3,3)
        node4 = travel_node(1,1,4)
        node5 = travel_node(1,2,5)
        node6 = travel_node(2,3,6)
        node7 = travel_node(1,3,7)
        node8 = travel_node(1,0,8)
        node9 = travel_node(2,1,9)
        node10 = travel_node(3,2,10)
        node11 = travel_node(3,3,11)
        
        node0.add_connection(node1)
        node1.add_connection(node2)
        node2.add_connection(node3)
        node3.add_connection(node7)
        node0.add_connection(node4)
        node4.add_connection(node5)
        node5.add_connection(node6)
        node6.add_connection(node7)
        node0.add_connection(node8)
        node8.add_connection(node9)
        node9.add_connection(node10)
        node10.add_connection(node11)
        node6.add_connection(node11)
        self.t.nodes=[node0,node1,node2,node3,node4,node5,node6,node7,node8,node9,node10,node11]
    
        
        
    def take_recording(self,edge_no,time,day=1,store=True,noise=True):
        
        date = datetime(2013,1,day,int(time/60), time- 60*int(time/60),0,0)
        if noise:
            noise = random.gauss(0,1)
        else: 
            noise=False
        dist =[5,5,5,5,5,5,5,5,5,5,5,5,5]
        if self.mode==1: # static environment
           # no change
           self.mode=1
        else:
            if self.mode==2:#changes across a day
                
                if time ==650:
                    dist[4]=7
                    dist[6]=8
                    dist[9]=7
                    dist[1]=15
                    dist[2]=9
                    dist[8]=4
                else:
                    if time==640:
                        dist[1]=6
                        dist[4]=15
                        dist[7]=4
                        dist[2]=15
                        dist[3]=8
                    else:
                        if time==630:
                            dist[11]=15
                            dist[4]=15
                            dist[3]=9
                            dist[2]=11
                            dist[7]=90
                            
            else:
                if self.mode==3:#changes across the week
                    if day ==1:
                        dist[3]=8
                        dist[5]=8
                        dist[1]=15
                        dist[4]=9
                        dist[2]=9
                    else:
                        if day ==2:
                            dist[3] =7
                            dist[4]=9
                            dist[8]=15
                            dist[5]=7
                            dist[11]=15
                            dist[9]=30
                        else:
                            if day ==3:
                                dist[1] =7
                                dist[2]=9
                                dist[9]=15
                                dist[8]=7
                                dist[10]=15
                                dist[3]=30
                            else:
                                if day ==4:
                                    dist[2] =7
                                    dist[3]=9
                                    dist[5]=15
                                    dist[6]=7
                                    dist[11]=15
                                    dist[1]=30
                else:
                    if self.mode==4: #changes across all
                        if day ==1:
                            if time>640:
                                dist[3]=7
                                dist[4]=8
                                dist[5]=9
                                dist[7]=8
                                dist[8]=30
                            else:
                                if time==620:
                                    dist[9]=7
                                    dist[7]=9
                                    dist[6]=9
                                    dist[1]=8
                                    dist[2]=30
                                else:
                                    if time ==630:
                                        dist[1]=9
                                        dist[7]=15
                                        dist[8]=14
                                        dist[9]=12
                        else:
                            if day==2:
                                if time==640:
                                    dist[5]=9
                                    dist[7]=7
                                    dist[8]=7
                                    dist[10]=9
                                    dist[11]=11
                                else:
                                    if time == 630:
                                        dist[8]=15
                                        dist[4]=9
                                        dist[3]=30
                                        dist[2]=90
                                        dist[1]=7
                            else:
                                if day==3:
                                    if time==620:
                                        dist[5]=9
                                        dist[7]=7
                                        dist[8]=7
                                        dist[10]=9
                                        dist[11]=11
                                    else:
                                        if time == 610:
                                            dist[8]=15
                                            dist[4]=9
                                            dist[3]=30
                                            dist[2]=90
                                            dist[1]=7
                                else:
                                    if day==4:
                                        if time==620:
                                            dist[5]=9
                                            dist[7]=7
                                            dist[8]=7
                                            dist[10]=9
                                            dist[11]=11
                                        else:
                                                if time == 610:
                                                    dist[8]=15
                                                    dist[4]=9
                                                    dist[3]=30
                                                    dist[2]=90
                                                    dist[1]=7
                                
                    else:
                        if self.mode==5:#furst to have bimodals in, constant across days
                            dist[5]=[0.9,5,0.1,9]
                            dist[7]=[0.5,5,0.5,10]
                            dist[8]=[0.5,4,0.5,10]
                            dist[1]=[0.9,5,0.1,90]
                            dist[2]=[0.5,5,0.5,10]
                            dist[3]=[0.5,4,0.5,10]
                        else:
                            if self.mode==6:# bimodal ,changing with  day
                                if day==3:
                                    dist[7]=[0.9,10,0.1,15]
                                    dist[9]=[0.5,5,0.5,20]
                                    dist[4]=[0.5,4,0.5,10]
                                    dist[5]=[0.9,5,0.1,9]
                                    dist[2]=[0.5,5,0.5,90]
                                    dist[8]=[0.5,4,0.5,10]
                                else:
                                    if day ==5:
                                        dist[3]=[0.5,6,0.5,8]
                                        dist[1]=[0.9,1,0.1,10]
                                        dist[2]=[0.5,4,0.5,10]
                                        dist[5]=[0.9,5,0.1,9]
                                        dist[7]=[0.1,5,0.9,90]
                                        dist[8]=[0.5,4,0.5,10]  
                                    else:
                                        if day ==5:
                                            dist[4]=[0.5,6,0.5,8]
                                            dist[9]=[0.9,1,0.1,10]
                                            dist[10]=[0.5,4,0.5,10]
                                            dist[11]=[0.9,5,0.1,9]
                                            dist[8]=[0.1,5,0.9,90]
                                            dist[7]=[0.5,4,0.5,10]  
                            else:
                                if self.mode==7:#bimodal with changes across both week and day
                                    if day==1:
                                        if time <640:
                                            dist[4]=[0.4,5,0.6,7]
                                            dist[7]=[0.1,5,0.9,8]
                                            dist[8]=[0.5,4,0.5,10]
                                            dist[1]=[0.4,5,0.6,30]
                                            dist[2]=[0.1,5,0.9,8]
                                            dist[3]=[0.5,4,0.5,10]
                                    else:
                                        if day == 3:
                                            if time==650:
                                                dist[4]=[0.5,6,0.5,9]
                                                dist[2]=[0.1,5,0.9,7]
                                                dist[7]=[0.5,4,0.5,10]
                                                dist[4]=[0.4,5,0.6,7]
                                                dist[1]=[0.1,5,0.9,8]
                                                dist[3]=[0.5,4,0.5,40]
                                        else:
                                            if day ==5:
                                                if time==630:
                                                    dist[1]=[0.5,6,0.5,9]
                                                    dist[2]=[0.1,5,0.9,7]
                                                    dist[7]=[0.5,4,0.5,10]
                                                    dist[5]=[0.4,5,0.6,7]
                                                    dist[6]=[0.1,5,0.9,8]
                                                    dist[7]=[0.5,4,0.5,40]
                                    
                            
                            
                                        
             
        
        
        if isinstance(dist[edge_no],int):
           durr = dist[edge_no]+noise
           
               
        else:
            
                chance = random.uniform(0,1)
                cum_chance =0
                for i in range(len(dist[edge_no])/2):
                    cum_chance+=dist[edge_no][i*2]
                    if cum_chance > chance:
                        durr = dist[edge_no][1+i*2]+noise
                        break
           
        if durr ==0:
            duration=None
        else:  
            if durr<3:
                durr=3
            duration = timedelta(seconds= int(durr), microseconds=(durr-int(durr))*(10**6.))
       
        if store==True:
            self.t.path_times_list[edge_no].add_false_recording(duration,date)
        else:
            return durr        
    
    def interpret(self,edge,time):
        e=edge_interpreter()
        print e.interpret(self.t.path_times_list[edge],3)
    def generate_route_at_time(self,time,type,days):
        self.t.current=self.t.get_first_node_clusters(time,days,type)
        return self.t.generate_route_entropy([],time,type=type,days=days)
    def follow_route(self,route,time,day,store,max=False):
        route_duration=0
        if max==False:
            max =  len(route)-1
        for i in range(0,max):
            for j in range( len(self.t.path_times_list)):
                if ((self.t.path_times_list[j].edge.A.node_num==route[i].node_num) & (self.t.path_times_list[j].edge.B.node_num==route[i-1].node_num)) | ((self.t.path_times_list[j].edge.B.node_num==route[i].node_num) & (self.t.path_times_list[j].edge.A.node_num==route[i-1].node_num)):
                    route_duration +=self.take_recording(j,time,day,store)
        return route_duration       
        #print self.t.nodes
    def test_all_routes(self,type,setup=True,days=True):
        days=[]
        for day in range(1,7):
            print 'day is {0}'.format(day)
            slices=[]
            for i in range(600,660,10):
                
                if setup:
                    blocked = self.t.a_star_route_setup(i, type,days)
                else:
                    self.setup_ground_truth(i, day)
                    blocked=[]
                
                
                #print 'slice is {0}'.format(i)
                
                slice=0
                for n in range(len(self.t.path_times_list)-1):
                    #print n
                    for j in range(len(self.t.path_times_list)-1):
                        #print j
                        if (n !=j) & (n<j):# dont want to go from a node to itself + only want to do each path once
                            average=0
                            route = self.t.a_star(n,j,blocked)
                            
                            for k in range(10):
                              #  print 'k {0}'.format(k)
                                
                                average+= self.follow_route(route,i,day,False)
                            average/=10
                            slice+=average
                        
                slice/=((n-1)*(n-1))
                slices.append(slice)
            days.append(slices)
        
        return days
    def setup_ground_truth(self,time,day):
        for i in range(len(self.t.path_times_list)):
            sum =0
            for j in range(10):
                sum+=self.take_recording(i,time,day,False)        
            self.t.path_times_list[i].edge.cost =  (sum/10) 
          
    def run_tests(self,days):
        truth = self.test_all_routes(0,False,days)
        clust = self.test_all_routes(3,True,days)
        ex = self.test_all_routes(4,True,days)
        
        clust_diff =[]
        clust_sd =0
        ex_diff=[]
        ex_sd=0
        for i in range(len(truth)):
            for j in range(len(truth[i])):
                clust_diff.append(abs(truth[i][j]-clust[i][j]))
                clust_sd+=((truth[i][j]-clust[i][j])**2)
                
                ex_diff.append(abs(truth[i][j]-ex[i][j]))
                ex_sd+= ((truth[i][j]-ex[i][j])**2)
        print clust_diff
        clust_sd=math.sqrt(clust_sd/(len(truth)*len(truth[i])))       
        ex_sd=math.sqrt(ex_sd/(len(truth)*len(truth[i])))
        clust_av=sum(clust_diff)/len(clust_diff)
        ex_av=sum(ex_diff)/len(ex_diff)
        return(clust_av,clust_sd,ex_av,ex_sd)
    def naive_day(self,day,edges_repeat):
        times= range(600,660,10)
        for i in range(len(self.t.path_times_list)):
            for j in range(edges_repeat):
               for k in range(len(times)):
                    self.take_recording(i,times[k],day)
    def intelligent_day(self,day,type,edges_per_hour,days):
        for i in range(600,660,10):
            route = self.generate_route_at_time(i,type=type,days=days)
            if edges_per_hour>len(route):
                edges_per_hour=len(route)
            
            for k in range(edges_per_hour):
                for j in range( len(self.t.path_times_list)):
                    if ((self.t.path_times_list[j].edge.A.node_num==route[k]) & (self.t.path_times_list[j].edge.B.node_num==route[k-1])) | ((self.t.path_times_list[j].edge.B.node_num==route[k]) & (self.t.path_times_list[j].edge.A.node_num==route[k-1])):
                        self.take_recording(j,i,day,True)
    def dumb_day(self, date, number_of_edges):
        for i in range(600,660,10):
            for j in range(number_of_edges):
                self.take_recording(random.randint(0,len(self.t.path_times_list)-1),i,date)
def do_sim(sim_type,tasks,days):
    s = sim(sim_type)
    clust_averages=[]
    clust_sds=[]
    expo_averages=[]
    expo_sds=[]
    for (task,date) in tasks:
        if task==0:#just do a naive run on that day
            s.naive_day(date,3)
        else:
            if task==1:#do a entropy based exploration on that day
                s.intelligent_day(date,3,3,days)
            else:
                if task==2:# do a confidence interval based exploration on that day
                    s.intelligent_day(date,4,3,days)
                else:
                    if task==3:#do random smaples
                        s.dumb_day(date,3)
        (clust_av,clust_sd,exp_av,exp_sd) =s.run_tests(days)
        clust_averages.append(clust_av)
        clust_sds.append(clust_sd)
        expo_averages.append(exp_av)
        expo_sds.append(exp_sd)
    return (clust_averages,clust_sds,expo_averages,expo_sds)
                    
                
            
        
def main(args):
    if len(args)>2:
        if args[1]==1:
            from_file=True
        else:
            from_file==False
        if args[2]==1:
            week_long_start==True
        else:
            week_long_start=False
        
        if from_file== False:
            task = []
            if week_long_start==False:
                actions = [[0,1],[1,2],[1,3],[1,4],[1,5],[1,6],[1,7],[1,1],[1,2],[1,3],[1,4],[1,5],[1,6],[1,7]]
            else:
                actions = [[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[1,1],[1,2],[1,3],[1,4],[1,5],[1,6],[1,7]]
            for i in range(1,7):
                print('first batch',i)
                task.append([])
                (clust_averages,clust_sds,expo_averages,expo_sds) = do_sim(i,actions,True)
                task[i-1].append(clust_averages)
                (clust_averages,clust_sds,expo_averages,expo_sds) =do_sim(i,actions,False)
                task[i-1].append(clust_averages)
            if week_long_start==False:
                        actions = [[0,1],[2,2],[2,3],[2,4],[2,5],[2,6],[2,7],[2,1],[2,2],[2,3],[2,4],[2,5],[2,6],[2,7]]
            else:
                actions= [[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[2,1],[2,2],[2,3],[2,4],[2,5],[2,6],[2,7]]
            for i in range(1,7):
                print('second batch',i)
                (clust_averages,clust_sds,expo_averages,expo_sds) =do_sim(i,actions,True)
                task[i-1].append(expo_averages)
                (clust_averages,clust_sds,expo_averages,expo_sds) =do_sim(i,actions,False)
                task[i-1].append(expo_averages)
            if week_long_start==False:
                actions = [[0,1],[3,2],[3,3],[3,4],[3,5],[3,6],[3,7],[3,1],[3,2],[3,3],[3,4],[3,5],[3,6],[3,7]]
            else:
                actions=  [[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[3,1],[3,2],[3,3],[3,4],[3,5],[3,6],[3,7]]
            
            for i in range(1,7):
                print('third batch',i)
                (clust_averages,clust_sds,expo_averages,expo_sds) =do_sim(i,actions,False)
                task[i-1].append(clust_averages)
                task[i-1].append(expo_averages)
            try:
                file = open("thesis_experiemnts.ex","wb")
                pickle.dump( task, file )
            except EOFError:
                print 'error, but lets just carry on anyway'
        else:
            file =open("thesis_experiemnts.ex","rb")
            task = pickle.load(file)
        plt.gca().set_color_cycle(["b", "g", "r", "c", "m", "y", "k"])
        #plt.legend(('clusters - days', 'clusters', 'exponential - days', 'exponential','random exploration - averages','random exploration - exponential'),loc=2)
        print task[1]
        for i in range(6):
            #print len(task[i])
            print len([1,2,3,4,5,6,7,8,9,10,11,12,13,14])
            for j in range(len(task[i])):
                plt.plot([1,2,3,4,5,6,7,8,9,10,11,12,13,14],task[i][j])
                plt.title('Environment {0}'.format(i+1))
                plt.legend(('clusters - days', 'clusters', 'exponential - days', 'exponential','random exploration - averages','random exploration - exponential'),loc=1)
                plt.xlabel('Day of simulation')
                plt.ylabel('Average time over optimum (seconds)')
                
            #plt.show()
            #plt.close()
            plt.figure()
        #s = sim(3)
        #days =True
        #s.naive_day(1)
        
        #(day_one_clust_av,day_one_clust_sd,day_one_ex_av,day_one_ex_sd) =s.run_tests(days)
        
        
        #for i in range(len(s.t.path_times_list)):
        #s.intelligent_day(2,3,5,days)
        #(day_two_clust_av,day_two_clust_sd,day_two_ex_av,day_two_ex_sd) =s.run_tests(days)
        
        
        
        #s.intelligent_day(3,3,5,days)
        #(day_three_clust_av,day_three_clust_sd,day_three_ex_av,day_three_ex_sd) =s.run_tests(days)
        
        
        
        
        #s.intelligent_day(4,3,5,days)
        #(day_four_clust_av,day_four_clust_sd,day_four_ex_av,day_four_ex_sd) =s.run_tests(days)
        #print 'day one'
        #print 'clustering, average error:{0} , s.d: {1}'.format(day_one_clust_av,day_one_clust_sd)
        #print 'exponential, average error:{0} , s.d: {1}'.format(day_one_ex_av,day_one_ex_sd)
        #print 'day two'
        #print 'clustering, average error:{0} , s.d: {1}'.format(day_two_clust_av,day_two_clust_sd)
        #print 'exponential, average error:{0} , s.d: {1}'.format(day_two_ex_av,day_two_ex_sd)
        #print 'day three'
        #print 'clustering, average error:{0} , s.d: {1}'.format(day_three_clust_av,day_three_clust_sd)
        #print 'exponential, average error:{0} , s.d: {1}'.format(day_three_ex_av,day_three_ex_sd)
        #print 'day four'
        #print 'clustering, average error:{0} , s.d: {1}'.format(day_four_clust_av,day_four_clust_sd)
        #print 'exponential, average error:{0} , s.d: {1}'.format(day_four_ex_av,day_four_ex_sd)
        
      
       
       #for pt in s.t.path_times_list:
         #   print pt            
        #s.t.graph_edge(0)
        #e= edge_interpreter()
        #times = e.interpret(s.t.path_times_list[0],3)
        #s.t.graph_range(times[0][60:72],times[1][60:71],'edge traversal estimations - clustering')
        #s.t.graph_range(times[0],times[2][0:143],'entropy')
        #times = e.interpret(s.t.path_times_list[0],4)
        #s.t.graph_range(times[0][60:72],times[1][60:71],'edge traversal estimations - exponential')
        #s.t.graph_range(times[0],times[2][0:143],'confidence interval')
        #s.t.graph_path_times(True)
       # print day_one_clust-truth
        #print day_one_hier-truth
       # for pt in path_times
        #for i in range(0,10):
          #  print random.gauss(0,1)
       # print 'count',count
       # route = s.generate_route_at_time(650,type=4)
       # print 'route is',route
        #s.follow_route(route ,650)
        #print s.t.path_times_list
        #time =630
        #print 'clustering route'
        #route= s.t.a_star_route_advanced(time,3,0,7)
        #for node in route:
         #   print node
            
        #print 'exponential route'
        #route= s.t.a_star_route_advanced(time,4,0,7)
        #for node in route:
       #     print node
        #e = edge_interpreter()
        #s.t.graph_edge(0,False)
        #times = e.interpret(s.t.path_times_list[0],3)
        #s.t.graph_range(times[0],times[1],'edge traversal estimations - clustering')
        #s.t.graph_range(times[0],times[2][0:143],'entropy')
        #times = e.interpret(s.t.path_times_list[0],4)
        #s.t.graph_range(times[0],times[1],'edge traversal estimations - exponential')
        #s.t.graph_range(times[0],times[2][0:143],'confidence interval')
        
        #matplotlib.pyplot.show()
        #count=0
        #for pt in s.t.path_times_list:
         #   print pt
            #for rec in pt.recordings:
          #      count+=1
        #print 'count',count
    else:
        print 'Arguments required to run:'
        print 'First argument: From file or not, used when editing code to  change graphs used. 0 to use old data,1 to generate new data.'
        print 'Second argument:1for system will do 7 days of naive exploration then 7 using intelligent search.'
        print '0 will do a single day of intelligent search, then 13 consecutive days of intelligent search.'
        
if __name__ == '__main__':
    main(sys.argv)