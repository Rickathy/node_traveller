#!/usr/bin/env python

#ROS dependences
import rospy
import roslib
import actionlib
import tf

import sys
import math
import random
import time
import pickle
from datetime import datetime

import sys, numpy, scipy
import scipy.cluster.hierarchy as hier
import scipy.spatial.distance as dist


roslib.load_manifest('node_traveller')
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionGoal

# a class for travelling between nodes of a graph
class travel:

   # initializes this node
    def __init__(self):
        rospy.init_node('travel')
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.pub = rospy.Publisher("move_base",MoveBaseAction)
        self.listener = tf.TransformListener()
        self.path_times_list = []
    def save_path_times(self):
        pickle.dump( self.path_times_list, open( "path_times.pt", "wb" ) )
    def load_path_times(self):
        self.path_times_list = pickle.load( open( "path_times.pt", "rb" ) )
    def initialize_path_times_from_nodes(self):
        edges =[]
        for node in self.nodes:
            for edge in node.connections:
                if edge  not in edges:
                    edges.append(edge)
        for edge in edges:
            self.path_times_list.append(path_times(edge))
    # initializes a test map to be used for navigation testing
    def initialize_test_map(self,num):
        if num ==0:## no switch statements in python, so use nested ifs instead
            self.initialize_test_map_one()
    def initialize_test_map_one(self):
        #start node
        node0 =node(50,50,0)
        #entrance room1
        node1 =node(50,49.3,1)
        node0.add_connection(node1)
        #room1
        node2 = node(49.9,48.2,2)
        node3 = node(49.7,46.3,3)
        node4 = node(49.9,44.5,4)
        node1.add_connection(node2)
        node2.add_connection(node3)
        node3.add_connection(node4)
        #entrance room2
        node5=node(49.9,50.5,5)
        node0.add_connection(node5)
        #room2
        node6=node(49.9,52.1,6)
        node7=node(49.8,53.7,7)
        node8=node(49.8,55.3,8)
        node9=node(51.7,55.1,9)
        node10=node(53.4,55,10)
        node11=node(53.6,52.5,11)
        node12=node(51.5,52.5,12)
        node5.add_connection(node6)
        node6.add_connection(node7)
        node7.add_connection(node8)
        node8.add_connection(node9)
        node9.add_connection(node10)
        node10.add_connection(node11)
        node11.add_connection(node12)
        node12.add_connection(node6)

        node7.add_connection(node12)

        self.nodes = [node0,node1,node2,node3,node4,node5,node6,node7,node8,node9,node10,node11,node12]



    # changes the robot's heading to point at the goal node
    def set_heading(self,current,goal):
        x0 =current[0][0]
        y0 =current[0][1]
        x1 =x0+1
        y1 =y0
        x2 =goal.target_pose.pose.position.x
        y2=goal.target_pose.pose.position.y
        angle1 = math.atan2(y0-y1,x0-x1)

        angle2 = math.atan2(y0-y2,x0-x2)
        new_heading = angle1-angle2

        heading_goal = MoveBaseGoal()
        heading_goal.target_pose.header.frame_id="base_link"
        heading_goal.target_pose.header.stamp=rospy.Time.now()
        (r,p,y) = tf.transformations.euler_from_quaternion([current[1][0],current[1][1],current[1][2],current[1][3]])
        new_heading = -new_heading

        [rx,ry,rz,rw] =tf.transformations.quaternion_from_euler(0,0,new_heading-y)
        heading_goal.target_pose.pose.orientation.x=rx
        heading_goal.target_pose.pose.orientation.y=ry
        heading_goal.target_pose.pose.orientation.z=rz
        heading_goal.target_pose.pose.orientation.w=rw
        print( 'about to change heading to node')
        self.move_base.send_goal_and_wait(heading_goal)
        print('heading changed')
        print(self.move_base.get_goal_status_text())


    #gets the robots current position
    def get_position(self):
        found =False
        while not found  :
            try:
                trans  = self.listener.lookupTransform('/map','/base_link',rospy.Time(0))
                found = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        return trans

    def get_current_node(self):
        trans=self.get_position()
        distance = sys.maxint
        closest = 0
        for i in range(len(self.nodes)):
            if self.euclidean(trans[0][0],trans[0][1],self.nodes[i].x,self.nodes[i].y)< distance:
                distance = self.euclidean(trans[0][0],trans[0][1],self.nodes[i].x,self.nodes[i].y)
                closest = i
        self.current = closest
    # sends the robot to the specified (x,y) coordinate of the map
    # records the time taken to travel in a straight line between the nodes
    def head_to_position(self,node):

        trans=self.get_position()
        print( trans)

        #wait for the action server to be available
        move_base_client =actionlib.SimpleActionClient('move_base',MoveBaseAction)
        print('waiting for server')
        move_base_client.wait_for_server()
        print( 'server found')
        #construct a simple goal in the base_link frame

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id="map"
        goal.target_pose.header.stamp=rospy.Time.now()
        goal.target_pose.pose.position.x=node.x
        goal.target_pose.pose.position.y=node.y
        goal.target_pose.pose.position.z=trans[0][2]
        goal.target_pose.pose.orientation.x =trans[1][0]
        goal.target_pose.pose.orientation.y =trans[1][1]
        goal.target_pose.pose.orientation.z =trans[1][2]
        goal.target_pose.pose.orientation.w =trans[1][3]

        self.set_heading(trans,goal)

        #send the goal and wait for the base to get there
        print( "heading to node: ",node.node_num,'at position: ',node.x,', ',node.y)
        time_start = datetime.today()
        move_base_client.send_goal(goal)
        target_reached = False
        while not target_reached:
            trans  = self.listener.lookupTransform('/map','/base_link',rospy.Time(0))
            diff1= trans[0][0]-goal.target_pose.pose.position.x
            diff2 = trans[0][1]-goal.target_pose.pose.position.y
            tot = math.sqrt(diff1*diff1+diff2*diff2)

            if tot<0.1: #threshold for how close we want to be
                target_reached=True
                move_base_client.cancel_goal()
                for path_time in self.path_times_list:
                    if ((path_time.edge.A.node_num==self.current) & (path_time.edge.B.node_num==node.node_num )) | ((path_time.edge.B.node_num==self.current) & (path_time.edge.A.node_num==node.node_num )):
                        path_time.add_recording(datetime.today() -time_start)
                        print(  'time taken' ,datetime.today() -time_start)
                self.current = node.node_num
            else:
                goal_status = move_base_client.get_state()
               # if goal_status is not
                if (goal_status is not 1) & (goal_status is not 0):# 0= pending, 1= active
                    print('failed to travel to node')
                    return False




        return True

    # travel through all the nodes in the order they were received (ignored connections currently)
    def travel_all_nodes_naive(self):
        for node in self.nodes:
            self.head_to_position(node)
    # follows a provided path (nodes need to be connected in the path for proper timing gathering)
    def follow_path(self, path):
        for node in path:
            if self.head_to_position(node) is False:
                print('Failed to follow path')
                return
    #generate an a star route from the current position
    # assumes the robot knows the current node it is on
    def a_star_from_current(self,end):
        return self.a_star(self.current,end)

    #performs an a* search between two nodes
    # the path returned assumes we are starting at the start node (so does not include it in the path)
    def a_star(self,start,end):

        visited_edges = []
        visited_nodes = []
        frontier = [a_star_node(self.nodes[start],None,self.nodes[end])]
        #print('frontier')
       # print(frontier[0])
        while True:
            best= sys.maxint
            best_pos=0
            for i in range(len(frontier)):
                #print('current best is ',best)
                #print('looking at heuristic of: ',frontier[i].heuristic)
                if frontier[i].heuristic<best:
                   # print('replaced')
                    best= frontier[i].heuristic
                    best_pos=i
            ## we have found the best node at this stage of the search
            best_node = frontier[best_pos]
            frontier.remove(best_node)
            visited_nodes.append(best_node)
            #print 'best node', best_node
            #print 'end ', end
            if best_node.node is self.nodes[end]:
                # we have found the goal node, can return the path to it
                return self.find_path(best_node)
          #  rospy.sleep(1)
            #print '---------------------visited edges-------------------------\n',visited_edges
            for edge in best_node.node.connections:
                if edge not in visited_edges:
                    vis =False
                    for n in visited_nodes:

                        if (n.node is edge.A) | (n.node is edge.B):
                            # This node has been visited before
                            vis=True
                    visited_edges.append(edge)
                    if edge.A is best_node.node:
                      #  print('making new node')
                        frontier.append(a_star_node(edge.B,best_node,self.nodes[end]))
                    else:
                      #  print('making new node')
                        frontier.append(a_star_node(edge.A,best_node,self.nodes[end]))

    ## given an astar node, returns  the path from the start
    # The path is composed of the node ids
    def find_path(self,end):
        path=[]
        current = end
        while current.parent is not None:
            path.insert(0,current.node)
            current=current.parent

        return path
    # prints a route (of nodes)
    def print_route(self,route):
        for node in route:
            print('node :',node.x, ', ' , node.y,' \n')
    #returns the distance between two points
    def euclidean(self,x1,y1,x2,y2):
        return math.sqrt(math.pow(x1-x2,2)+math.pow(y1-y2,2))

    def follow_exploration_route(self, route):
        for node in route:
            path = self.a_star_from_current(node)
            self.follow_path(path)

    def visit_all_nodes(self):
        self.get_current_node()
        route = []
        for i in range(len(self.nodes)):
            route.append(i)
        self.follow_exploration_route(route)

    def best_route_through_all_edges(self,start,n):
        best=[]
        length =sys.maxint
        for i in range(n):
            candidate = self.route_through_all_edges(start)
            if len(candidate)<length:
                length = len(candidate)
                best = candidate
        return candidate

    # create a route through all the edges in the map
    def route_through_all_edges(self, start):
        route = []
        current_pos = start
        remaining_edges= [] # create a list of all the edges
        visited_edges=[]
        for path_times in self.path_times_list:
            remaining_edges.append(path_times.edge)
            #we now have a list of all the edges.
        '''simplest case is to just visit each one, hopefully we can remove any possible of duplicates by looking at the path generated
        by A* and removing additional nodes from our to visit list'''
        while len(remaining_edges) is not 0:
            current_edge =remaining_edges[random.randint(0,len(remaining_edges)-1)]
            #remaining_edges.remove(current_edge)

            routeA =self.a_star(current_pos,current_edge.A.node_num)
            routeB = self.a_star(current_pos,current_edge.B.node_num)
            ''' Whichever route is longest, must be the one that encompasses the edge we are examining'''
            if len(routeA)>len(routeB):
                current_route = routeA
            else:
                current_route=routeB
           # print( 'current route ')
           # for node in current_route:
           #     print( node)
            for i in range(len(current_route)):

                found = False
                for edge in remaining_edges:

                    if (edge.A.node_num==current_pos) & (edge.B.node_num==current_route[i].node_num):
                        remaining_edges.remove(edge)
                        visited_edges.append(edge)
                        current_pos = edge.B.node_num
                        found=True
                        break
                    else:
                        if (edge.B.node_num==current_pos) & (edge.A.node_num==current_route[i].node_num):
                            remaining_edges.remove(edge)
                            visited_edges.append(edge)
                            current_pos=edge.A.node_num
                            found=True
                            break
                if not found:
                    for edge in visited_edges:
                        if (edge.A.node_num==current_pos) & (edge.B.node_num==current_route[i].node_num):
                            current_pos = edge.B.node_num
                            break
                        else:
                            if (edge.B.node_num==current_pos) & (edge.A.node_num==current_route[i].node_num):
                                current_pos=edge.A.node_num
                                break


                route.append(current_route[i])
           # print( 'route')
           # for node in route:
           #     print( node)
           # print( 'remaining edges ')
           # for edge in remaining_edges:
           #     print( edge)

        return route
    # visit every edge in the map
    def visit_all_edges(self):
        self.get_current_node()
        route = self.best_route_through_all_edges(self.current,10)
        print('route made')
        print(route)
        path=[]
        for node in route:
            path.append(node.node_num)
        self.follow_exploration_route(path)

    # finds all the odd nodes in the graph
    def find_odd_nodes(self):
        odd =[]
        for node in self.nodes:
            if len(node.connections) % 2 ==0:
                continue
            else:
                odd.append(node.node_num)
        return odd
    def create_odd_graph(self ):
        odd_list = self.find_odd_nodes()
        print('odd list', odd_list)
        matrix =[]
        for start in odd_list:
            current =[]
            for end in odd_list:
                length = len(self.a_star(start,end))
                current.append(length)
            matrix.append(current)
        return numpy.array(matrix), odd_list
    # first generates a graph comprising only those nodes with odd degree
    # then connects nodes so that they have even degree with nearest neighbours
    # returns the last path as well, if the start or end of it are the current position they can be ignored.
    # (maybe if any of the nodes are the start or end position they can be ignored?)
    def connected_odd_graph(self,position):
        graph,nodes = self.create_odd_graph()
        potential_shortcut=[]
        shortcut_gain=0
        visited = []
        connections = []
        additions=0
        print('graph',graph)
        print(len(graph))
        while additions < (len(graph)/2):
            lowest = sys.maxint
            pos =0

            for i in range(len(graph)):
                for j in range(len(graph[0])):
                    if (i not in visited) & (j not in visited) &  (graph[i][j]< lowest) &( graph[i][j] != 0)  :

                        lowest = graph[i][j]
                        pos = (i,j)
            print(pos)
            route =self.a_star(nodes[pos[0]],nodes[pos[1]])

            connections.append((nodes[pos[0]],route[0].node_num))
            for i in range(len(route)-1):
                connections.append((route[i].node_num,route[i+1].node_num))

            additions +=1
            visited.append(pos[0])
            visited.append(pos[1])
            print('route print', route)
            for node in route:
                print node.node_num
            print('start pos ',self.nodes[position].node_num)

            if nodes[pos[0]] == self.nodes[position].node_num:

                if len(route)>shortcut_gain:
                    'found improvement C'
                    shortcut_gain = len(route)
                    potential_shortcut= list(route)
                    potential_shortcut.insert(0,self.nodes[nodes[pos[0]]])
            print self.nodes[position]
            print 'route'
            print self.nodes[position] in route
            for node in route:
                print(nodes)
            if self.nodes[position] in route:

                print 'node in route'
                index = route.index(self.nodes[position])+1
                print 'index', index
                if index*2<len(route):#node is in first half of path
                    print 'F'
                    print len(potential_shortcut)
                    print shortcut_gain
                    if (len(route)-index)>shortcut_gain:
                        print'found improvement A'
                        potential_shortcut= list(route)
                        potential_shortcut.insert(0,self.nodes[nodes[pos[0]]])
                        shortcut_gain=index

                else:# nodes is in second half of path
                    print 'G'
                    if index> shortcut_gain:
                        print'found improvement B'
                        potential_shortcut= route
                        #potential_shortcut.insert(0,self.nodes[nodes[pos[0]]])
                        shortcut_gain=len(route)-index
        print('number of connections ', len(connections))
        print( 'route')
        print( self.nodes[nodes[pos[0]]])
        route.insert(0,self.nodes[nodes[pos[0]]])
        for node in route:
            print( node.node_num)
        print('shortcut')
        for node in potential_shortcut:
            print node.node_num
        short_cut_index = potential_shortcut.index(self.nodes[position])
        print('connections', connections)
        if short_cut_index*2<len(route):
            print short_cut_index
            if short_cut_index==0  :
                print('A')
                for i in range(0,len(potential_shortcut)-1):
                    connections.remove((potential_shortcut[i].node_num,potential_shortcut[i+1].node_num))
            else:
                print('B')
                for i in range(short_cut_index,len(potential_shortcut)-1):
                    print((potential_shortcut[i].node_num,potential_shortcut[i+1].node_num))
                    connections.remove((potential_shortcut[i].node_num,potential_shortcut[i+1].node_num))
        else:
            if short_cut_index == len(potential_shortcut)-1:
                print('C')
                for i in range(0,len(potential_shortcut)-1):
                    connections.remove((potential_shortcut[i].node_num,potential_shortcut[i+1].node_num))


            else:
                print('D')
                for i in range(0,short_cut_index):
                    print((potential_shortcut[i].node_num,potential_shortcut[i+1].node_num))
                    connections.remove((potential_shortcut[i].node_num,potential_shortcut[i+1].node_num))



        return connections
    def copy_connections(self):
        graph =[]
        for node in self.nodes:
            for edge in node.connections:
                if edge.A.node_num == node.node_num : # maintains only one of each edge
                    graph.append((node.node_num,edge.B.node_num))
        return graph
    def euler_tour(self,position):
        odd_graph = self.connected_odd_graph(position)
        graph = self.copy_connections()

        for odd_edge in odd_graph:
            graph.append(odd_edge)
        print('graph is', graph)
        tour = self.find_eulerian_tour(graph,position)
        tour = self.remove_duplicates_from_tour(tour)
        tour.reverse()
        print('before repeat removal', tour)
        self.remove_repeat_visit(tour)
        #print ('before move of start', tour)
        #tour = self.set_start(position,tour)

        return tour

    def find_eulerian_tour(self,graph,start):
        whole_tour=[]
        self.E = graph[:]                   # copy the graph so we don't destroy it
        First = True
        self.tour =[]
        self.find_tour(start)
        for node in self.tour:
            whole_tour.append(node)
        while len(self.E)> 0:
            self.tour = []                      # the tour starts out empty
            self.find_tour(self.E[0][0])             # find a tour using the first node in the edge list

            for node in self.tour:
                whole_tour.append(node)

        return whole_tour

    def find_tour(self,u):
        for (i, j) in self.E:           # find an edge with u as the source node
            if i == u:             # check each edge going one way
                self.E.remove((i, j))   # remove the found edge so it isn't used twice
                self.find_tour(j)       # continue the tour from the sink node
            elif j == u:           # check each edge going the other way if we need to
                self.E.remove((i, j))
                self.find_tour(i)
        self.tour.append(u)             # we found an edge from u, so its part of the tour

    def remove_duplicates_from_tour(self,tour):
        new_tour=[]
        for i in range(len(tour)-1):
            if tour[i]!= tour[i+1]:
                new_tour.append(tour[i])
        if tour[len(tour)-1]!= tour[(len(tour))-2]:
            new_tour.append(tour[len(tour)-1])
        return new_tour

    def remove_repeat_visit(self,tour):
        #start at the end of the journey and keep going back until a path is found that is unvisited
        position = len(tour)-1
        while self.more_than_single_instance(tour, tour[position],tour[position-1]):
            tour.pop(position)
            position-=1
    # return true if there is more than one instance of this traversal
    def more_than_single_instance(self,tour,nodeA,nodeB):
        count =0
        for i in range(len(tour)-1):
            if ((tour[i] == nodeA) & (tour[i+1]== nodeB)) | ((tour[i] == nodeB) & (tour[i+1]== nodeA)):
                count+=1
                if count >1:
                    return True
        return False



    def set_start(self,start_pos, tour):
        while tour[0] != start_pos:
            tour.append(tour.pop(0))
        return tour



# for use in a* search
class a_star_node:

    def __init__(self,node,parent,goal):
        self.node=node
        if parent is None:
        #this is the first node in the search
            self.parent=None
            self.cost=0
        else:
            self.parent=parent
           # print('parent')
           # print( parent)
            edge = self.find_edge(parent.node)
          #  print( 'edge: ',edge)
            self.cost=parent.cost+edge.cost

        self.heuristic = self.cost + self.euclidean(node.x,node.y,goal.x,goal.y)
    def __str__(self):
        return 'A* node\nnode: {0} \n parent: {1} \n cost: {2} \n heuristic: {3} '.format(self.node , self.parent ,  self.cost , self.heuristic)
    def find_edge(self,destination):
        for conn in self.node.connections:

            if (conn.A is destination) | (conn.B is destination):
                return conn
        ## we shouldn't be able to return nothing, but just in case
        return None
    def euclidean(self,x1,y1,x2,y2):
        return math.sqrt(math.pow(x1-x2,2)+math.pow(y1-y2,2))

#class to represent a node in a graph
class node:
    # it's just an x and y coordinate
    def __init__ (self,x,y,node_num):
        self.x=x
        self.y=y
        self.connections= []
        self.node_num=node_num
    # add an edge between this node and another
    def __eq__(self,other):
        self.x=other.x
        self.y=other.y
    def add_connection(self,node):
        connection = edge((self,node))
        self.connections.append(connection)
        node.connections.append(connection)
    def __str__(self):
        return str('node number is {0}'.format(self.node_num))

class path_time:
    def __init__(self,date,time):
        self.date=date #when the reading was recorded
        self.time=time #how long the reading took
    def __rpr__(self):
        return str('Traversal at {0} took {1}'.format(self.date,self.time))
    def __str__(self):
        return str('Traversal at {0} took {1}'.format(self.date,self.time))

#class to represent a bunch of timing recordings between a node pair
class path_times:

    def __init__(self,edge):
        self.edge=edge
        self.recordings = []
    def add_recording(self, time_taken):
        self.recordings.append(path_time(datetime.today(),time_taken))
    def __str__(self):
        path_string =[]
        for recording in self.recordings:
            path_string.append(str(recording))

        return str(('Edge: {0} to {1} recordings:'.format(self.edge.A.node_num,self.edge.B.node_num),path_string))
    def data_matrix(self):
        matrix = []
        for recording in self.recordings:
            matrix.append([recording.date.hour*60+recording.date.minute,recording.time.seconds+recording.time.microseconds/10**6.])
        return matrix
# class to represent an edge in a graph
class edge:
    def __str__(self):
        return 'A: {0}, B: {1}, cost: {2}'.format(self.A.node_num,self.B.node_num,self.cost)

    def __init__(self,args):
       # print 'args: ',args

        if len(args)==2:
            self.A=args[0]
            self.B=args[1]
            self.cost=1
        else:
            if len(args)==3:
                self.A=args[0]
                self.B=args[1]
                self.cost=args[2]
            else:
                return Exception("incorrect number of parameters for initialization of edge")








def main(args):
    t = travel()
    t.initialize_test_map(0)
   # print( t.a_star(0,1))
   # print('----------------------')
   # print( t.a_star(0,4))
   # print('----------------------')
   # print( t.a_star(0,8))
   # print('----------------------')
   # print( t.a_star(0,10))
   # print(t.get_current_node())
   # print('----------------------')
   # path=t.a_star_from_current(5)
   # t.print_route(path)
   # t.follow_path(path)
   # t.visit_all_nodes()
    #t.initialize_path_times_from_nodes()

    t.load_path_times()
    tour=t.euler_tour(7)
    print('tour',tour)


    #t.visit_all_edges()
    #t.save_path_times()
    #print dist.pdist(t.path_times_list[5].data_matrix())
    #for ti in t.path_times_list:
    #   print( ti)
    #  print(ti.data_matrix())





   # t.save_path_times()


    #print(t.path_times_list)
    #for i in [1,2,3,4,5,10,15,20,25,50,100]:
    #    for j in range(10):
    #        ti = time.time()
    #        path = t.best_route_through_all_edges(0,i)
    #        ti =  time.time()-ti
    #        print( 'path length was: ', len(path),'using ',i, ' iterations and took ',ti)

    #print( 'final path is:')
   # for node in path:
    #    print(node)

   # t.travel_all_nodes_naive()


if __name__ == '__main__':
    main(sys.argv)