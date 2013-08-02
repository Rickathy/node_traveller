#!/usr/bin/env python

#ROS dependences
import actionlib.simple_action_client
import rospy
import roslib
import actionlib
import tf
from std_msgs.msg import UInt32,Int32MultiArray
import sys
#print('\n'.join(sorted(sys.path)))

import math
import random
import time
import pickle
from datetime import datetime
import matplotlib
import matplotlib.pyplot
#from pylab import *


from travel_node import travel_node
from travel_edge import travel_edge
from a_star_node import a_star_node
from path_time import path_time
from path_times import path_times
from edge_interpreter import edge_interpreter
from clustering import clustering

import numpy, scipy



roslib.load_manifest('node_traveller')
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionGoal

# a class for travelling between nodes of a graph
class travel:

   # initializes this node
    def __init__(self):
        rospy.init_node('travel')
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.pub = rospy.Publisher("move_base",MoveBaseAction)
        self.dest_pub = rospy.Publisher("/node_traveller/dest",UInt32)
        self.route_pub= rospy.Publisher("/node_traveller/route",Int32MultiArray)
        self.listener = tf.TransformListener()             
        rospy.on_shutdown(self.end_route)
        self.graph_file = rospy.get_param("~graph", "map1.graph")
        self.path_times_list = []

    # pickle path_times for use later
    def save_path_times(self):
        pickle.dump( self.path_times_list, open( "path_times.pt", "wb" ) )

    # load the pickled path times
    def load_path_times(self):
        try:
            self.path_times_list = pickle.load( open( "path_times.pt", "rb" ) )
        except IOError:
            print('No path times were present before')
        
    #using the node list, initialize path times
    def initialize_path_times_from_nodes(self):
        edges =[]
        for node in self.nodes:
            for edge in node.connections:
                if edge  not in edges:
                    edges.append(edge)
        print('Path times initialized')
        for edge in edges:
            self.path_times_list.append(path_times(edge))
        for path_time in  self.path_times_list:
            print(path_time)

    # initializes a test map to be used for navigation testing
    def initialize_test_map(self,num):
        if num ==0:## no switch statements in python, so use nested ifs instead
            self.initialize_test_map_one()
        elif num ==1:
                self.initialize_test_map_two()

    #smaller map for testing
    def initialize_test_map_one(self):
        #start node
        node0 =travel_node(50,50,0)
        #entrance room1
        node1 =travel_node(50,49.3,1)
        node0.add_connection(node1)
        #room1
        node2 = travel_node(49.9,48.2,2)
        node3 = travel_node(49.7,46.3,3)
        node4 = travel_node(49.9,44.5,4)
        node1.add_connection(node2)
        node2.add_connection(node3)
        node3.add_connection(node4)
        #entrance room2
        node5=travel_node(49.9,50.5,5)
        node0.add_connection(node5)
        #room2
        node6=travel_node(49.9,52.1,6)
        node7=travel_node(49.8,53.7,7)
        node8=travel_node(49.8,55.3,8)
        node9=travel_node(51.7,55.1,9)
        node10=travel_node(53.4,55,10)
        node11=travel_node(53.6,52.5,11)
        node12=travel_node(51.5,52.5,12)
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

    # larger map for testing
    def initialize_test_map_two(self):
        #start node
        node0 =travel_node(50,50,0)
        #entrance room1
        node1 =travel_node(50,49.3,1)
        node0.add_connection(node1)
        #room1
        node2 = travel_node(49.9,48.2,2)
        node3 = travel_node(49.7,46.3,3)
        node4 = travel_node(49.9,44.5,4)
        node1.add_connection(node2)
        node2.add_connection(node3)
        node3.add_connection(node4)
        #entrance room2
        node5=travel_node(49.9,50.5,5)
        node0.add_connection(node5)
        #room2
        node6=travel_node(49.9,52.1,6)
        node7=travel_node(49.8,53.7,7)
        node8=travel_node(49.8,55.3,8)
        node9=travel_node(51.7,55.1,9)
        node10=travel_node(53.4,55,10)
        node11=travel_node(53.6,52.5,11)
        node12=travel_node(51.5,52.5,12)
        node5.add_connection(node6)
        node6.add_connection(node7)
        node7.add_connection(node8)
        node8.add_connection(node9)
        node9.add_connection(node10)
        node10.add_connection(node11)
        node11.add_connection(node12)
        node12.add_connection(node6)

        node7.add_connection(node12)



        #central corridor
        node13= travel_node(51.5,50,13)
        node14 = travel_node(53,50,14)
        node15=travel_node(53,49,15)
        node16 = travel_node(52.9,48,16)
        node17= travel_node(51.9,47,17)
        node18 = travel_node(51.9,45.4,18)
        node19 =  travel_node(52.8,44.3,19)
        node20 = travel_node(54,45,20)
        node21 = travel_node(53.8,46.7,21)
        node22 = travel_node(54.5,49.9,22)
        node23 = travel_node(55.9,50,23)
        node24 = travel_node(55.8,50.5,24)
        node25 = travel_node(55.9,52,25)
        node26 = travel_node(55.6,53.6,26)
        node27 = travel_node(55.6,55.1,27)
        node28 = travel_node(57.2,55.3,28)
        node29 =  travel_node(57,53.7,29)
        node30 = travel_node(57,52.1,30)
        node31 = travel_node(56,49,31)
        node32 = travel_node(56,48,32)
        node33 = travel_node(55.5,46.9,33)
        node34 = travel_node(56,46,34)
        node35 = travel_node(56,44.5,35)
        node36 = travel_node(59,50,36)
        node37 = travel_node(59,49,37)
        node38 = travel_node(59,47.6,38)
        node39 = travel_node(58.7,46,39)
        node40 = travel_node(58.6,44.5,40)
        node41 = travel_node(61.8,49.3,41)
        node42 = travel_node(61.9,45.9,42)
        node43 = travel_node(62.1,47.5,43)
        node44 = travel_node(62,49,44)
        node45 = travel_node(62,49.9,45)
        node46 = travel_node(62.2,50.8,46)
        node47 = travel_node(62.2,51.9,47)
        node48 = travel_node(63.7,51,48)
        node49 = travel_node(65.2,50.7,49)
        node50 = travel_node(65.1,49.3,50)
        node51 = travel_node(63.3,49.3,51)
        node52 = travel_node(63.3,50.2,52)
        node53 = travel_node(65.1,47.5,53)
        node54 = travel_node(64.7,46,54)
        node55 = travel_node(64.7,44.1,55)
        node56 = travel_node(66.4,44.4,56)
        node57 = travel_node(66.4,45.8,57)
        node58 = travel_node(66.5,47.5,58)
        node59 = travel_node(68.2,44.2,59)
        node60 = travel_node(68.1,46,60)
        node61 = travel_node(68.3,47.3,61)

        node0.add_connection(node13)
        node13.add_connection(node14)
        node14.add_connection(node15)
        node15.add_connection(node16)
        node16.add_connection(node17)
        node17.add_connection(node18)
        node18.add_connection(node19)
        node19.add_connection(node20)
        node20.add_connection(node21)
        node21.add_connection(node16)
        node14.add_connection(node22)
        node22.add_connection(node23)
        node23.add_connection(node31)
        node31.add_connection(node32)
        node32.add_connection(node33)
        node33.add_connection(node34)
        node34.add_connection(node35)
        node23.add_connection(node24)
        node24.add_connection(node25)
        node25.add_connection(node26)
        node26.add_connection(node27)
        node27.add_connection(node28)
        node28.add_connection(node29)
        node29.add_connection(node30)
        node30.add_connection(node25)
        node23.add_connection(node36)
        node36.add_connection(node37)
        node37.add_connection(node38)
        node38.add_connection(node39)
        node38.add_connection(node43)
        node39.add_connection(node40)
        node39.add_connection(node42)
        node40.add_connection(node41)
        node41.add_connection(node42)
        node42.add_connection(node43)
        node43.add_connection(node44)
        node44.add_connection(node45)
        node45.add_connection(node46)
        node45.add_connection(node51)
        node46.add_connection(node47)
        node46.add_connection(node52)
        node47.add_connection(node48)
        node48.add_connection(node49)
        node49.add_connection(node52)
        node49.add_connection(node50)
        node50.add_connection(node51)
        node50.add_connection(node53)
        node51.add_connection(node52)
        node53.add_connection(node54)
        node53.add_connection(node58)
        node54.add_connection(node55)
        node54.add_connection(node57)
        node55.add_connection(node56)
        node56.add_connection(node57)
        node56.add_connection(node59)
        node57.add_connection(node58)
        node57.add_connection(node60)
        node58.add_connection(node61)
        node59.add_connection(node60)
        node60.add_connection(node61)

        self.nodes = [node0,node1,node2,node3,node4,node5,node6,node7,node8,node9,node10,node11,node12,node13,node14,node15,node16,node17,node18,node19,node20,node21,node22,node23,node24,node25,node26,node27,node28,node29,node30,node31,node32,node33,node34,node35,node36,node37,node38,node39,node40,node41,node42,node43,node44,node45,node46,node47,node48,node49,node50,node51,node52,node53,node54,node55,node56,node57,node58,node59,node60,node61]
    def initialize_test_map_three(self):
        node0 =travel_node(0,0,0)
        node1=travel_node(1,1,1)
        node2=travel_node(1,0,2)
        node3=travel_node(0,1,3)
        node0.add_connection(node1)
        node0.add_connection(node2)
        node0.add_connection(node3)
        node1.add_connection(node2)
        node2.add_connection(node3)

        self.nodes=[node0,node1,node2,node3]

    # changes the robot's heading to point at a provided node, given the robot's position
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

    #returns the closest node to the robot
    def get_current_node(self):
        trans=self.get_position()
        distance = sys.maxint
        closest = 0
        for i in range(len(self.nodes)):
            if self.euclidean(trans[0][0],trans[0][1],self.nodes[i].x,self.nodes[i].y)< distance:
                distance = self.euclidean(trans[0][0],trans[0][1],self.nodes[i].x,self.nodes[i].y)
                closest = i
        self.current = closest
        print('Current closest node is {0}'.format(closest))

    # sends the robot to the specified (x,y) coordinate of the map
    # records the time taken to travel in a straight line between the nodes
    def head_to_position(self,node):

        trans=self.get_position()
        print( trans)

        #wait for the action server to be available
        move_base_client =actionlib.SimpleActionClient('move_base',MoveBaseAction)

        move_base_client.wait_for_server()

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

        # we want to use our new heading not the old one
        trans=self.get_position()
        goal.target_pose.pose.orientation.x =trans[1][0]
        goal.target_pose.pose.orientation.y =trans[1][1]
        goal.target_pose.pose.orientation.z =trans[1][2]
        goal.target_pose.pose.orientation.w =trans[1][3]

        #send the goal and wait for the base to get there
        print( "heading to node: {0} at position {1},{2} ".format(node.node_num,node.x,node.y))

        self.dest_pub.publish(node.node_num)
        time_start = datetime.today()
        result = self.move_base.send_goal_and_wait(goal)# send the goal and return whether it was completed or not
        if result is not 3:#something went wrong, 3 is success
            print('heading to node: {0} failed'.format(node.node_num))
            if not rospy.is_shutdown():
                for path_time in self.path_times_list:

                    if ((path_time.edge.A.node_num==self.current) & (path_time.edge.B.node_num==node.node_num )) | ((path_time.edge.B.node_num==self.current) & (path_time.edge.A.node_num==node.node_num )):
                        path_time.add_recording(None)
                        self.save_path_times()
            return False


        print("Goal reached.")
        '''move_base_client.send_goal(goal)
        target_reached = False
        while not target_reached:
            trans  = self.listener.lookupTransform('/map','/base_link',rospy.Time(0))
            diff1= trans[0][0]-goal.target_pose.pose.position.x
            diff2 = trans[0][1]-goal.target_pose.pose.position.y
            tot = abs(diff1)+abs(diff2)
            goal_status = move_base_client.get_state()

            if  goal_status is 3 :
                print(' status is ', goal_status)
                target_reached = True
            else:
                if goal_status is 1:
                    if tot<0.01: #threshold for how close we want to be
                        target_reached=True
                        move_base_client.cancel_goal()
                        print('Goal reached')
                        print('tot is ',tot)
                        print('path times len', len(self.path_times_list))



                       # if goal_status is not
                else:

                    if (goal_status is not 0):# 0= pending, 1= active
                                print goal_status
                                print('failed to travel to node')
                                return False '''


        for path_time in self.path_times_list:

                if ((path_time.edge.A.node_num==self.current) & (path_time.edge.B.node_num==node.node_num )) | ((path_time.edge.B.node_num==self.current) & (path_time.edge.A.node_num==node.node_num )):
                    path_time.add_recording(datetime.today() -time_start)
                    print(  'time taken {0}'.format(datetime.today() -time_start))
                    break
        self.save_path_times()
        self.current = node.node_num




        return True

    # travel through all the nodes in the order they were received (ignored connections currently)
    def travel_all_nodes_naive(self):
        for node in self.nodes:
            self.head_to_position(node)

    #generate an a star route from the current position
    # assumes the robot knows the current node it is on
    def a_star_from_current(self,end):
        return self.a_star(self.current,end,[])

    #performs an a* search between two nodes
    # the path returned assumes we are starting at the start node (so does not include it in the path)
    #includes a list of blockages
    def a_star(self,start,end,blocked):

        visited_edges = []
        visited_nodes = []
        frontier = [a_star_node(self.nodes[start],None,self.nodes[end])]
        #print('frontier')
       # print(frontier[0])
        while True:
            best= sys.maxint
            best_pos=-1
            for i in range(len(frontier)):
                #print('current best is ',best)
                #print('looking at heuristic of: ',frontier[i].heuristic)
                if frontier[i].heuristic<best:
                   # print('replaced')
                    best= frontier[i].heuristic
                    best_pos=i
            ## we have found the best node at this stage of the search
            try:
                best_node = frontier[best_pos]
            except IndexError:
                print('index error, best pos is ', best_pos)
                raise
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
                inblocked=False
                if blocked is not True:
                    for block in blocked:
                   #' print edge.A.node_num,' ',block.edge.A.node_num,' ', edge.B.node_num ,' ',block.edge.B.node_num
                        if ((edge.A.node_num==block.A.node_num)&(edge.B.node_num==block.B.node_num))|((edge.B.node_num==block.A.node_num)&(edge.A.node_num==block.B.node_num)):

                            inblocked=True

                if (edge not in visited_edges) & (not inblocked):
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
    # follow a provided route
    def follow_exploration_route(self, route, position_in_route,time_until=-1):
        m = Int32MultiArray()
        m.data=route
        self.route_pub.publish(m)
        for i in range(position_in_route,len(route)):
            print('heading to {0} of {1}'.format(i,len(route)))
            if not rospy.is_shutdown():
                if time_until >0:
                    print 'time is now',10*int((datetime.today().hour*60+datetime.today().minute)/10)
                    if 10*int((datetime.today().hour*60+datetime.today().minute)/10)>=time_until:
                        print('time is up on this run')
                        return False
                if self.head_to_position(self.nodes[route[i]]) is False:#There was a problem reaching that node
                    print('Got to node {0} in route'.format(i))
                    return i
            else:
                return False
        return True
    # follows a provided path (nodes need to be connected in the path for proper timing gathering)
    def follow_path(self, path):
        if len(path)>1:
            print('path is longer than 1:', len(path))
        for i in range(len(path)):
            print('heading to {0}'.format( path[i].node_num))
            if self.head_to_position(path[i]) is False:
                print('Failed to follow path')
                return i
        return True


    def load_graph(self):
        graph_file = open(self.graph_file, "r")
        g = pickle.load(graph_file)
        nodes = g[0]
        edges = g[1]
       # print('g')
        #print len(edges)
        self.nodes =[]
        for node in nodes:
            new_node = travel_node(node.m_coords[0],node.m_coords[1],node.id)
            self.nodes.append(new_node)
       # print len(nodes)
        print('Nodes loaded')
        for node in self.nodes:
            print(node.node_num, node.x, node.y)
        print('Edges loaded')
        for edge in edges:

                print(edge.node1, edge.node2)
                self.nodes[int(edge.node1)].add_connection(self.nodes[int(edge.node2)])

    #visit every node in the order they are saved
    def visit_all_nodes(self):
        self.get_current_node()
        route = []
        for i in range(len(self.nodes)):
            route.append(i)
        self.follow_exploration_route(route)


    # visit every edge in the map
    def visit_all_edges(self):
        self.get_current_node()
        route = self.best_route_through_all_edges(self.current,10)
       # print('route made')
      #  print(route)
        path=[]
        for node in route:
            path.append(node.node_num)
        self.follow_exploration_route(path)

    # finds all the odd nodes in the graph
    def find_odd_nodes(self,blocked,whole,position):
        odd =[]
        if blocked is True:
            for node in self.nodes:
                if len(node.connections) % 2 ==0:
                    continue
                else:
                    odd.append(node.node_num)
        else:
            for node in self.nodes:
                try:
                    self.a_star(position,node.node_num,blocked)
                except IndexError:
                    break
                count=0
                for conn in node.connections:
                    for block in blocked:
                        if ((conn.A== block.A) &( conn.B== block.B)) | ((conn.B== block.A) & (conn.A== block.B)):
                            count =1
                if (len(node.connections)-count) % 2 ==0:
                    continue
                else:
                    odd.append(node.node_num)
        return odd


    # create a matrix stating the distance between each odd node
    def create_odd_graph(self,blocked,whole ,position):
        odd_list = self.find_odd_nodes(blocked,whole,position)
        #print('odd list', odd_list)
        matrix =[]
        for start in odd_list:
            current =[]
            noneCount=0
            for end in odd_list:
                try:
                    if blocked is True:
                        length = len(self.a_star(start,end,[]))
                    else:

                        length = len(self.a_star(start,end,blocked))
                        #print('length is',length)
                except IndexError:
                    print('None')
                    length = None
                    noneCount+=1
                current.append(length)
            matrix.append(current)
        print('matrix')
        print numpy.array(matrix), odd_list
        return numpy.array(matrix), odd_list



    '''
    ' first generates a graph comprising only those nodes with odd degree
    ' then connects nodes so that they have even degree with nearest neighbours
    ' returns the last path as well, if the start or end of it are the current position they can be ignored.
    ' (maybe if any of the nodes are the start or end position they can be ignored?)
    '''
    def connected_odd_graph(self,position,blocked,whole):

        graph,nodes = self.create_odd_graph(blocked,whole,position)

        potential_shortcut=[]
        shortcut_gain=0
        visited = []
        connections = []
        additions=0
       # print('graph',graph)
       # print(len(graph))
        if(len(graph)==0):
            return []
        while additions < (len(graph)/2):
            print('addition',additions)
            lowest = sys.maxint
            pos =0

            for i in range(len(graph)):
                for j in range(len(graph[0])):
                    if (i not in visited) & (j not in visited) &  (graph[i][j]< lowest) &( graph[i][j] != 0) & (graph[i][j] is not None) :

                        lowest = graph[i][j]
                        pos = (i,j)
           # print(pos)
            if blocked is True:
                route =self.a_star(nodes[pos[0]],nodes[pos[1]],[])
            else:
                try:
                    route =self.a_star(nodes[pos[0]],nodes[pos[1]],blocked)
                except IndexError:
                    print nodes[pos[0]]
                    print nodes[pos[1]]
                    break
            connections.append((nodes[pos[0]],route[0].node_num))
            for i in range(len(route)-1):
                connections.append((route[i].node_num,route[i+1].node_num))

            additions +=1
            visited.append(pos[0])
            visited.append(pos[1])
          #  print('route print', route)
          #  for node in route:
          #      print node.node_num
          #  print('start pos ',self.nodes[position].node_num)

            if nodes[pos[0]] == self.nodes[position].node_num:

                if len(route)>shortcut_gain:
           #         'found improvement C'
                    shortcut_gain = len(route)
                    potential_shortcut= list(route)
                    potential_shortcut.insert(0,self.nodes[nodes[pos[0]]])
          #  print self.nodes[position]
          #  print 'route'
          #  print self.nodes[position] in route
          #  for node in route:
          #      print(nodes)
            if self.nodes[position] in route:

              #  print 'node in route'
                index = route.index(self.nodes[position])+1
              #  print 'index', index
                if index*2<len(route):#node is in first half of path
                  #  print 'F'
                   # print len(potential_shortcut)
                   # print shortcut_gain
                    if (len(route)-index)>shortcut_gain:
                   #     print'found improvement A'
                        potential_shortcut= list(route)
                        potential_shortcut.insert(0,self.nodes[nodes[pos[0]]])
                        shortcut_gain=index

                else:# nodes is in second half of path
                  #  print 'G'
                    if index> shortcut_gain:
                  #      print'found improvement B'
                        potential_shortcut= route
                        #potential_shortcut.insert(0,self.nodes[nodes[pos[0]]])
                        shortcut_gain=len(route)-index

        route.insert(0,self.nodes[nodes[pos[0]]])

        if len(potential_shortcut)==0:
            return connections
        short_cut_index = potential_shortcut.index(self.nodes[position])
        #print('connections', connections)
        if short_cut_index*2<len(route):
          #  print short_cut_index
            if short_cut_index==0  :
               # print('A')
                for i in range(0,len(potential_shortcut)-1):
                    connections.remove((potential_shortcut[i].node_num,potential_shortcut[i+1].node_num))
            else:
               # print('B')
                for i in range(short_cut_index,len(potential_shortcut)-1):
                    print((potential_shortcut[i].node_num,potential_shortcut[i+1].node_num))
                    connections.remove((potential_shortcut[i].node_num,potential_shortcut[i+1].node_num))
        else:
            if short_cut_index == len(potential_shortcut)-1:
               # print('C')
                for i in range(0,len(potential_shortcut)-1):
                    connections.remove((potential_shortcut[i].node_num,potential_shortcut[i+1].node_num))


            else:
               # print('D')
                for i in range(0,short_cut_index):
                  #  print((potential_shortcut[i].node_num,potential_shortcut[i+1].node_num))
                    connections.remove((potential_shortcut[i].node_num,potential_shortcut[i+1].node_num))

        #print('connections',connections)
        return connections

    # create a list of all the edges
    def copy_connections(self,blocked,position):
        graph =[]
        for node in self.nodes:
            for edge in node.connections:
                if edge.A.node_num == node.node_num : # maintains only one of each edge
                    clear = True
                    if blocked is not True:
                        for block in blocked:
                            if ((edge.A.node_num == block.A.node_num)& (edge.B.node_num == block.B.node_num))|((edge.B.node_num == block.A.node_num)& (edge.A.node_num == block.B.node_num)):
                                clear=False
                                break
                    if clear is True:
                        #print('')
                        try:
                            tour = self.a_star(position,edge.A.node_num,blocked)

                            graph.append((node.node_num,edge.B.node_num))
                        except IndexError:
                            print('error')
                            continue
        print('graph', graph)
        return graph

    #quick and dirty graph building for comparison
    def simple_edges(self,position):
        tour = []
        edges = []
        visited = []
        for node in self.nodes:
            for conn in node.connections:
                edges.append(conn)
        change = True
        pos = position
        while change is True:
            dist = sys.maxint
            A = False
            pos = 0
            change = False
            for i in range(len(edges)):
                if((edges[i].A.node_num, edges[i].B.node_num) not in visited) & ((edges[i].B.node_num, edges[i].A.node_num) not in visited):
                    distA =len(self.a_star(position,edges[i].A.node_num,[]))
                    distB = len(self.a_star(position,edges[i].B.node_num,[]))
                    if distA > distB:
                        if distA < dist:
                            dist = distA
                            A = True
                            pos = i
                            change = True
                    else:
                        if distB < dist:
                            dist = distB
                            A = False
                            pos =i
                            change = True
            route = []

            if A is True:

                route = self.a_star(position, edges[pos].A.node_num,[])
                visited.append((position,route[0].node_num))
                position = edges[pos].A.node_num
            else:
                route = self.a_star(position,edges[pos].B.node_num,[])
                visited.append((position,route[0].node_num))
                position = edges[pos].B.node_num

            for node in route:
                tour.append(node.node_num)



            for i in range(len(route)-1):
                visited.append((route[i].node_num,route[i+1].node_num))
        return tour



    # create a path visiting every node once
    def node_visit(self,position):
        tour = []
        visited = []
        for node in self.nodes:
            if node.node_num not in visited:
                route = self.a_star(position, node.node_num,[])
                for node in route:
                    tour.append(node.node_num)
                    visited.append(node.node_num)
                position = node.node_num
        return tour

    # create a path to each edge in the order they are saved/generated
    def quick_tour(self,position):
        tour =[]
        visited = []
        for node in self.nodes:
            for edge in node.connections:
                if ((edge.A.node_num, edge.B.node_num) not in visited) & ((edge.B.node_num, edge.A.node_num) not in visited):
                    routeA = self.a_star(position,edge.A.node_num,[])
                    routeB = self.a_star(position,edge.B.node_num,[])
                    route = []
                    if len(routeA )< len(routeB):
                       route = routeB
                       position = edge.B.node_num
                    else:
                        route = routeA
                        position = edge.A.node_num
                    for i in range(len(route)-1):
                        visited.append((route[i].node_num,route[i+1].node_num))
                    for node in route:
                        tour.append(node.node_num)
        return tour




    # create a tour through all edges in the graph using the idea of euler tours

    def euler_tour(self,position,blocked,visited):

        graph = self.copy_connections(blocked,position)

        odd_graph = self.connected_odd_graph(position,blocked,graph)





        for odd_edge in odd_graph:
            graph.append(odd_edge)

        print('graph after block removal',graph)

        tour = self.find_eulerian_tour(graph,position,visited,blocked)

        print('tour before duplicate removal', tour)
        tour = self.remove_duplicates_from_tour(tour)
        #tour.reverse()


        self.remove_repeat_visit(tour)

        return tour

    # find the eulerian tour of the graph
    def find_eulerian_tour(self,graph,start,visited,blocked):
        whole_tour=[]
        print('graph',graph)
        self.E = graph[:]                   # copy the graph so we don't destroy it
        First = True
        self.tour =[]
        self.find_tour(start)
        print('first tour',self.tour)
        self.tour.reverse()
        for node in self.tour:
            whole_tour.append(node)
        while len(self.E)> 0:
            self.tour = []                      # the tour starts out empty
            self.find_tour(self.E[0][0])             # find a tour using the first node in the edge list
           # print('new tour found',self.tour)
            seen=0
            for i in range(len(self.tour)-1):
                for vis in visited:
                    print vis, self.tour[i],self.tour[i+1]
                    if ((self.tour[i]==vis[0].node_num) &(self.tour[i+1]==vis[1].node_num))|((self.tour[i+1]==vis[0].node_num) &(self.tour[i]==vis[1].node_num)):
                        seen+=1
            if seen < len(self.tour)-1:
                print('sub tour', self.tour)
                print('seen is',seen)
                #print('visited is', visited[0])
                for node in self.tour:
                    whole_tour.append(node)
        #this is a check for edge cases where the tour was not a continuous path
        print('whole tour', whole_tour)
        temp=[]
        for i in range(len(whole_tour)-1):
            temp.append(whole_tour[i])
            if((whole_tour[i],whole_tour[i+1]) not in graph )&((whole_tour[i+1],whole_tour[i]) not in graph):
                temp_path = self.a_star(whole_tour[i],whole_tour[i+1],blocked)
                for node in temp_path:
                    temp.append(node.node_num)
        if temp[len(temp)-1] ==whole_tour[len(whole_tour)-1]:
            print('ends identical')
        else:
            route = self.a_star(temp[len(temp)-1],whole_tour[len(whole_tour)-1],blocked)
            for node in route:
                temp.append(node.node_num)
        print('temp',temp)

        return temp

    # find a single tour
    def find_tour(self,u):
        for (i, j) in self.E:           # find an edge with u as the source node


                    if i == u:             # check each edge going one way
                       # try:
                            self.E.remove((i, j))   # remove the found edge so it isn't used twice
                            self.find_tour(j)       # continue the tour from the sink node
                        #except ValueError:
                        #    continue
                    elif j == u:           # check each edge going the other way if we need to
                        #try:
                            self.E.remove((i, j))
                            self.find_tour(i)
                        #except ValueError:
                        #    continue

        self.tour.append(u)             # we found an edge from u, so its part of the tour

    # cut out any duplicates from a tour
    def remove_duplicates_from_tour(self,tour):
        new_tour=[]
        for i in range(len(tour)-1):
            if tour[i]!= tour[i+1]:
                new_tour.append(tour[i])
        if tour[len(tour)-1]!= tour[(len(tour))-2]:
            new_tour.append(tour[len(tour)-1])
        return new_tour

    # Begins at the end of the tour going backwards, removing any edges that were visited previously and thus are no longer needed (stops once it doesnt remove a edge)
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


    # set the robot's start position
    def set_start(self,start_pos, tour):
        while tour[0] != start_pos:
            tour.append(tour.pop(0))
        return tour

    def graph_edge(self, pos,show=False):
        x= []
        y=[]
        for i in range(len(self.path_times_list[pos].recordings)):
            x.append(self.path_times_list[pos].recordings[i].date.minute+self.path_times_list[pos].recordings[i].date.hour*60)
            y.append(self.path_times_list[pos].recordings[i].time.seconds+self.path_times_list[pos].recordings[i].time.microseconds/10**6.)
        fig = matplotlib.pyplot.figure()
        ax = fig.add_subplot(110)
        print('info on the edge traversals')
        print(x)
        print(y)


        ax.set_title('Edge traversals for edge between node {0} and node {1}'.format(self.path_times_list[pos].edge.A.node_num,self.path_times_list[pos].edge.B.node_num))
        ax.set_xlabel('Time of day (minutes from midnight)', fontsize=20)
        ax.set_ylabel('Time taken (seconds)', fontsize=20)
        ax.scatter(x,y)
        if show:
            matplotlib.pyplot.show()
    def graph_path_dates(self,show=False):
        
        x =[]
        y=[]
        for i in range(len(self.path_times_list)):
            for recording in self.path_times_list[i].recordings:
                x.append(i)
                try:
                    y.append(recording.date.hour*60+recording.date.minute)
                except AttributeError:
                    y.append(0)
        print(x)
        print(y)
        print(len(x))
        print(len(y))

        fig = matplotlib.pyplot.figure()
        ax = fig.add_subplot(111)
        ax.set_title('Edge Traversals')
        ax.set_xlabel('Edge Index', fontsize=20)
        ax.set_ylabel('Time of Day Recorded (Minutes After Midnight)', fontsize=20)
        ax.scatter(x,y)
        if show:
            matplotlib.pyplot.show()
    def graph_path_times(self,show):

        x =[]
        y=[]
        for i in range(len(self.path_times_list)):
            for recording in self.path_times_list[i].recordings:
                x.append(i)
                try:
                    y.append(recording.time.seconds+recording.time.microseconds/10**6.)
                except AttributeError:
                    y.append(0)
        print(x)
        print(y)
        print(len(x))
        print(len(y))

        fig = matplotlib.pyplot.figure()
        ax = fig.add_subplot(111)
        ax.set_title('Edge Traversals')
        ax.set_xlabel('Edge Index', fontsize=20)
        ax.set_ylabel('Time Taken (Seconds)', fontsize=20)
        ax.scatter(x,y)
        if show:
            matplotlib.pyplot.show()


    def naive_run(self):
        blocked = []

        tour = self.euler_tour(self.current,True,[])
        print('Tour generated: ')
        print(tour)
        
        result =  self.follow_exploration_route(tour,0)
        while result is not True:#something went wrong in the run. get to the next node, avoiding the blocked one and try to carry on
            if not rospy.is_shutdown():
                print('heading back to previous node')
                self.head_to_position(self.nodes[tour[result-1]])##attempt to head back to the node we were at previously
                visited =[]
                for j in range(1,result-1):#create a list of all the nodes visited
                    visited.append((self.nodes[tour[j-1]],self.nodes[tour[j]]))
                print('visited is: {0}'.format(visited))
                
                for path_time in  self.path_times_list:# add the blocked node to the list of blocked nodes
                    if ((path_time.edge.A == self.nodes[tour[result-1]]) & (path_time.edge.B == self.nodes[tour[result]]))|((path_time.edge.B == self.nodes[tour[result-1]]) & (path_time.edge.A == self.nodes[tour[result]])):
                        blocked.append(path_time.edge)
                        print('blocked being added to',path_time.edge.A, ', ',path_time.edge.B)
                        break
                print('blocked is: {0}:'.format(blocked))
                tour = self.euler_tour(self.current,blocked,visited)
                
                result = self.follow_exploration_route(tour,0)
            else:
                print('Run cancelled, saving path times')
                self.save_path_times()
                return False


            #result = self.follow_exploration_route(tour,result)


        print('Run finished. Path times are:')
       # for pt in self.path_times_list:
        #    print(pt)
        self.save_path_times()
        m = Int32MultiArray()
        m.data=[]
        self.route_pub.publish(m)


    def set_up_for_test(self):
        self.load_graph()
        self.initialize_path_times_from_nodes()
        self.current=0
    def set_up_for_run(self):
        self.load_graph()

        self.initialize_path_times_from_nodes()

        self.get_current_node()
    def graph(self,x,y,title,xlabel,ylabel,show):
        fig = matplotlib.pyplot.figure()
        ax = fig.add_subplot(111)
        ax.set_title(title)
        ax.set_xlabel(xlabel, fontsize=20)
        ax.set_ylabel(ylabel, fontsize=20)
        ax.scatter(x,y)
        
        if show:
            matplotlib.pyplot.show()
        matplotlib.pyplot.close()
    def graph_range(self,partitions,times,title,show=False):
        
        x =[]
        y=[]
        for i in range(len(times)):
            x.append((partitions[i]+partitions[i+1])/2)
            y.append(times[i])
        self.graph(x,y,title,'Time of day','Expected Time Taken (Seconds)',show)
    def gather_data_forever(self):
         self.set_up_for_run()
         self.load_path_times()
         while not rospy.is_shutdown():
            self.naive_run()
            self.save_path_times()
         self.graph_path_times(True)
    def gather_data_entropy_clusters(self):
        self.set_up_for_run()
        self.load_path_times()
        current_time = 10*int((datetime.today().hour*60+datetime.today().minute)/10)
        print('current time',current_time)
        tour = self.generate_route_entropy_clusters([],current_time)
        print('next time is', current_time+10)
        result = self.follow_exploration_route(tour,0,current_time+10)
        blocked=[]
        while not rospy.is_shutdown():
            while result is not True:#something went wrong in the run. get to the next node, avoiding the blocked one and try to carry on
                if not rospy.is_shutdown():
                    print('heading back to previous node')
                    self.head_to_position(self.nodes[tour[result-1]])##attempt to head back to the node we were at previously
                    for path_time in  self.path_times_list:# add the blocked node to the list of blocked nodes
                        if ((path_time.edge.A == self.nodes[tour[result-1]]) & (path_time.edge.B == self.nodes[tour[result]]))|((path_time.edge.B == self.nodes[tour[result-1]]) & (path_time.edge.A == self.nodes[tour[result]])):
                            blocked.append(path_time.edge)
                            print('blocked being added to',path_time.edge.A, ', ',path_time.edge.B)
                            break
                    print('blocked is: {0}:'.format(blocked))
                    new_time =10*int((datetime.today().hour*60+datetime.today().minute)/10)
                    if new_time != current_time:
                        current_time=new_time
                        blocked=[]
                    route = self.generate_route_entropy_clusters(blocked,current_time)
                    result = self.follow_exploration_route(route,0,new_time)
                else:
                    print('Run cancelled, saving path times')
                    self.save_path_times()
                    return False
            ### we have finished a run, and have some time until the next one should start, this means we can position ourself at the best place to start the next run!
            next_node = self.get_first_node_clusters(current_time+10)
            route = self.a_star_route(self.current,next_node,[])
            print('heading to node for next time')
            result =self.follow_exploration_route(route)
            if result== True:
                print('next node reached')
            else:
                print('route to next node blocked, I\'ll wait here')
            #rospy.sleep(((current_time+10)-(datetime.today().hour*60+datetime.today().minute))*60)
        print('Run finished. Path times are:')
        #for pt in self.path_times_list:
         #   print(pt)
        self.save_path_times()
        m = Int32MultiArray()
        m.data=[]
        self.route_pub.publish(m)
        
    def get_first_node_clusters(self,next_time):
        entropies = self.generate_entropies()
        current_index = next_time/10
        current_entropies =[]
        print len(entropies)
        print len(entropies[0])
        for i in range(len(entropies)):
           current_entropies.append(entropies[i][current_index])
        import operator
        sorted_entropies= sorted(enumerate(current_entropies), key=operator.itemgetter(1))
        return sorted_entropies[0][0]
    def generate_route_entropy_clusters(self,blocked,current_time):
        
        entropies = self.generate_entropies()
        current_index = current_time/10
        current_entropies =[]
        print len(entropies)
        print len(entropies[0])
        max_count=0
        count=0
        print('current time',current_time)
        print current_index
        for i in range(len(entropies)):
            if entropies[i][current_index] == sys.maxint :
                max_count+=1
            count+=1
            
            current_entropies.append(entropies[i][current_index])
        print('{0} of {1} edges had max entropy'.format(max_count,count))    
        
        if max_count == count:# we've got only max entropies, the graph hasn't been explored at this time yet
            
            return self.euler_tour(self.current,[],[])
            
        import operator
        sorted_entropies= sorted(enumerate(current_entropies), key=operator.itemgetter(1))
        print current_entropies
        
        seen=[]
        tour = []
        position = self.current
        sorted_entropies.reverse()
        print sorted_entropies
        for (index, entropy) in sorted_entropies:
            print('index {0}'.format(index))
            used_before=False
            for (A,B) in seen:
                
                if ((self.path_times_list[index].edge.A.node_num==A.node_num )& ( self.path_times_list[index].edge.B.node_num==B.node_num)) | ((self.path_times_list[index].edge.A.node_num==B.node_num) & (self.path_times_list[index].edge.B.node_num==A.node_num)):
                    used_before=True
                    break
            
            if used_before==False:
               
                routeA = self.a_star(position, self.path_times_list[index].edge.A.node_num, blocked)
                routeB = self.a_star(position, self.path_times_list[index].edge.B.node_num, blocked)
                #print 'len(A)',len(routeA)
                #print 'len(B)',len(routeB)
                if len(routeA)>0:
                    if len(routeA)< len(routeB):
                        route = routeA
                        route.append(self.path_times_list[index].edge.B)
                        #print 'A<B'
                    else:
                        if len(routeB)>0:
                            #print 'A>B'
                            route=routeB
                            route.append( self.path_times_list[index].edge.A)
                        else:
                            
                            route.append(self.path_times_list[index].edge.A)
                else: 
                    #print 'A=0'
                    #route=routeB
                    route.append( self.path_times_list[index].edge.B)
                
                #print route
                print('new part')
                for node in route:
                    print(node.node_num)
                    
                    
                print 'length', len(route)
                
                position = route[len(route)-1].node_num
                for node in route:
                    tour.append(node)
                for i in range(len(route)-1):
                    seen.append((route[i],route[i+1]))
                
        temp =[]
        for node in tour:
            temp.append(node.node_num)
        return temp
        
        
    def generate_entropies(self):
        entropies=[]
        e = edge_interpreter()
        for edge in range(len(self.path_times_list)):
            #print e.interpret_clustering_ten_minutes(self.path_times_list[edge].recordings,True)
            entropies.append(e.interpret_clustering_ten_minutes(self.path_times_list[edge].recordings,True)[2])
        return entropies
        
    def gather_data_once(self):
         self.set_up_for_run()
         self.load_path_times()
         self.naive_run()
         self.save_path_times()
         self.graph_path_times(True)
         
    def end_route(self):        
        m = Int32MultiArray()
        m.data=[-1]
        self.route_pub.publish(m)
        

def main(args):
    t = travel()
   # t.gather_data_forever()
    t.gather_data_entropy_clusters()


if __name__ == '__main__':
    main(sys.argv)
