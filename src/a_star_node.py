#!/usr/bin/env python
import travel_edge
import math
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
