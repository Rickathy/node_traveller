#!/usr/bin/env python
from travel_edge import travel_edge

class travel_node:
    # it's just an x and y coordinate
    def __init__ (self,x,y,node_num):
        self.x=x
        self.y=y
        self.connections= []
        self.node_num=node_num
        # add an edge between this node and another
    def __eq__(self,other):
        if (self.x== other.x) & (self.y==other.y):
            return True
        else:
            return False
    def add_connection(self,node):
        connection = travel_edge((self,node))
        self.connections.append(connection)
        node.connections.append(connection)
    def __str__(self):
        return str('node number is {0}'.format(self.node_num))
