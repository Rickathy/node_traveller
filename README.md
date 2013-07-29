node_traveller
======

ROS code to navigate a robot around a graph

======

In order to run it, there are a few packages that need to be installed (as well as the ROS dependencies)

1. Hcluster, this is for clustering : https://code.google.com/p/scipy-cluster/ explains how to install this.

2. Exploration ROS Stack: this can be checked out from http://svn.code.sf.net/p/bosch-ros-pkg/code/trunk/stacks/exploration

3.  libbullet-dev may also need to be installed for the running of the exploration stack, that can be installed with the standard sudo apt-get install libbullet-dev

======

####Usage:

Run the program using the command line below, replacing **FILEPATH** by the path to the *.graph* file that you want to work with.
>rosrun node_traveller travel.py _graph:=\"**FILEPATH**\"

Alternatively, you can omit the *_graph* parameter, and the program will use the default graph file (as of now, it is *map1.graph*)
>rosrun node_traveller travel.py
