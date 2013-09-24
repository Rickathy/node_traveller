node_traveller
======

ROS code to navigate a robot around a graph

======

In order to run it, there are a few packages that need to be installed (as well as the ROS dependencies)

1. Hcluster, this is for clustering : https://code.google.com/p/scipy-cluster/ explains how to install this.

2. Exploration ROS Stack: this can be checked out from http://svn.code.sf.net/p/bosch-ros-pkg/code/trunk/stacks/exploration

3.  libbullet-dev may also need to be installed for the running of the exploration stack, that can be installed with the standard sudo apt-get install libbullet-dev

======

####Real world runs Usage:

Run the program using the command line below, replacing **FILEPATH** by the path to the *.graph* file that you want to work with (keep the quotes).
>rosrun node_traveller travel.py mode _graph:="FILEPATH"

Alternatively, you can omit the *_graph* parameter, and the program will use the default graph file (as of now, it is *map1.graph*)
>rosrun node_traveller travel.py mode

The modes can be selected from the following:

0. The naive mode is used, this will just explore without giving any preference to edges using previously gained knowledge.

1. Entropy of edges is used to decide which to examine first. Days of the week are ignored.

2. The confidence interval of an estimate is used. Days of the week are ignored

3. Entropy of edges is used to decide which to examine first. Days of the week are used in addition to times of day.

4. The confidence interval of an estimate is used. Days of the week are used in addition to the time of day.

=====

####Graph generation

A number of different graphs can be generated from the data, command line arguments are not used and instead the code must be used directly. node_traveller/src/test.py has a large amount of these commented out for examination. 

The main method is as follows: 
>t=travel()
>t.set_up_for_tests()

if you want to see all the recordings that have been made (edges against the duration of traversal):
>t.graph_path_times()

if you want to see all the recordings that have been made (edeges against time of recording):
>t.graph_path_dates()

if you want to show the recordings on edge N:
>t.graph_edge(N)

if the whole system uncertainty is required at a specific time:
>t.graph_each_edge_entropy_at_time(epoch_index)
or
>t.graph_each_edge_confidence_at_time(55)
Where the epoch_index is the number of minutes since midnight divided by 10.

To get the entropy over time of a specific edge:
>t.graph_edge_entropy(edge_index)

It is also possible to temporarily remove entries past a specific date:
>t.temp_remove_records_after(day,month,year)
This is useful for seeing the system at a time in the past.

If any more specific interpretation is needed, an additional class is required:
>e = edge_interpreter()
>times = e.interpret(t.path_times_list(N),mode)

Where the modes are:
0. Average
1. Hourly average
2. ten minute averages
3. clustering is used
4. exponential distribution is used
Will return a list consisting of first the time epochs that each estimate refers to, followed by the estimates themselves (the estimate at position N, is for the time epoch between epoch N and epoch N+1) and the third element is a list of all the uncertainties generated for this method (using the same convention as estimates for what epoch they refer to)
Additionally 
>times = e.interpret(t.path_times_list(N),mode,True)
For modes 3 and 4 will take into account the days of the week. The time is handled by the first 1440 minutes being day 0, the second 1440 minutes being day 1 and so on.

####Simulation

The simulation is ran by using
>rosrun node_traveller sim.py from_file week_long_start

If from_file is 0, the system will generate new data, 1 is for when the data is already generated and graphs are being reproduced (or different graphs are being produced)
If week_long start is 0, then the system will do a week of unguided exploration, followed by a week of guided exploration. If it is 1, a single day of unguided exploration is followed by 13 days of guided exploration.

Running a simulation can take up to around 2 hours, this is because of the high number of evaluations done on the models as well as the number of models used.


