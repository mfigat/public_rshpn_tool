# public_rshpn_tool2

Description - folder consists of several projects (some projects are not publicly available, i.e. PetriNetEditor)
###################################################
I) PetriNetEditor project -- program with user interface - it can be utilised to model robotic systems based on Hierarchical Petri Net and automatically generate C++ code for Robot Operating System: 
1) compile PetriNetEditor using script a) or using QtCreator b)
1a) using script - enter in the terminal the following commands:
$ cd script # enter to the folder with scripts
$ bash compileHPNTool.sh <PATH_TO_QT> # compile project
e.g. bash compileHPNTool.sh ~/Programs/Qt5.9.1/5.9.1/
$ bash runHPNTool.sh # open PetriNetEditor
1b) using QtCreator
o) Open Qt Creator
o) Choose file from: PetriNetEditor/PetriNetEditor.pro
o) Build project and run
2) Open example_hpn/lwr4.rsl_pn or turtleSimulation.rsl_pn in PetriNetEditor
3) Generate code for both examples
4) Run Examples. 
4a) For turtle run in new terminal the following command:
$ rosrun turtlesim turtlesim_node 
$ # before move the turtle to specific position:
$ rosservice call /turtle1/teleport_absolute "x: 2.0 y: 2.0 theta: -3.141592265359"
$ rosservice call /clear # clear
4b) For LWR4 example follow manually (II and III) or (execute II and utilise the PetriNetTool - to generate code, compile and run the presented example)


II) LWR4_SIMULATOR
1) cd gazebo_lwr4_simulator/scripts
2) bash compile.sh # compile project
3) bash test.sh # open gazebo
4) Add within gazebo LWR4+ Manipulator, i.e. Kuka LWR

III) Execute generated package for LWR:
1) cd <PATH_TO_FOLDER>/public_rshpn_tool/generated_ros_lwr4
2) catkin_make
3) bash runAll.sh # run script running launchfile - all nodes will be launched





