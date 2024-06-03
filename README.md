## This is the repository of group 17 from the Multi Disciplinary Projects DARP
In this document, all instructions to build the code and explanation about the implementation can be found.

The code has been adapted originally by Felipe Bononi Bello (fbononibello@gmail.com) from group 13, isolating the code from the rest of the cpswarm package and allowing stand-alone execution. Ming Da Yang (m.d.yang@student.tudelft.nl) from group 15 contributed to the repository by refactoring the code to allow an arbitrary amount to be used to divide the area and generate that amount of paths. We group 17 have implemented a listener for the occupancygrid, and integrated group 15s multiple robots to determine the amount and planning and starting locations using tf messages.

### Path planning

To run the path planning section one must create a mirte workspace in which you clone this repository inside of the `src` folder.
```
source devel/setup.bash
catkin_make
```
To launch the seperate packages (area_division and coverage_path) write the following commands in different terminal windows.

```
roslaunch area_division area_division.launch
```

```
roslaunch coverage_path coverage_path.launch
```

To launch both the area_division and coverage_path packages write the following commands:

```
roslaunch area_division area_path.launch
```

Currently you have to hard code the amount of robots by changing the `num_robots` integer varable in both the packages source files `coverage_path.cpp main.cpp` and change the robot locations in `main.cpp` of the area_division package. Make sure that the amountof robot locations in `main.cpp` is equal to `num_robots` otherwise the code will not work.



<!-- As input publish to topic `area_division/robots' -->

