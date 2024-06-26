# Research Track 2 Assignments
This README will give an overview of the Research Track 2 Assignments. In total there will be 3 Assignments in the continous evaluation of the course.

## Assignment 1: Documentation of Research Track 1 second assignment
In order to properly document the second assignment of Research Track 1 course the Sphynx (documentation for software projects) is used because the software language used is python. Sphynx was originally created for Python and therefore better supports the documentation of this project.

The documentation of Research Track 1 assignment can be found here: https://irislaanearu.github.io/RT2_assignments/

## Assignment 2: Using widgets and data visualization for the Research Track 1 Second Assignment
In this assignment the author:
- created a jupyter notebook to replace the user interface (node A) made for the RT1 course
- used widgets to let the user know the position of the robot and all targets that have been set and cancelled in the environment
- used FuncAnimation to plot the robot position
- used FuncAnimation to plot the set and cancelled targets in the environment

In the end, the notebook has:
- an interface to assign (or cancel) goals to the robot
- live info about the goal status
- a plot with the robot´s position and target´s positions in the environment
- a plot for the number of set and cancelled targets

First download the ROS package __assignment_2_2023__ to your ROS workspace __src__ folder.

To use the created jupyter notebook follow these steps:
1. In terminal download Jupyter notebook using the following command:
```bash
$ pip install jupyter
```
2. Launch the Jupyter Notebook server:
```bash
$ jupyter notbeook
```
3. Move to the ROS package __assignment_2_2023__ and choose the notebook in the folder __scripts__ with the name __NodeA_jupyter.ipynb__.

4. To use the jupyter notebook __NodeA_jupyter.ipynb__ go back to the __src__ folder in your terminal and run the launch file to start the Research Track 1 assignment 2:
```bash
$ roslaunch assignment_2_2023 assignment1.launch
```
5. Now compile and run the notebook with widgets to set goals, cancel goals, get the robot position and all the set and cancelled targets in the robot environment.

## Assignment 3: Statistical Analysis of the Research Track 1 First Assignment 
The report for the statistical analysis of the assignment is named _RT2_Assignment_3.pdf_. Two algorithms were tested in the simulation environment and the results analysed. The report is composed of hypothesis made, description of the experimental setup, results, statistical analysis and conclusion.


