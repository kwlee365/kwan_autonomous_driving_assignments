# RRT python assignment :

## Topics Intelligent Convergence Systems (2023) 
%%% That code belongs to the DYROS, Seoul National University.
%%% This code is written for the class material.

## Dependencies :
- Python 3.8, other versions might cause compatibility issues! (spyder recommended)
- scikit-learn
- matplotlib
- numpy
- reeds-shepp (https://github.com/ghliu/pyReedsShepp)
- ffmpeg (optional if you want to visualize the car that tracks the path, '$sudo apt-get install ffmpeg')

This code is a rapidly-exploring random tree (RRT) implementation code for a 2D non-holonomic robot (such as a car).
It consists of two algorithms; RRT and the optimal variant RRT (RRT*).
The assignment basically consists of filling the empty functions and analysing the results (see the questions below).
Additional points may be given if you optimize your code (hint: the reeds shepp distance is always >= to the euclidian distance).

## How to run?
In a terminal, run 'main.py'.
Also, depending on each homework, uncomment the line 198~201.

## Brief descriptions
Line 16~18: About visualization. 
Line 20~32: Parameters for RRT* and environments. You don't need to modify this part to do the homeworks.
Line 35~40: Initialize the environment that you are working on. You don't need to modify this part to do the homeworks as well. Later, if you want to study your own RRT algorithm, you may change a variable and test your algorithm in different environments.
