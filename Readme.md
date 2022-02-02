# Ground Removal Experiment

## Aim

In order to optimise algorithm performance, we need to determine which paramaters will best suit our needs. To facilitate this, this experiment program written in C++ will help to collect algorithm performance data for different parameter sets which we can then plot to determine optimal patameters.

## Point Categorization

We will assign all points in a frame to one of three categories:

1. Ground
2. Cone
3. Unassigned

A C++ program will be written using pcl for assigning points to categories. The program will display a point cloud to the user in 3D space, which the user should be able to navigate around. The program will allow the user to select different points and categorize them accordingly. Once the user has finished selecting points, the user will be able to save the assignments to a .pcd file.

For convenience, the file will contain all of the original point data with additional colour data which will represent the category of each individual point.

- White = Unassigned (default)
- Red = Cone
- Green = Ground

## Algorithm Simulation

A C++ program using pcl will be written to run the ground removal algorithm with different parameter sets. The program will then output the data in .csv format so that it can be plotted. The program will allow the user to specify a number of .pcd files with colour assignments to perform the experiment.

A configuration file will be provided to allow the user to specify various parameters for the experiment. A serialized copy of the parameters used in a configuration will be exported alongside the experimental data.

The parameters that the user will be able to change include:

- Minimum/Maximum per parameter
- Iterations per parameters
- Incremental step per parameter
- Baseline set – A parameter set which will be used as a baseline whenever an individual parameter is varied.

To run the experiment, each algorithm variable will be altered individually and the algorithm will then be tested on each of the .pcd files provided. The results compiled will contain the average percentage share for each category from the points removed from the cloud, ignoring unassigned points. The data will also contain the average total number of points removed as well as the average percentage of the total of each category which was removed. The optimal algorithm will have a high percentage of ground points removed and a minimal percentage of cone points removed.

## Analysing the Data

A program written in Python using matplotlib will consume the csv files produced by the simulation program. This program will provide various different plots to display the data produced.

Plots that will be displayed include:

- Each individual parameter against the percentage breakdown of the categories of the points removed.
- Each individual parameter against the average number of points removed across each frame.
