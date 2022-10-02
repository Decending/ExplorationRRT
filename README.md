# Exploratory rapidly-exploring random tree (ERRT)

The errt folder contains the implemented ERRT framework; This includes the launch file to facilitate easier modification of the variables. The launch file contains all the internal variables and a description for the them.

The ufomap_mapping folder contains a customized ufomap mapper module, which builds an ufomap on depth one through four, given the necessary point cloud data.

# Running the framework

The errt requires the ufomap from the mapper to function properly, so to run the framework one has to launch the mapper with:
```
roslaunch ufomap_mapper server.launch
```
followed by the errt module by running:
'''
{roslaunch errt errt.launch}
'''
