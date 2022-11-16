Overview
===============

Implementation of occupancy grid mapping and SLAM in simulation. There exists many assumptions in the current implementations. The main goal is to parameterise these assumptions and to build a generalised software.

Description
====================

Occupancy Grip Mapping
------------------------

1. Considering neighborhood of a cell in the measurement model and parameterising the mapping task by its size.

2. Allowing different types of noise for sensor and odom data modeling.  

3. Based on confidence of localisation, updating the map. This might help to handle
dynamic obstacles