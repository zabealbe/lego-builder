# Lego Builder
Lego builder is an experimental playground for manipulating Lego blocks.
The goal is to be able to create a structured assembly starting from a
sparse and randomly oriented set of blocks.\
\
To reach our goal we had to implement a few features:
+ A manipulation system with great dexterity and precision
+ A computer vision system to infer exact models position and orientation

## Kinematics
Assignment 1, 2 and 3 can be found [`here`](./src/lego_builder/lego_builder_kinematics/main.ass3.py)\
Assignment 4 can be found [`here`](./src/lego_builder/lego_builder_kinematics/main.ass4.py)

## Vision
The real-time vision pipeline can be found [`here`](./src/lego_builder/lego_builder_vision/main.py)\
The custom dataset farming toolset can be found [`here`](./src/lego_builder/lego_builder_dataset_farmer)
