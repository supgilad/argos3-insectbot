 ## Installation

Make sure you have ARGoS >= 3.0.0-beta52 installed!
Make sure you have python3 installed

 ## Building Insectbot Plugin:
```shell
./src/scripts/build_insectbot.sh
```
## Running a Simple Example
```shell
argos3 -c src/experiments/insectbot_simple.argos
```
## Running a 2 behavior Example
```shell
argos3 -c src/experiments/insectbot_two_behaviours_simple.argos
```
## Running a simple Torus Example
```shell
./src/scripts/run_insectbot_torus_example.sh
```
## Running a Custom Torus Example With 5 robots
```shell
./src/scripts/run_torus_insectbot_experiment.sh insectbot_avoider_multiple_torus 0.5 0.2 5 1
```

## Folders
1. controllers - holds robot behavior classes
2. experiments - holds all insectbot simulations
3. plugins - holds insectbot basic behaviour and simulation configurations
4. scripts - holds various scripts for using insectbot plugin 


## ARGoS
  * Architecture
    * Multi-thread, multi-process architecture
    * Robots can run different behaviors
    * Global variables can be used to contain state
  * Models
    * Models of Insectbot, other robots, boxes, cylinders
      * Motion is full 2D dynamics
      * Robots can push other objects
