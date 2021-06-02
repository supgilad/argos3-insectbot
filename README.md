# Compiling the code

Make sure you have ARGoS >= 3.0.0-beta52 installed!

Make sure you have python3 installed

 ## Building Insectbot Plugin:
```shell
./build_insectbot.sh
```
## Running a Simple Example
```shell
argos3 -c src/examples/experiments/insectbot_simple.argos
```
## Running a 2 behavior Example
```shell
argos3 -c src/examples/experiments/insectbot_simple.argos
```

## Running a Torus Example
```shell
./run_generated_insectbot_experiment.sh insectbot_multiple 0.5 0.2 10 1
```


## ARGoS
  * Architecture
    * Multi-thread, multi-process architecture
    * Robots can run different behaviors
    * Global variables can be used to contain state
  * Models
    * Models of Insectbot, other robots, boxes, cylinders
      * Motion is full 2D dynamics
      * Robots can push other objects
