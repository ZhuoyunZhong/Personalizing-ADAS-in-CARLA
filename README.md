# Lane Changing for Autonomous Vehicles in CARLA Simulator

This project aims to design a lane changing method for autonomous vehicles with the self-driving cars simulator CARLA.

### Demonstration



### Platform

Ubuntu 18.04

[CARLA](http://carla.org/) 0.9.7

### Run

1. Make sure `numpy` and `pygame` are correctly installed

2. Put `Carla_Simulator` folder and <u>this repository</u> folder in the same folder.

3. cd to Carla_Simulator folder

   run CARLA in Low Graphic Quality 

   `  ./CarlaUE4.sh -quality-level=Lo`

   or off-screen mode

   `DISPLAY= ./CarlaUE4.sh -opengl`

4. cd to the **environment folder** of <u>this repository</u> and start the scene by

   `python run.py`
   
   or generate only the ego vehicle by
   
   `python ego_vehicle.py`
   
   
