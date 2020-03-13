# Assistive-Driving-in-CARLA

This project aims to design a lane changing method for autonomous vehicles with the self-driving cars simulator CARLA.

### Demonstration


### Platform

Ubuntu 18.04

[CARLA](http://carla.org/) 0.9.7

### Run

1.  `numpy` and `pygame` should be correctly installed

2. Put **Carla_Simulator** folder and **this repository** folder in the same folder.

3. cd to Carla_Simulator folder

   run CARLA in Low Graphic Quality 

   `  ./CarlaUE4.sh -quality-level=Lo`

   or off-screen mode

   `DISPLAY= ./CarlaUE4.sh -opengl`

4. cd to the **environment folder** of **this repository** . There are options for generating vehicles.

   To generate only the ego vehicle
   
   `python ego_vehicle.py`
   
   To generate scene 1 where one vehicle is in front of the ego vehicle
   
   `python run.py -s 1`
   
   To generate scene 2 where multiple vehicles are surrounding the ego vehicle
   
   `python run.py -s 2` 
   
   



...
