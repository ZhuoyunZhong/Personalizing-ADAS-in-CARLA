# Assistive-Driving-in-CARLA

This project aims to design a lane changing method for autonomous vehicles with the self-driving cars simulator CARLA.

### Demonstration


### Platform

Ubuntu 18.04
[CARLA](http://carla.org/) 0.9.8

#### Carla Installation Steps
Add the CARLA 0.9.8 repository to the system.
```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 304F9BC29914A77D &&
sudo add-apt-repository "deb [arch=amd64 trusted=yes] http://dist.carla.org/carla-0.9.8/ all main"
```

Install CARLA and check for the installation in the /opt/ folder.

```
sudo apt-get update
sudo apt-get install carla
cd /opt/carla
```

### Run

1.  `numpy` and `pygame` should be correctly installed

2. Put **Carla_Simulator** folder and **this repository** folder in the same folder.

3. cd to Carla_Simulator folder

   run CARLA in Low Graphic Quality 

   `  ./CarlaUE4.sh -quality-level=Low`

   or off-screen mode

   `DISPLAY= ./CarlaUE4.sh -opengl`

4. cd to the **environment folder** of **this repository** . There are options for generating vehicles.

   To generate only the ego vehicle
   
   `python ego_vehicle.py`
   
   To generate scene 1 where one vehicle is in front of the ego vehicle
   
   `python run.py -s 1`
   
   To generate scene 2 where multiple vehicles are surrounding the ego vehicle
   
   `python run.py -s 2` 
   
5.  The vehicle is initialized with autopilot mode **on**. To switch to manual control, press `p`. To start or end the learning process, press `l` and drive the vehicle. After learning, it requires to regenerate the scene in order to see the learning result.
