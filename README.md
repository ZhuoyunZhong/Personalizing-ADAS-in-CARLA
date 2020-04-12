# Assistive-Driving-in-CARLA

This project aims to design a lane changing method for autonomous vehicles with the self-driving cars simulator CARLA.

## Demonstration

![Demo](./demo/default_model.gif)


## Platform

Ubuntu 18.04
[CARLA](http://carla.org/) 0.9.8

## Carla Simulator Installation
Go to official [Carla Releases](https://github.com/carla-simulator/carla/releases) Github page.

Download **[Linux] CARLA 0.9.8**, unzip the file and rename the folder as **CARLA_Simulator**. Additional maps are optional.

Put **CARLA_Simulator** folder and **this repository** folder in the same folder. So the folder structure should look like:

```
Carla                              
├── Assistive-Driving-in-CARLA     # This repository
│   ├── agents                     # Self-driving and Personalization module
│   │   ├── ...
│   ├── data                       # Useful data
│   │   ├── ...
│   ├── demo                       # Demonstration for README
│   │   ├── ...
│   ├── environment                # Scenes and different scenes set up
│   │   ├── ...
│   ├── LICENSE                    
│   ├── README.md
└── CARLA_Simulator                # Carla Simulator of suggested version    
    ├── ...                    
```

## Run

1.  `numpy`, `scikit-learn` and `pygame` should be correctly installed

2. Go to **CARLA_Simulator** folder,

   `cd Carla/CARLA_Simulator` 

   run CARLA in Low Graphic Quality

   `./CarlaUE4.sh -quality-level=Low`

   or off-screen mode.

   `DISPLAY= ./CarlaUE4.sh -opengl`

3. Go to the **environment folder** of **this repository**. 

   `cd Carla/Assistive-Driving-in-CARLA/environment `

   There are options for generating different scenes. To generate only the ego vehicle:

   `python ego_vehicle.py`

   To generate scene 1 where one vehicle is in front of the ego vehicle:

   `python ego_vehicle.py -s 1`

   To generate scene 2 where multiple vehicles are surrounding the ego vehicle:

   `python ego_vehicle.py -s 2` 

4. The vehicle is initialized with autopilot mode **on**. To switch to manual control, press `p`. To start or end the learning process, press `l` and drive the vehicle. After learning, it requires to regenerate the scene in order to see the learning result (for now).
