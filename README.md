# AVDASI 2 Team 4

The application will be divided into 2 parts: 
- Controller side
- Aircraft side

Each of the parts will be run from separate Teensy micro-controllers and have separate sketches (.ino files) and will be located in different folders.

## Controller

The code for the custom built controller (comms with the aircraft, elevator control) running on Teensy 4.0 micro-controller. The code will reside in the *controller* folder. 

There should be only a single sketch and should be named *controller.ino*. All dependencies should either reside in an external *libs* folder or be split into separate *header (.h)* and *c++ (.cpp)* files.

## Aircraft

The code for the aircraft system (on-board sensing, comms, data storage, actuation, on-board algorithms) running on Teensy 4.1 micro-controller. The code will reside in the *aircraft* folder. 

There should be only a single sketch and should be named *aircraft.ino*. All dependencies should either reside in an external *libs* folder or be split into separate *header (.h)* and *c++ (.cpp)* files.