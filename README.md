# DroneControl

A package to connect to and control multiple drones.

## Installation
1.  windows, you will also have to download the appropriate MAVSDK release (see https://mavsdk.mavlink.io/v2.0/en/cpp/guide/installation.html) and extract the mavsdk-server-bin.exe file into the same directory as drone.py.

2. To install this package, simply clone this repository, move into the root directory and then install with pip:
```
pip install --upgrade pip
pip install -e .
```

3. As part of the installation a command called ```dm``` is installed, which starts the terminal interface. 
Alternatively you can run the app.py script.

### API
Coming SOON>>

## Branch
Main is for the Dronemanager
Master for the Frontend in React.js

## Test
1. Activate PX4 : make px4_sitl gz_x500 (install PX4 Gazebo) - if using dummy data Run : python udp_sender2.py
2. Run DM and activate the drone
3. Run : python udp_transporter.py
4. Run locally or deploy the Frontend


## License
For open source projects, say how it is licensed.

## Project status
If you have run out of energy or time for your project, put a note at the top of the README saying that development has slowed down or stopped completely. Someone may choose to fork your project or volunteer to step in as a maintainer or owner, allowing your project to keep going. You can also make an explicit request for maintainers.
