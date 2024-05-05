# Simulation

## SimCB

SimCB is the name used for the control board firmware built as a binary to run on a PC operating system (Windows, macOS, Linux). This is achieved using the FreeRTOS windows or posix ports. Instead of using UART over USB (as real hardware does), SimCB opens TCP sockets (server) for communication. The code using the control board would then connect to this socket (as a client) instead of opening a UART port to talk with a physical control board. The communication protocol (message structure and format) is exactly the same as described for UART.

The SimCB allows a control board to run without hardware. The same firmware that would run on the board instead runs on your PC. There are a few limitations with this. SimCB will only operate in simhijack mode (it cannot be released from simhijack) and only the sim sensors will be available.

TODO: Details on where this code lives, how TCP is handled and why.

## Simulation Hijack

TODO: Simhijack and support for this in firmware (including multiple sensor stuff)

TODO: Why? Allow a simulated environment to use a control board to actually control a simulated vehicle. Allows motion testing / validation without in-water time (with a real control board). Also allows debugging / testing actual firmware without in-water time with a vehicle.

TODO: By combining with SimCB (which can also be simhijacked), motion can be tested without in-water time and without a real control board.


## Simulator Implementation & Development

The simulator is implemented using the Godot game engine (3.2). This provides a 3D rendering and physics engine.

[GodotAUVSim on GitHub](https://github.com/MB3hel/GodotAUVSim)

This simulator was initially designed to allow control board firmware development without in-water testing time with a physical vehicle. It has expanded into a reference simulator for end user use. It does not model any environment, just the vehicle(s).

TODO: Implementation details / tech docs

TODO: Docs on adding vehicles, etc


## Control Board Setups

Thus, there are four ways a "control board" can be used

- Real control board over UART
- SimCB over TCP
- Simulator models environment and provides inputs to control board and retrieves outputs from control board
    - Can also do this with SimCB. Simulator is a "man in the middle"
