# Simulation

TODO: Why simulation
- Reduce in-water testing time
- Testing without hardware
- Unit tests in complex codebases using control board

## SimCB

TODO: SimCB support and FreeRTOS simulation using sockets for USB

TODO: Why? Allow testing comms without hardware and allows end users to easily incorporate a control board into their own simulated environments.

## Simulation Hijack

TODO: Simhijack and support for this in firmware (including multiple sensor stuff)

TODO: Why? Allow a simulated environment to use a control board to actually control a simulated vehicle. Allows motion testing / validation without in-water time (with a real control board). Also allows debugging / testing actual firmware without in-water time with a vehicle.

TODO: By combining with SimCB (which can also be simhijacked), motion can be tested without in-water time and without a real control board.


## Simulator Implementation & Development

TODO: Implementation details / tech docs

TODO: Docs on adding vehicles, etc


## Control Board Setups

Thus, there are four ways a "control board" can be used

- Real control board over UART
- SimCB over TCP
- Simulator models environment and provides inputs to control board and retrieves outputs from control board
- Can also do this with SimCB! Simulator is a "man in the middle"

TODO: Diagram of four modes.