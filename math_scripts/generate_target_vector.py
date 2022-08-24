#!/usr/bin/env python3

import numpy as np

def main():

    world_target = np.array(
        #   +x      +y      +z    +pitch  +roll    +yaw
        [   0,      0,      0,      0,      0,      0   ]
    )

    # Unit vector in 3d space [x, y, z] where [0, 0, -1] is "down" (gravity orientation when level)
    # If gravity vector = [0, 0, -1] the world_target and local_target will be equal
    gravity_vector = [0, 0, -1]

    

if __name__ == "__main__":
    main()