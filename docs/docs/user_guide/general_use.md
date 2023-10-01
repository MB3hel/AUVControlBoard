# Using Control Board

TODO

- General Procedure
    - Connecting to Control Board
    - System configuration (motor matrix, thruster inversions)
    - Sensor configuration
    - Validating sensor connectivity
    - Modes of operation & setting speed
    - Motor Watchdog
    - Reading Sensor data
    - Example program using python interface script
- LED Indicator Info


- Versioning and version check info
    - Version Format: MAJOR.MINOR.REVISION
    - REVISION Changes (MAJOR.MINOR remains the same)
        - No commands or messages will be removed from the control board
        - Commands / messages will remain backward compatible (lengths of existing messages will also not change)
        - New commands / messages may be added provided they do not impact operation of other existing commands / messages
        - An interface script from an older revision will work with the firmware of a newer revision (with the exception of bugfixes such as timing bugs, comms bugs, etc)
        - A newer interface, however, may not work with the older firmware (as the new interface may expect some commands the older firmware does not provide)
    - MINOR changes (MAJOR remains the same)
        - Comm spec may not be backwards compatible
        - Removal of commands, significant changes to commands or messages (data, length, prefixes, etc)
        - Changes to the intent of how existing commands / messages work / operate (not bug fixes, but changes of intent)
    - MAJOR changes
        - Reserved for fundamental redesigns of things. This could be control modes, command interface, message format, etc
    
    - When connecting to control board, it is recommended to compare version reported by the control board to the expected version using the following rules. "eMAJOR.eMINOR.eREVISION" is the expected version (the version the control board interface code expects). "fMAJOR.fMINOR.fREVISION" is the version of the firmware on the control board.
        - If the MAJOR or MINOR versions do not match (fMAJOR != eMAJOR or fMINOR != eMINOR), this is likely not OK
        - If eREVISION < fREVISION (the firmware is a newer revision) this is likely fine
        - If eREVISION > fREVISION, this may be an issue (firmware may not have features expected)
        - If the version matches exactly, there is no issue
    
    - Note that pre-release firmware will also have a "type" and "build".
        - [MAJOR].[MINOR].[REVISION]-[TAG][BUILD]
        - Tag is "a", "b", or "c" for "alpha", "beta", "rc"
        - When running version checks, it is recommended to always print / log that the control board is running beta firmware (or alpha or release candidate)
        - For version checks, ignore the tag and build for comparisons
