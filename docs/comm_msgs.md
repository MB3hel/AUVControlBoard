# Communication Messages

The following are the messages sent to the control board or received from the control board and what they mean / do. These messages are the *payload* in the message format described on the [Communication Protocol](./comm_protocol.md) page. Note that all characters are ASCII encoded unless enclosed within square brackets. A literal square bracket is escaped with a backslash. A literal backslash is also escaped with a backslash. Tokens inside square brackets are described per message below.


**Set MODE**

*Sent from computer to control board*

```
MODE[mode_val]
```

where `mode_val` is one of the following

- `R`: RAW mode
- `L`: LOCAL mode


**Get MODE**

*Sent from computer to control board*

```
?MODE
```

Response (from control board to computer) will be in the format

```
MODE[mode_val]
```

where `mode_val` is one of the following

- `R`: RAW mode
- `L`: LOCAL mode


**Set Thruster Inversions**

*Sent from computer to control board*

```
TINV[invert1][invert2][invert3][invert4][invert5][invert6][invert7][invert8]
```

where `invertx` (for `x` = 1, 2, ..., 8) is a 1 or a zero (8-bit integer) where 1 means thruster `x` is inverted and 0 means thruster `x` is not inverted.


**Get Thruster Inversions**

*Sent from computer to control board*

```
?TINV
```

Response (from control board to computer) will be in the format

```
TINV[invert1][invert2][invert3][invert4][invert5][invert6][invert7][invert8]
```

where `invertx` (for `x` = 1, 2, ..., 8) is a 1 or a zero (8-bit integer) where 1 means thruster `x` is inverted and 0 means thruster `x` is not inverted.


**Set RAW Speeds**

*Sent from computer to control board*

```
RAW[speed1][speed2][speed3][speed4][speed5][speed6][speed7][speed8]
```

where `speedx` (for `x` = 1, 2, ..., 8) is a little endian encoded 32-bit float between -1.0 and 1.0 indicating the speed of thruster `x`.


**Feed Motor Watchdog**

*Sent from computer to control board*

```
WDGF
```


**Motor Watchdog Did Disable**

*Sent from control board to computer if the motor watchdog automatically kills motors.*

```
WDGK
```
