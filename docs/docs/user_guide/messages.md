# Messages

This section describes what specific messages are sent to / received from the control board and what they do / mean. This does not address how messages are constructed or sent. For such information, see [Communication Protocol](./comm_protocol.md).

*Note: The Communication Protocol page uses the term "payload" for the data being transferred and "message" for the formatted / fully constructed set of data. Here, what we refer to as "messages" are actually the "payload" data, not the constructed data.*

## Types of Messages

- **Commands**: Messages instructing an action be taken. These messages must be acknowledged upon receipt. The acknowledgement typically contains no data. Sent from PC to control board.
- **Queries**: Messages requesting information. These messages must be acknowledged upon receipt. The acknowledgement will contain the requested information. Sent from PC to control board.
- **Acknowledgements**: A very specific type of message acknowledging receipt of another message (with an error code and optional data). Sent from control board to PC.
- **Status Messages**: Unprompted messages containing information about state / data changes. Sent from control board to PC.


## Commands

- TODO


## Queries

- TODO


## Acknowledgements

- TODO


## Status Messages

- TODO
