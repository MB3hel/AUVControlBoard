# Overview

TODO: Project Structure and Build System

TODO: Hardware abstraction (not really proper abstraction, just multiple implementatiuons of the same external API defined in `include/hardware` headers; each target has implementation in `src/hardware` files using include guards to choose which implementation)

TODO: Generator projects

<!--
## Why Generators

The decision to use generator projects comes down to the following

1. There are multiple versions of Control Board with multiple chips. Using generators to create chip-specific initialization code reduces the chip-specific parts of the firmware that must be written / maintained.

2. The generators are often required to use the manufacturer's HAL (or make using the HAL much easier). Use of the manufacturer's HAL / libraries reduces development time and makes maintenance easier (especially for those less familiar with the codebase).

3. Having a GUI tool (which these generators usually are) to configure the system is an easy way to quickly understand the architecture of the system (clocks, peripheral use, pinout, etc).
-->

TODO: System architecture
