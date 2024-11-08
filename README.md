This repository contains UART DMA receiver code developed for the 2025 University Rover Challenge (URC) for the University of Waterloo Robotics Team. It acts as a fallback layer in case of a Jetson failure in the Mars rover so we can secure communication with only the SIK radio available. There exists a partner repository that contains transmitter code to interface with the receiver code, and it will be linked in the near future (TBD).

Features:
- DMA for Non-Blocking Operation
- Serialization/De-serialization
- Packet Loss Handling
