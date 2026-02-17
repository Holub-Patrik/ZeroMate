# Protocol definitions

Client-Server doesn't matter in here, only the one who tries to connect to the other side

## Communication steps

1) Both sides open port for communication for setup and 2 ports for TX/RX for back and forth communication
2) One side sends a configuration to the other, the other responds accept or decline
3) In case of decline the connection is considered "dropped"
4) After accept, an acknowledgement of accept should be sent
    - This step basically ensures that both sides are ok with sending/receiving data

## Setup definition

- Protocol information (most likely a number from an enum)
- Clock rate to use ?

## UART
What needs to be defined:

Start bit count
Data bit count
Parity style
Stop bit count

Depenging on that define the length of the buffer for the message (Most likely byte aligned)
The messages sent can be quite large

## I2C

Data message or clock message definition (1st byte)
If data message, data bit needs to be sent and then a clock signal needs to sent

## SPI

The same basic idea, but here the data and clock needs to be sent at all times
