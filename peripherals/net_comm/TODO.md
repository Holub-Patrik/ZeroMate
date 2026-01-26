### Notes
The actual implementation should be fairly simple
Both sides will open a socket as a server and try to connect to the other side
The servers should use poll to read IO
The protocol will be most likely very simple, and maybe even binary:
- Set -> Binary form
- Read -> When a change happens on pin, send to other side
- GPIO Pin number should be small (I think 2 bytes should suffice)
- ZM|S/R|0/1||GPIO Pin number|
The changes maybe need to carry information about time
Since the net can put 2 messages together I need to know what timeout to set between them

### Protocol Details
The protocol has to be a bit more complex and the flow of program has to change

Beginning:
TCP handshake implemented on UDP
The handshake will be done using a hello message, then exchange of config options, finally a message that stream of data can start

The config has to have:
Bit-banging rate: How quickly will the app expect messages to come, aka to which time intervals will it be synced
No info handling: What to do when no data comes in time
- always zero
- always one
- repeat last

The procedure has to go like this:
Client sends a config
Server responds with it's own config
- Either the same as client
- Different as the client
Client sends back the server config or disconnect message
If the server agrreed to the client config, it will send it back to client.

At that point the communication configuration should be saved into internal data and used for sending
Connection:
- Address to send to
- bit banging 
    - rate example 115200 baudrate -> 8.68 us (4 us so the changes are fast enough)
        - I have no idea if this rate even is possible
    - rate example 9600 baudrate -> 104,16 us (50 us so the changes are fast enough)
        - This might be doable
