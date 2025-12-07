## Implementation steps
1. Create GUI to send pin samples (UDP/TCP)
2. Create GUI element to open a socket and try to connect to other side
3. Create GUI element to set which communication mode to use

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
