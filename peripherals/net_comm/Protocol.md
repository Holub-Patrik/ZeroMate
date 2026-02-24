# Protocol definitions

Client-Server doesn't matter in here, only the one who tries to connect to the other side

The app should be relatively generic to support even more complex connection schemes.

## Communication steps

1. Both sides open port for communication for setup and port for back and forth communication
1. One side sends a configuration to the other, the other responds accept or decline
1. In case of decline the connection is considered "dropped"
1. After accept, an acknowledgement of accept should be sent
   - This step basically ensures that both sides are ok with sending/receiving data

### Datatypes information:

#### Conf message

- Magic | 1B set of bits (number)
- Opened Port | 2B number
- Protocol Identifier | 1B Enum
- Protocol Information | Arbitrary Length data bounded by protocol identifier
  - UART - 4B
    - Baudrate
  - I2C - 1B
    - Slave/Master
  - SPI - 1B
    - Slave/Master
  - General Buffered
  - General Unbuffered
- Clock | 1B T/F
- Clock Unit | 1B number
- Clock Value | 4B number

#### Accept/Decline

- Magic | 1B set of bits
- Accept | 1B T/F
- Port | 2B number

#### Accept Ack

- Magic | 1B set of bits
- Ack | 1B T/F

## Setup definition

### Send info

- Magic byte
- Protocol identifier (most likely a number from an enum)
- Explicit/Implicit clock rate
  - Implicit brings higher overhead in packets (data + diffs from 0th bit)
  - Explicit is started with the 1st bit and abides the speed required
  - Explicit doesn't require diffs so the data can be smaller
- Port informatioa
  - Opened port for RX (Incoming)

### ACK info

- Magic byte
- Accept/Decline
- Port opened for RX (Incoming)

## Data definition

Scratch to work on data to be used

TODO:

- Where to store relation between pins and connections

```cpp
// configuration port/socket can be only one as it's data rate isn't too high
in_port_t config_port;
// for each connection a new port will be opened
std::map<in_port_t, bool> opened_ports{};

enum Protocol {
    UART,
    I2C,
    SPI,
};

struct UART {
    struct sockaddr_in other_side;
};

struct I2C {
    std::vector<struct sockaddr_in> slaves;
};

struct SPI {
    std::vector<struct sockaddr_in> slaves;
    int chip_select;
};

union ConnectionInfo {
    struct UART,
    struct I2C,
    struct SPI,
};

struct ProtocolConnectionInfo {
    enum Protocol p;
    union ConnectionInfo info;
};

// this is incorrect
// I need to move to a BusConnection and P2PConnection paradigm
struct Connection {
    enum Protocol;

    // Explicit Clock rate will be (1 / (10^clock_unit)) * clock_value 
    // so for example:
    // - clock_unit = 6
    // - clock_value = 10
    // - final clock_rate = 10 microseconds
    bool explicit_clock;
    std::int8_t clock_unit;
    std::uint64_t clock_value;

    // when closing connection this is the port to shutdown
    in_port_t opened_port; 

    // used for recvfrom/sendto
    // constructed from received config ip
    struct ProtocolConnectionInfo prot_conn;
};

// map using unique connection ids
// This unique id is structured as such 0(2bytes) PORT(2bytes) IP(4bytes)
std::map<std::uint64_t, Connection> connection_map;

// I will need a way to map pins to connections they belong to
std::map<std::uint8_t, std::uint64_t> pin_to_connection;
```

### Data notes

#### Sockaddr_in

```cpp
struct sockaddr_in {
    sa_family_t sin_family;
    in_port_t sin_port; // uint16_t
    struct in_addr sin_addr;
};

struct in_addr {
    in_addr_t s_addr; // uint32_t
};

// uint32_t -> netlong -> htonl | ntohl
// uint16_t -> netshort -> htons | ntohs
```

## UART

What needs to be defined in UI:

Start bit count
Data bit count
Parity style
Stop bit count

Depenging on that define the length of the buffer for the message (Most likely byte aligned)
The messages sent can be quite large so I can afford say for example 4 bytes for data, each including the bit info.

## I2C

Data message or clock message definition (1st byte)
If data message, data bit needs to be sent and then a clock signal needs to sent

For I2C from what I understand it will be communication based like this:

1. The master sends a byte and clock
1. My component will pulse data and clock for slave while accumulating data from slave
1. Send back to master and pulse the data

## SPI

The same basic idea, but here the data and clock needs to be sent at all times
