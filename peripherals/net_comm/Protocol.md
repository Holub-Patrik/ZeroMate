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

How the hell should the routing happen ?? And how to do it so it actually runs fast ??
This is a question what needs to be associated with each pin
The data pushed will have to be a pair of pin and value
The connection information saved will have to do the ruouting later

```cpp
struct Connection {
    int queue_idx; // into which queue to place the data. But what data?
};

struct PinInfo{
    Connection conn;
}

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

I2C communication is driven on a high level using requests and responses. The Master tracks the state and sends commands to the Slave.

### Packet Format

I2C packets use a fixed-size structure:
- Type (1 byte): `I2C_START`, `I2C_STOP`, `I2C_ADDRESS`, `I2C_WRITE_BYTE`, `I2C_READ_BYTE`, `I2C_ACK`, `I2C_DATA`.
- Value (1 byte): Payload (Address, Data byte, or bool for ACK).

### Master -> Slave Commands

1. **I2C_START**: Sent when a START condition is detected.
2. **I2C_ADDRESS**: Sent after accumulating 8 bits (Address + R/W). Master halts and waits for `I2C_ACK` from Slave.
3. **I2C_WRITE_BYTE**: Sent after accumulating 8 bits of data. Master halts and waits for `I2C_ACK`.
4. **I2C_READ_BYTE**: Sent when Master starts reading a byte. Master halts and waits for `I2C_DATA` (8 bits).
5. **I2C_ACK**: Sent after Master reads a byte, contains the ACK/NACK bit driven by the Master.
6. **I2C_STOP**: Sent when a STOP condition is detected.

### Slave -> Master Responses

1. **I2C_ACK**: Sent by the Slave in response to `I2C_ADDRESS` or `I2C_WRITE_BYTE`.
2. **I2C_DATA**: Sent by the Slave in response to `I2C_READ_BYTE`, contains the full 8-bit data.

### Implementation Details

The Slave acts as a local Master for peripherals on its side, bit-banging the high-level commands locally to SCL/SDA pins and reading back responses (ACKs or data) to send back over the network.

## SPI

The same basic idea, but here the data and clock needs to be sent at all times
