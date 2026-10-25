# MIL-STD-1553: An Engineering Overview

MIL-STD-1553 is a deterministic, command/response serial communication protocol developed for military avionics and subsequently adopted throughout spacecraft and launch-vehicle systems.

Its primary purpose is to provide reliable, predictable communication between flight computers, sensors, actuators, and other mission-critical equipment.

For your work on launch-vehicle modeling and simulation, understanding 1553 is particularly valuable because it connects several areas: flight software, guidance/navigation/control (GNC), embedded hardware, fault management, and simulation.

## 1. What is MIL-STD-1553?

## 1 Mbps

Bit rate

## 32 bits

Command word

## 31

Max RT addresses

## Dual

Redundant buses

1553 is fundamentally different from Ethernet, CAN, or a typical UART connection.

It is a centrally controlled shared bus. Devices do not simply transmit whenever they have data. A designated controller determines when communication occurs.

There are three principal device roles:

- Bus Controller (BC): Schedules and initiates communication.
- Remote Terminal (RT): Responds to commands from the controller. Often interfaces to a sensor, actuator, or subsystem.
- Bus Monitor (BM): Passively observes bus traffic, useful for telemetry, recording, diagnostics, and simulation.

### Conceptual 1553 architecture

In actual installations, each terminal connects through appropriately designed coupling hardware, and Bus A and Bus B are physically separate redundant networks.

## 2. Why aerospace uses 1553

1553 was standardized in the 1970s, yet remains relevant because its design solves problems that high-performance general-purpose networks do not necessarily solve by default.

| Requirement           | How 1553 addresses it                               |
| --------------------- | --------------------------------------------------- |
| Determinism           | The bus controller schedules transactions           |
| Reliability           | Error detection, status reporting, retries          |
| Fault tolerance       | Independent redundant bus channels                  |
| Electrical robustness | Differential signaling, transformer coupling        |
| Interoperability      | Standardized message formats and terminal behavior  |
| Timing predictability | Bounded message lengths and scheduled communication |

Its principal weakness is bandwidth: 1 Mbps is extremely slow by modern networking standards.

But consider a flight computer requesting angular-rate measurements from an IMU. The measurement might require only a handful of 16-bit words. Predictable delivery and fault handling can matter more than raw bandwidth.

This is why 1553 can remain useful for control and housekeeping even when Ethernet or other high-speed links are available.

## 3. How 1553 communication actually works

The bus controller initiates every normal transaction. A remote terminal cannot arbitrarily send a message without being commanded, although it can signal that service is requested.

### Three common transactions

A. Controller requests sensor data

B. Controller sends an actuator command

C. Controller transfers data between terminals

The third case is particularly interesting: the controller orchestrates the transaction, but the data can travel directly between remote terminals without being relayed through the controller's application software.

## 4. The wire protocol

Each 1553 word contains 20 transmitted bits:

There are three word types: command, status, and data.

The command word is particularly important:

The fields specify:

- RT address: Which terminal is being addressed (0–30 for ordinary RTs; 31 is reserved for broadcast).
- T/R: Whether the terminal transmits or receives.
- Subaddress: Which logical endpoint or buffer is involved.
- Word count: Number of 16-bit data words, from 1 through 32. Zero encodes 32 for ordinary transfers.

Subaddresses 0 and 31 have special significance for mode commands.

### A concrete example

Suppose we want to request six data words from an IMU assigned RT address 3, subaddress 2.

The command would be:

```
RT Address:    00011    (3)
T/R:           1        (Transmit)
Subaddress:    00010    (2)
Word Count:    00110    (6)

Command word:  00011 1 00010 00110
Hex:           0x1C46
```

The controller sends this command. The terminal responds with its status word followed by six data words.

A data word could encode an angular velocity, acceleration, temperature, health indicator, or some other value. 1553 defines the transport, not the engineering meaning of the data. That meaning comes from the system's interface control document (ICD).

This distinction is crucial when implementing a simulator.

## 5. Where does Artemis use 1553?

This requires distinguishing between the SLS launch vehicle, the Orion spacecraft, and the Interim Cryogenic Propulsion Stage (ICPS).

[SLS flight software and avionics in "run for record" testing - NASASpaceFlight.com](https://images.openai.com/static-rsc-4/9ERE0ceN9e0fpra0VyZumaissRXRXjVAri2CPEm73pGahJ7qVZxPsjgZHa3_8bNaqaZi1GrTLbAh9YpuHQd4m3WECWoL8iDn0KX0uTRkoSv3cp4eT8Ox5xunOmJbzWj6uND1zWcY0nyzAu-xRuq1BY6mtZY463p41zXbKkSKRiU?purpose=inline)

[Space Launch System (SLS) Overview | NASA](https://images.openai.com/static-rsc-4/yGt-IAeA51aH_t_Tj7oj_1gVGzWq561_xf5kUikFdoMTUc6NfbYuQrEo47yEkQP3HY_hsd-STN1VHM3XNC9GsMEzt5RVHxTYZe4IcwBCe4tAAPzHnCYD4U7rtDv4h-Ypky4vwKKoEyUIq-LUOrRDZPM7alauntmTDPZAsPCdspI?purpose=inline)

[NASA starts buying long lead parts for third Orion ESM, SLS Core Stage - NASASpaceFlight.com](https://images.openai.com/static-rsc-4/x-00WG5rLXbfY_HHTN2ZbE4DBWX9PPCD_8OESlyTQf96q6ARr2wHfhVrWDKh0-7wQchyCZqdvls3aYxYvVYueZaoR_vIt8RvZzENDWXPNMNiiC10R0a7HXSyaT_9jO9e7XFzeveF2zSLMhzzBnqLCUs6F9oQI-QgYbwOu3zVpCU?purpose=inline)

5

NASA confirms that SLS uses distributed avionics, including three flight computers, redundant inertial navigation hardware, engine controllers, booster avionics, and communication interfaces.&#x20;

[image](https://www.google.com/s2/favicons?domain=https://www.nasa.gov\&sz=32)

NASA

+1



However, an important qualification: the public NASA documentation I found does not establish that every one of these interfaces uses MIL-STD-1553. We should not assume that the presence of a flight computer, IMU, or engine controller means its interface is 1553.

Here is the architecture I would use to organize the investigation:

| Artemis subsystem         | Location                 | 1553 assessment                                                     |
| ------------------------- | ------------------------ | ------------------------------------------------------------------- |
| SLS flight computers      | Core-stage forward skirt | Confirmed computers; exact 1553 interfaces need verification        |
| Inertial navigation units | Core-stage avionics      | Confirmed navigation hardware; bus type needs verification          |
| RS-25 engine controllers  | Engine section           | Confirmed controller links; protocol needs verification             |
| Booster avionics          | Solid rocket boosters    | Confirmed communications; protocol needs verification               |
| ICPS avionics             | Upper stage              | Separate avionics architecture; interface details need verification |
| Orion avionics            | Crew/service modules     | Primary onboard network is Time-Triggered Gigabit Ethernet          |

The Orion distinction is especially important. NASA explicitly describes its triple-redundant Time-Triggered Gigabit Ethernet network, rather than 1553, as the primary onboard data network.&#x20;

[image](https://www.google.com/s2/favicons?domain=https://www.nasa.gov\&sz=32)

NASA



There is historical context: NASA's earlier Ares I launch-vehicle avionics design explicitly selected MIL-STD-1553B for most critical functions, with RS-422 for other critical interfaces. But Ares I is not SLS, and its design cannot be treated as proof of SLS wiring.&#x20;

[image](https://www.google.com/s2/favicons?domain=https://ntrs.nasa.gov\&sz=32)

NASA Technical Reports Server



## 6. How I would simulate 1553 in Trick

For your NASA Trick ramp-up, I would build a small, deterministic bus simulation with three remote terminals:

- RT 3: IMU, supplying angular velocity and acceleration.
- RT 5: Engine controller, accepting throttle or gimbal commands.
- RT 7: Telemetry controller, supplying subsystem health.

These addresses and assignments are illustrative, not Artemis interface definitions.

Trick Executive / Simulation Scheduler

Advances simulation time and invokes scheduled jobs

1553 Bus Controller Model

Transaction schedule, timeout, retry, bus selection

Virtual 1553 Bus A / Bus B

IMU RT

Angular rates + acceleration

Engine RT

Actuator commands + status

Telemetry RT

Health + measurements

### Recommended C++ model

```
struct CommandWord {
    uint8_t rtAddress;
    bool transmit;
    uint8_t subAddress;
    uint8_t wordCount;
};

struct Transaction {
    CommandWord command;
    std::vector<uint16_t> data;
    uint16_t status;
    double startTime;
    double completionTime;
    bool success;
};

class Bus1553 {
public:
    void schedule(const Transaction& tx);
    void update(double simTime);
    void injectFault(int rtAddress);
    void selectBus(bool useBusB);
};
```

This is only a conceptual API. A full implementation should also model mode commands, response timing, status bits, terminal-to-terminal transfers, and the protocol's special word-count encoding.

I would implement the simulator in three stages:

1. Functional simulation: Commands, responses, and data movement without electrical timing.
2. Timing simulation: Word transmission time, intermessage gaps, response delays, bus utilization, and scheduling.
3. Fault simulation: Missing responses, parity errors, terminal failures, bus A/B switching, and retries.

The key is to use Trick simulation time, not operating-system sleeps, to advance transactions. That preserves repeatability and allows deterministic Monte Carlo fault testing.

## 7. Useful NASA technical references

The most relevant starting documents I found are:

- [NASA MIL-STD-1553 Standard](https://standards.nasa.gov/standard/NASA/MIL-STD-1553?utm_source=chatgpt.com) — protocol specification and NASA endorsement.
- [SLS Avionics: The Brains of SLS](https://www.nasa.gov/reference/sls-avionics/?utm_source=chatgpt.com) — flight computers, navigation, engine controllers, and integrated testing.
- [SLS Reference Guide (PDF)](https://www.nasa.gov/wp-content/uploads/2022/03/sls-reference-guide-2022-v2-508-0.pdf?utm_source=chatgpt.com) — distributed avionics locations and flight-control architecture.
- [Artemis I Reference Guide (PDF)](https://www.nasa.gov/wp-content/uploads/2023/05/artemis-i-reference-guide-inter.pdf?utm_source=chatgpt.com) — Orion avionics and onboard network architecture.

## 8. What I recommend learning next

For your modeling-and-simulation work, the highest-value topics are bus-controller scheduling, remote-terminal state machines, interface control documents, redundant bus behavior, and timing/error injection.

One particularly useful exercise would be to model a 100 Hz GNC control loop that receives simulated IMU data over a virtual 1553 bus, computes a thrust-vector command, and transmits that command to an actuator terminal. We could then introduce latency, dropped responses, and bus failures to measure their effect on closed-loop stability.

That would connect MIL-STD-1553, C++, Trick, and flight-dynamics simulation in one realistic engineering exercise.