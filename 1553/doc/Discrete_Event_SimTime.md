# Phase 2: Discrete-Event Simulation Timeline & SimTime

## 1. Executive Summary & Problem Statement

### The Problem in Synchronous Bus Simulation
In early simulation prototypes, bus operations are typically executed **synchronously**:
```cpp
// Flawed approach: Instantaneous execution or host sleeping
auto result = bus.execute(command); // Returns immediately or sleeps host thread
```

This presents critical fidelity and performance deficiencies:

1. **Host CPU Sleeping Destroys Simulation Throughput**:
   Calling `std::this_thread::sleep_for()` or `std::this_thread::sleep_until()` forces the simulation host CPU to sit idle waiting for real-world wall-clock time to pass. A simulation that should execute in 5 milliseconds takes 10 full seconds of wall-clock time.

2. **Transmitted-Cell Counters vs. True Bus Timeline**:
   A local clock tick counter that increments only when bits are transmitted measures *transmitted cell activity*, not *elapsed bus time*. It cannot account for:
   * **RT Response Delays**: The $4.0\,\mu\text{s}$ to $12.0\,\mu\text{s}$ dead time between a command and the terminal's status response.
   * **Intermessage Gaps**: The mandatory $\ge 4.0\,\mu\text{s}$ quiet period between consecutive bus transfers.
   * **No-Response Timeouts**: The $14.0\,\mu\text{s}$ wait period before the Bus Controller declares an RT non-responsive.

3. **Causality Violations (Observing Data Before Transmission Completes)**:
   In synchronous execution, guidance algorithms observe telemetry *instantaneously* upon calling `execute()`, even though on a real physical bus the data transfer requires dozens to hundreds of microseconds to serialize across the twisted pair.

4. **Bus Contention & Serialization**:
   A single-channel MIL-STD-1553 bus is a shared, half-duplex medium. If a flight software task requests an IMU read while an actuator command is still transmitting on the wire, the second transaction must be queued or deferred until the bus becomes available.

---

## 2. The Solution: Discrete-Event Architecture & SimTime

Phase 2 replaces synchronous execution with an authoritative, discrete-event simulation timeline driven by `SimTime`.

```
========================================================================================
                                 SIMULATION TIMELINE (SimTime)
========================================================================================

    t = 0.0 us                t = 20.0 us      t = 28.0 us   t = 48.0 us     t = 112.0 us
         |                         |                |              |               |
         +-------------------------+----------------+--------------+---------------+
         |    Command Word (20us)  | RT Resp (8us)  | Status (20us)| 3x Data (60us)| Gap(4us)
         +-------------------------+----------------+--------------+---------------+
         |<-------------------------- Total Duration = 112.0 us ------------------------->|

  [submit(cmd)]                                                            [poll() completes]
  (Returns TxID=1)                                                         (Data visible)
========================================================================================
```

### Key Architectural Pillars

1. **`SimTime` Representation (`1553/include/mil1553/SimTime.hpp`)**:
   * Stored internally as a 64-bit integer representing **nanoseconds** (`int64_t`), preventing floating-point drift during multi-hour simulations.
   * Provides zero-overhead factory helpers: `SimTime::fromMicroseconds()`, `SimTime::fromMilliseconds()`, `SimTime::fromSeconds()`, `SimTime::zero()`.
   * Implements full arithmetic (`+`, `-`, `*`, `/`), relational operators (`<`, `<=`, `>`, `>=`, `==`), and human-readable string formatting (`112.000 us`, `10.000 ms`).

2. **Standard MIL-STD-1553 Timing Mathematics (`Mil1553Timing`)**:
   * **Bit Rate**: 1.0 MHz ($1.0\,\mu\text{s}$ per bit).
   * **Word Duration**: Exactly 20 bits = $20.0\,\mu\text{s}$ (3-bit sync + 16-bit payload + 1-bit odd parity).
   * **RT Response Time**: $4.0\,\mu\text{s}$ to $12.0\,\mu\text{s}$ (nominal $8.0\,\mu\text{s}$).
   * **Intermessage Gap**: Mandatory minimum $4.0\,\mu\text{s}$ between transactions.
   * **No-Response Timeout**: $14.0\,\mu\text{s}$ wait period before declaring terminal timeout.

   $$\text{Duration}_{\text{BC}\rightarrow\text{RT}} = T_{\text{cmd}}(20\,\mu\text{s}) + (N \times 20\,\mu\text{s}) + T_{\text{response}}(8\,\mu\text{s}) + T_{\text{status}}(20\,\mu\text{s}) + T_{\text{gap}}(4\,\mu\text{s})$$

   $$\text{Duration}_{\text{RT}\rightarrow\text{BC}} = T_{\text{cmd}}(20\,\mu\text{s}) + T_{\text{response}}(8\,\mu\text{s}) + T_{\text{status}}(20\,\mu\text{s}) + (N \times 20\,\mu\text{s}) + T_{\text{gap}}(4\,\mu\text{s})$$

   $$\text{Duration}_{\text{RT}\rightarrow\text{RT}} = 2 \times T_{\text{cmd}}(40\,\mu\text{s}) + T_{\text{txResp}} + T_{\text{txStatus}} + (N \times 20\,\mu\text{s}) + T_{\text{rxResp}} + T_{\text{rxStatus}} + T_{\text{gap}}$$

   $$\text{Duration}_{\text{Timeout}} = T_{\text{cmd}}(20\,\mu\text{s}) + T_{\text{timeout}}(14\,\mu\text{s}) + T_{\text{gap}}(4\,\mu\text{s}) = 38.0\,\mu\text{s}$$

3. **Asynchronous Transaction Lifecycle (`submit` / `poll`)**:
   * **`submit(cmd, data, earliestStart)`**:
     Schedules a transaction. If the bus is currently transmitting a prior message, the new transaction is automatically scheduled to start at `busAvailableTime_`, enforcing half-duplex mutual exclusion.
   * **`poll(transactionId)`**:
     Inspects transaction status. If `currentTime < completionTime`, `isCompleted` is `false` and payload data is masked. Once `currentTime >= completionTime`, `isCompleted` is `true` and the full `TransactionResult` is returned.
   * **`processPending(upToTime)`**:
     Advances the timeline to `upToTime` and returns all transactions that reached completion.

4. **Decoupled Fast Transaction vs. Waveform Fidelity Modes**:
   * **Fast Transaction Mode (`waveformFidelity = false`)**:
     Directly executes message parsing, validation, and RT dispatch in $O(1)$ time while updating `SimTime` with microsecond precision. Allows GNC simulations to run at **$100\times$ to $1000\times$ real-time speed**.
   * **Waveform Fidelity Mode (`waveformFidelity = true`)**:
     Synthesizes Manchester II biphase level waveforms (40 samples/word) and drives differential voltages across `Hal1553::Wire` for physical layer and transceiver hardware testing.

5. **100% Backwards-Compatible Synchronous Execution**:
   * `execute()`, `transmitToRT()`, `receiveFromRT()`, and `transferRtToRt()` submit transactions and advance the simulation timeline directly to the completion timestamp, maintaining full compatibility with existing code and tests without sleeping the host CPU.

---

## 3. Code Examples & Usage

### Example 1: Asynchronous Transaction Scheduling and Polling
```cpp
#include "mil1553.hpp"
#include <iostream>

void exampleAsync() {
    BC bc;
    auto imu = std::make_shared<ImuRT>(3);
    bc.attachTerminal(imu);

    // Current simulation time: t = 0 us
    bc.setCurrentTime(SimTime::fromMicroseconds(0.0));

    // Submit an asynchronous read request for 3 words (Pitch, Roll, Yaw)
    // Duration: 20us (cmd) + 8us (resp) + 20us (status) + 60us (3 data words) + 4us (gap) = 112.0 us
    TransactionId txId = bc.submitReceiveFromRT(3, /*subaddress=*/1, /*wordCount=*/3);

    // Immediate poll at t = 0 us (transfer is in-flight)
    auto statusAt0 = bc.poll(txId);
    std::cout << "t = 0 us: isCompleted = " << std::boolalpha << statusAt0->isCompleted << "\n"; // false

    // Advance simulation timeline to t = 112 us
    bc.advanceTo(SimTime::fromMicroseconds(112.0));

    // Poll after completion timestamp has been reached
    auto statusAt112 = bc.poll(txId);
    std::cout << "t = 112 us: isCompleted = " << statusAt112->isCompleted << "\n"; // true
    std::cout << "IMU Status OK = " << statusAt112->result.status.isOk() << "\n";
}
```

### Example 2: Automatic Bus Contention Serialization
```cpp
void exampleContention() {
    BC bc;
    auto imu = std::make_shared<ImuRT>(3);
    auto actuator = std::make_shared<ActuatorRT>(5);
    bc.attachTerminal(imu);
    bc.attachTerminal(actuator);

    bc.setCurrentTime(SimTime::zero());

    // Submit IMU Read (completes at t = 112.0 us)
    TransactionId id1 = bc.submitReceiveFromRT(3, 1, 3);

    // Submit Actuator Command at the same time (t = 0)
    DataBlock1553 actData{};
    actData.count = 1;
    actData.words[0] = 500;
    TransactionId id2 = bc.submitTransmitToRT(5, 1, actData);

    auto rec1 = bc.poll(id1);
    auto rec2 = bc.poll(id2);

    // Transaction 1 executes from 0.0 us to 112.0 us
    std::cout << "Tx 1 Start: " << rec1->actualStart << ", Complete: " << rec1->completionTime << "\n";

    // Transaction 2 is automatically queued and serialized to start at 112.0 us!
    std::cout << "Tx 2 Start: " << rec2->actualStart << ", Complete: " << rec2->completionTime << "\n";
}
```

### Example 3: Accelerated 100 Hz Flight Control Loop (Zero Wall-Clock Sleeps)
```cpp
void runAcceleratedFlightSimulation() {
    BC bus;
    // ... attach terminals ...

    SimTime simTimeline = SimTime::zero();
    const SimTime cyclePeriod = SimTime::fromMilliseconds(10.0); // 100 Hz = 10 ms

    for (int cycle = 0; cycle < 1000; ++cycle) {
        SimTime cycleStart = simTimeline;

        // 1. Read IMUs over bus (advances simTimeline by ~224 us)
        auto imu1 = bus.receiveFromRT(3, 1, 3);
        auto imu2 = bus.receiveFromRT(4, 1, 3);

        // 2. GNC control computation
        // ... compute controlDemand ...

        // 3. Write Actuators (advances simTimeline by ~72 us)
        DataBlock1553 actCmd{};
        actCmd.count = 1;
        actCmd.words[0] = 1200;
        bus.transmitToRT(5, 1, actCmd);

        // Advance simulation clock to the start of the next 100 Hz cycle
        simTimeline = cycleStart + cyclePeriod;
        bus.setCurrentTime(simTimeline);

        // Host CPU executes all 1000 cycles (10 simulated seconds) in ~15 milliseconds!
    }
}
```

---

## 4. Summary of Improvements

| Metric / Feature | Previous Prototype | Phase 2 Discrete-Event Architecture |
| :--- | :--- | :--- |
| **Simulation Speed** | Locked to 1x Real-Time (via host sleeping) | **100x to 1000x Real-Time** (Discrete Time) |
| **Bus Timing Model** | Transmitter tick count (active cells only) | **Full MIL-STD-1553 Timing (Words + Delays + Gaps + Timeouts)** |
| **Causality Integrity** | Data observable immediately upon calling API | **Data masked until simulated completion timestamp** |
| **Bus Contention** | Not modeled (overlapping calls corrupt wire) | **Automatic queuing & half-duplex wire serialization** |
| **Execution Modes** | Single path with Manchester overhead | **Fast Transaction Mode** + **Waveform Mode** |
