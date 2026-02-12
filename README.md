# Wireless Synchronization System Based on the Alignment Principle
## nRF52832 / nRF52 DK (PCA10040)

This repository demonstrates a **wireless multi-node synchronization system**
implemented on **nRF52832** using the **nRF52 DK (PCA10040)**.

The system synchronizes LED blinking patterns across multiple boards using a
simple **proprietary radio protocol**, without node IDs or parent/child roles.

---

## Features

- No parent / child roles and no node addressing
- Automatic clock drift compensation between nodes
- Intermittent RX/TX operation for low average power consumption
- Natural multi-hop (daisy-chain) propagation when nodes are not all in RF range
- Any node can update the shared LED pattern

---

## Synchronization Concept

### Basic Idea

1. One node periodically transmits a **sync + data** frame.
2. Other nodes receive at intervals that are **integer multiples** of the transmit interval.
3. Each receiver aligns its local timing based on the received sync information.
4. Once synchronized, each node also transmits its own sync frame.
5. Repeating this behavior across all nodes results in system-wide synchronization.

This timing structure causes nodes to **naturally line up over time** without
explicit scheduling or addressing.

---

## Timing Diagram
Time   
                                                                                                                            
Node A (TX):
|TX|    |TX|    |TX|    |TX|    |TX| ...
^                         ^
|----- 250 ms period -----|
Node B (RX):
|---------------- RX window ----------------|
(every 10 seconds)
Result:
Node B gradually aligns its timing to Node A.

---

## Packet Format


+------------+------------+----------------------+------------+------------+
| Header     | Sync (2B)  | RTC Timestamp (4B)   | Pattern 1B | Checksum   |
| "@wp1@"    | "00"-"95"  | 32-bit counter       | LED mode   | Sum(5..15) |
+------------+------------+----------------------+------------+------------+

- **Sync**: cyclic counter (0?95) used for LED timing alignment
- **RTC timestamp**: helps compensate clock drift
- **Pattern**: LED blinking pattern
- **Checksum**: simple byte sum for frame validation

---

## Multi-hop Propagation

Nodes do not need to be within direct RF range of all other nodes.


[Node A] ---> [Node B] ---> [Node C] ---> [Node D]
Sync and pattern propagate hop-by-hop

---

## Power Consumption

The system uses intermittent communication after synchronization.

- **TX interval**: every 250 ms (TX duration ≈ 3 ms)
- **RX interval**: every 10 seconds (RX duration ≈ 25 ms)
- **Supply voltage**: 3.0 V
- **DC/DC converter**: Enabled
- **TX output power**: +4 dBm (radio current ≈ 12–16 mA during TX)
**Average current consumption:**  
** 0.3 mA @ 3.0 V (UART OFF)**

> UART output significantly increases current consumption.  
> The above value is measured with UART disabled.

---

## How to Build and Run

1. Use **nRF5 SDK v12.3.0** (legacy SDK).
2. Copy `main.c` into  
   `examples/proprietary_rf/esb_prx/`  
   replacing the original file.
3. Build the example to generate a HEX file.
4. Flash **three or more** nRF52 DK boards.

### Operation

- **LED3** on all boards will blink in sync.
- **LED2 / LED4** may turn on for internal status indication.
- Press **Button1** on any board to change the blinking pattern.
- The new pattern propagates to all nodes and re-synchronizes.

? **Note:**  
Because button timing itself is not transmitted,  
wait **at least 60 seconds** between button presses for stable operation.

---

## Notes and Limitations

- Fixed RF channel and simple application-layer checksum
- No collision avoidance or node identification
- Designed as a demonstration of timing alignment and low-power scheduling
- nRF5 SDK 12.3.0 is legacy; porting to newer SDKs is future work

---

## Files

- `main.c` : main source file
- HEX file can be provided upon request

---

## License

This project is based on Nordic Semiconductor example code.
The original Nordic license header is retained in `main.c`.


