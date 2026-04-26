# micro-ROS Node Deep Dive (Zephyr app_microros_node)

This document gives a complete implementation-level view of the Zephyr micro-ROS node, including architecture, runtime behavior, Wi-Fi bug history, and synchronization design.

## Diagram Set (PlantUML Sources)

1. Component architecture: [docs/diagrams/microros_node_component.puml](diagrams/microros_node_component.puml)
2. Deployment architecture: [docs/diagrams/microros_node_deployment.puml](diagrams/microros_node_deployment.puml)
3. Class diagram: [docs/diagrams/microros_node_class_diagram.puml](diagrams/microros_node_class_diagram.puml)
4. Sequence diagram: [docs/diagrams/microros_node_sequence.puml](diagrams/microros_node_sequence.puml)
5. Activity flow: [docs/diagrams/microros_node_activity_flow.puml](diagrams/microros_node_activity_flow.puml)

## Architecture Summary

The app in [firmware/zephyr/app_microros_node/src/node_main.cpp](../firmware/zephyr/app_microros_node/src/node_main.cpp) wires together:
- Wi-Fi bootstrap and UDP transport setup.
- micro-ROS node/publishers/timers/executor lifecycle.
- Sensor acquisition and publish callbacks in [firmware/zephyr/app_microros_node/src/node_callbacks.cpp](../firmware/zephyr/app_microros_node/src/node_callbacks.cpp).
- OLED rendering through [firmware/zephyr/app_microros_node/src/oled_layout.cpp](../firmware/zephyr/app_microros_node/src/oled_layout.cpp) and [firmware/zephyr/app_microros_node/src/oled_wrapper.cpp](../firmware/zephyr/app_microros_node/src/oled_wrapper.cpp).
- SD logging in [firmware/zephyr/app_microros_node/src/storage.cpp](../firmware/zephyr/app_microros_node/src/storage.cpp).

Runtime split:
- Main thread: one-time initialization and executor thread spawn.
- Executor thread: periodic timer callbacks + control actions.
- Driver/network callback contexts: GNSS data callbacks and network IPv4 event callback.

## Wi-Fi Related Bugs Resolved

The following bugs are tracked as fixed or mitigated in this repository code and notes.

1. Wi-Fi connect reported before IP was actually usable
- Symptom: transport initialization raced DHCP completion.
- Fix: wait on IPv4 acquisition event (`NET_EVENT_IPV4_ADDR_ADD`) using a semaphore before declaring success.
- Implementation: [firmware/zephyr/app_microros_node/src/wifi_handler.cpp](../firmware/zephyr/app_microros_node/src/wifi_handler.cpp)

2. Connect attempts could inherit stale readiness state
- Symptom: retries could reuse previous semaphore state and return false success/failure.
- Fix: explicit `k_sem_reset(&ipv4_ready_sem)` at start of each connect attempt.
- Implementation: [firmware/zephyr/app_microros_node/src/wifi_handler.cpp](../firmware/zephyr/app_microros_node/src/wifi_handler.cpp)

3. Transient Wi-Fi attach failures caused hard startup failure
- Symptom: single connect attempt was too brittle for embedded startup timing.
- Fix: bounded retry loop with backoff (`WIFI_MAX_RETRIES`, `WIFI_RETRY_DELAY_S`) and disconnect on timeout.
- Implementation: [firmware/zephyr/app_microros_node/src/wifi_handler.cpp](../firmware/zephyr/app_microros_node/src/wifi_handler.cpp)

4. Agent ping loop could starve networking time on constrained targets
- Symptom: long ping bursts delayed network thread progress.
- Fix: keep ping calls short and add sleeps between attempts.
- Implementation: [firmware/zephyr/app_microros_node/src/node_main.cpp](../firmware/zephyr/app_microros_node/src/node_main.cpp)

5. ESP32 Wi-Fi + picolibc duplicate `random()` symbol conflict
- Symptom: link-time symbol collision between ESP32 Wi-Fi adapter and libc.
- Current app mitigation: allow multiple definitions in CMake.
- Implementation: [firmware/zephyr/app_microros_node/CMakeLists.txt](../firmware/zephyr/app_microros_node/CMakeLists.txt)
- Repository fix note: weak-symbol approach recorded in repository memory note (`esp32_wifi_picolibc_random_conflict_note.md`) for long-term cleanup.

## Why Mutex, Semaphores, Spinlocks, and Atomics Are Used

### 1) Mutex (`k_mutex`) in OLEDLayout
- Where: [firmware/zephyr/app_microros_node/src/oled_layout.cpp](../firmware/zephyr/app_microros_node/src/oled_layout.cpp)
- Protects: shared display state (active view, caches, draw/finalize sequencing).
- Why: multiple call paths can update display state (timer callbacks, startup flow, button handling). Mutex keeps frame composition coherent and prevents interleaved draw operations.

### 2) Semaphore (`k_sem`) in WifiHandler
- Where: [firmware/zephyr/app_microros_node/src/wifi_handler.cpp](../firmware/zephyr/app_microros_node/src/wifi_handler.cpp)
- Protects/synchronizes: event-to-thread handoff from async net callback to sync connect logic.
- Why: network events are asynchronous; the app needs a deterministic blocking wait until IPv4 is ready.

### 3) Spinlocks (`k_spinlock`) for GNSS message handoff
- Where producer lock/unlock in [firmware/zephyr/app_microros_node/src/node_main.cpp](../firmware/zephyr/app_microros_node/src/node_main.cpp)
- Where consumer lock/unlock in [firmware/zephyr/app_microros_node/src/node_callbacks.cpp](../firmware/zephyr/app_microros_node/src/node_callbacks.cpp)
- Protects: shared NavSatFix message structs copied between callback context and executor thread.
- Why: callback and executor can run concurrently; spinlock gives very short critical sections suitable for low-latency data snapshot transfer.

### 4) Atomics (`atomic_t` and atomic bit flags)
- Where: [firmware/zephyr/app_microros_node/src/node_main.cpp](../firmware/zephyr/app_microros_node/src/node_main.cpp), [firmware/zephyr/app_microros_node/src/node_callbacks.cpp](../firmware/zephyr/app_microros_node/src/node_callbacks.cpp)
- Used for:
  - readiness gates (`init_complete`, message ready bits),
  - control flags (`oled_view_switch_request`, `time_is_valid`),
  - counters (`*_satellites_tracked`).
- Why: lock-free state signaling for simple cross-context flags and counters, avoiding heavier locks when full structure protection is not needed.

## Practical Reading Order

To get the complete picture quickly:
1. Read component + deployment diagrams first.
2. Read sequence + activity diagrams for runtime behavior.
3. Read class diagram for API/ownership structure.
4. Use Wi-Fi bug and synchronization sections as implementation rationale.
