<!-- Note, it is intentionnal that the first heading starts as a "level2"/## and not as level1/# and should be kept like this!! -->
## RTOS (FreeRTOS on ESP32)

### For people that are new to multi-threading
An RTOS lets you split one big `loop()` into multiple independent “tasks” that run *as if* they were in parallel. The RTOS decides which task runs when, so timing-critical work (e.g. control loops) can keep a stable update rate even while other work (e.g. networking) is happening.

### Main idea
This firmware uses FreeRTOS to run multiple subsystems concurrently (IMU, control loop, sensors, diagnostics) with explicit timing and priorities.

Networking is implemented using an async web stack (AsyncTCP/ESPAsyncWebServer). Its callbacks run in their own context, so the project does not need a dedicated “networking loop task” for basic operation.


### What an RTOS is (and why we use it)
FreeRTOS is a small real-time operating system used on microcontrollers. “Real-time” here means we care about predictable timing: a task can run periodically (e.g. every 5ms) and the scheduler helps keep that periodic behavior even when other tasks are active.

Official FreeRTOS website: https://www.freertos.org/

### Two cores on the ESP32 (what that means)
Many ESP32 chips have two CPU cores. In practice that means the RTOS can run one task on core 0 while another task runs on core 1 at the same time.

In this firmware we *pin* some tasks to a specific core using `xTaskCreatePinnedToCore(...)`:
- **Core pinning** is mainly used to isolate “noisy” workloads (like network/web housekeeping) from timing-sensitive workloads (like sensor reading / control loops).
- Pinning does not magically make code thread-safe: any data shared between tasks still needs safe handoff (queues, atomic variables, etc.).

### Scheduling (very short reminder)
The scheduler is the RTOS component that decides which task runs next based on priority and “readiness”. A task becomes not-ready when it blocks (e.g. delays for some time, waits on a queue), and ready again when that wait condition is satisfied.

Common timing patterns you’ll see in this firmware:
- `vTaskDelay(...)`: simple “sleep” for at least a given time.
- `vTaskDelayUntil(...)`: periodic scheduling pattern (tries to keep a constant period, useful for control loops).

### RTOS building blocks used in this project (high level)
- **Tasks:** each subsystem runs in its own task function.
- **Priorities:** tasks can be assigned different priorities so time-critical tasks run first.
- **Queues:** tasks exchange data without sharing raw variables directly (producer/consumer pattern).

