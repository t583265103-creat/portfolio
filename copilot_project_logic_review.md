# Project Logic Review

## Summary

The autonomous wheelchair project has the right high-level architecture, but the current code contains several integration problems that make the end-to-end flow unreliable. The main pipeline is present, but implementation details are inconsistent and some important modules are not actually connected.

Key findings:
- Global graph planning via Python/A* exists.
- Local planning with D* Lite exists, but incremental updates are broken.
- Sensor fusion and factor graph logic are partially implemented in `ThreadGrafFector`.
- Thread-safety and coordinate frame consistency are weak areas.

## Runtime flow

1. `main.cpp` starts the system:
   - `StateThread`
   - `SensorConsumer`
   - `ThreadGrafFector`
   - `SensorProducer`
2. `SensorConsumer` processes raw sensor data into `FramePointers` via `SensorFrameManager`.
3. `StatePriorityQueue` receives EKF update tasks from sensor consumers.
4. `ThreadGrafFector` fuses object detections and builds factor graph constraints.
5. `ThreadGrafFector` calls `NavigationManager::runDLite(...)` every frame.
6. `NavigationManager` performs local map updates, runs D* Lite, and stores a local path.
7. `main.cpp` uses `NavigationManager::getNextStep()` and `computeCommand(...)` to issue drive commands.
8. The global route is built once in `main.cpp` using `NavigationManager::createMapGlobal(...)`.

## Critical bugs

### 1. Coordinate conversion mismatches
- `StatePriorityQueue::myPlceInWorld()` converts EKF meter state into latitude/longitude incorrectly.
- `Move.hpp::distToNode()` converts node lat/lon into meters assuming a common origin, but no common origin is maintained.
- `NavigationManager::tryAdvanceGlobalNode()` uses the same broken conversion, so global waypoint advancement is likely incorrect.

### 2. D* Lite incremental update bug
- `runDLite()` calls `m_dlite.updateAndReplan(m_mapData->getCells(), robot_col, robot_row);`
- `MapData::getCells()` returns a `cells` vector that is never populated.
- The actual changed map cells are stored in `MapLocal::cells`, but those are not passed.

### 3. Stale cell-change buffer
- `MapLocal::registerCellChange(...)` appends changes to `MapLocal::cells`, but the list is never cleared.
- This can cause D* Lite to receive old or invalid cell updates over time.

### 4. EKF thread-safety issue
- `StatePriorityQueue::getStateVector()` reads `EKF::x` without locking.
- `StateThread` may be updating EKF state concurrently, causing data races and inconsistent pose estimates.

### 5. Frame selection bug in `SensorFrameManager`
- `getProcessFrame()` depends on `ProcessFrame` starting at `0` and advancing only when `nextFrame` exists.
- If the frame bucket IDs are not aligned with that logic, `runDLite()` may receive `nullptr` frames or stale frames.

### 6. GPS units likely wrong
- `GpsProcessing::createObject()` copies raw GPS latitude and longitude into `GpsObject.x_local` and `y_local`.
- These values appear to be degrees, not meters, so GPS data is not in the same unit system as the EKF.

### 7. Dead / disconnected modules
- `MapLive/NavigationThread.hpp` defines a full navigation thread but is unused in `main.cpp`.
- `NavigationManager::updateFromLidar()` and `updateFromObjects()` exist but are not integrated into the active pipeline.

## Missing parts

- Proper global-to-local coordinate origin handling for lat/lon and meter frames.
- A safe shared state accessor for EKF reads.
- A functioning D* Lite update path that uses real changed cells.
- A robust `SensorFrameManager` process-frame advancement policy.
- A correct GPS conversion to local meters.
- A mechanism to clear stale local map change buffers.
- Proper integration or removal of `NavigationThread` and unused map update methods.

## Recommended order of fixes

1. Fix coordinate frame alignment first.
2. Fix D* Lite incremental replan logic.
3. Fix EKF thread safety and consistent state reads.
4. Fix process-frame management in `SensorFrameManager`.
5. Correct GPS local coordinate conversion.
6. Remove or integrate dead/unconnected modules.

## Exact files/functions to fix first

- `Thread/NavigationManager.cpp`
  - `NavigationManager::runDLite(...)`
  - `NavigationManager::tryAdvanceGlobalNode(...)`
- `MapLive/MapLocal.cpp`
  - `MapLocal::registerCellChange(...)`
  - add a method to clear or return fresh cell updates
- `Thread/StatePriorityQueue.hpp`
  - `StatePriorityQueue::getStateVector()`
  - `StatePriorityQueue::myPlceInWorld()`
- `MapLive/Move.hpp`
  - `distToNode(...)`
  - `latLonToMeters(...)`
- `Thread/SensorFrameManager.hpp`
  - `getProcessFrame()`
- `Gps/GpsProcessing.cpp`
  - `GpsProcessing::createObject(...)`

## Suggested corrected logic / pseudocode

```cpp
while (running) {
    auto state = stateQ.getStateVectorSafe();      // protected read

    if (nav.shouldStop()) {
        sendDriveCommand(0, 0);
        continue;
    }

    nav.tryAdvanceGlobalNode(state);               // advance global waypoint if reached

    auto frame = frameManager.getProcessFrame();   // get latest coherent frame
    if (frame) {
        auto tracked = frameManager.getTrackedObject();
        nav.runDLite(frame, tracked, state);       // update local map + D* Lite
    }

    auto maybe_next = nav.getNextStep();
    if (!maybe_next.has_value()) {
        sendDriveCommand(0, 0);
        continue;
    }

    auto next_cell = maybe_next.value();
    if (distToCell(state, next_cell, *nav.getMapData()) < CELL_REACH_THRESHOLD) {
        nav.popNextStep();
        continue;
    }

    auto [linear, angular] = computeCommand(state, next_cell, *nav.getMapData());
    sendDriveCommand(linear, angular);
}
```

### Corrected D* Lite flow

- Do not reinitialize D* Lite every frame.
- Maintain `robot_col/robot_row` start and local goal.
- Pass only changed cells to `DLite::updateAndReplan(...)`.
- Keep the local goal within the local map window.

### Corrected safety logic

- If `NavigationManager::shouldStop()` is true, do not plan or follow path.
- Clear the local path and send zero velocity.
- Only resume path following when safety flags clear and a valid path exists.

### Corrected coordinate logic

- Convert global lat/lon nodes and robot meter pose with a consistent origin.
- Use the same origin in both `StatePriorityQueue::myPlceInWorld()` and `MapLocal`.
- Avoid directly comparing degree-based GPS to meter-based EKF.
