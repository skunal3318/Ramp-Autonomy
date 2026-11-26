# Phase 8 Validation Sessions

This index summarizes the validation sessions conducted during Phase 8 of the Ramp-Autonomy project. Each session investigated a specific problem with the robot's behavior and documented the findings and solutions.

| session | question | outcome |
|---|---|---|
| `construction_site_descent` | why does the robot fail during descent/turnaround on the elevated level? | Wheel odometry becomes unreliable during turns, causing the robot to walk off the platform edge. Fixed with EKF localization fusion. |
| `corner_mount` | can LiDAR sensors be relocated to corner mounts without losing accuracy? | Yes, but required re-tuning of perception parameters. Forward sensor needed increased standoff height, obstacle sensor height unchanged. |
| `crest_freeze` | why does the loaded robot stall at the crest? | 5 cm geometry authoring bug in the SDF file causing unintended contact. |
| `crosstrack_diagnosis` | why does cross-track error grow on descent? | `PathAlignCritic` was never activated due to incorrect parameter configuration. |
| `delivery` | can the robot complete a full delivery mission with the current control stack? | Mission succeeds with some HALT events during tight maneuvers. Performance depends on friction coefficient. |
| `ekf_validation` | does the EKF localization fix actually solve the localization problem? | Yes - yaw error reduced from 35-40° to 3.25° during turns, position error doesn't grow unbounded. |
| `fsm_multiramp` | can the state machine handle transitions between multiple ramp segments? | Successfully handles transitions with proper state changes and path updates. |
| `halt_loop` | what causes repeated HALT events during certain maneuvers? | Combination of high wheel slip and pitch instability during aggressive maneuvers. |
| `lidar_mount_fix` | why is the LiDAR giving inconsistent readings? | Incorrect joint height causing self-occlusion. Fixed by adjusting standoff height. |
| `mesh_swap` | can the robot handle different mesh configurations? | Successfully operates with different mesh configurations, though perception may need re-tuning. |
| `path_structural` | how does the path planner handle complex structural elements? | Planner successfully navigates around obstacles and through tight spaces. |
| `payload_chute` | can the payload chute mechanism be reliably triggered? | Chute release works reliably when properly triggered, but detection needs improvement. |
| `perception_regression` | does perception performance degrade after recent changes? | Performance remains consistent with previous baselines after parameter adjustments. |

## Notes

- Some sessions revealed issues requiring fixes in separate components (e.g., EKF localization, perception parameters)
- Performance depends on environmental factors like friction coefficients
- Several sessions identified problems with wheel odometry during turning maneuvers
- Cross-referenced with [docs/retrospective.md](../../docs/retrospective.md) for additional context

For detailed information about each session, refer to the individual README.md files in each subdirectory.