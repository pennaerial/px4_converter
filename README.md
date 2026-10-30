# PennAIR PX4 ROS 2 Converter

A Foxglove extension for viewing ROS 2 MCAP recordings from PX4. It adds full aircraft orientation, local position, a short 3D flight trail, and a geographic map track using the standard 3D and Map panels. Conversion happens while viewing; the original recording is unchanged.

The fork retains the upstream ULog schema converters. The new ROS 2 converters use separate registrations and do not change the upstream ULog behavior. Use this fork in place of the upstream PX4 Converter extension to avoid duplicate ULog converter registrations.

## Inputs and outputs

| Recorded topic (example namespace) | ROS schema | Foxglove output |
| --- | --- | --- |
| `/uav_0/fmu/out/vehicle_attitude` | `px4_msgs/msg/VehicleAttitude` | `foxglove.FrameTransform`: full orientation |
| `/uav_0/fmu/out/vehicle_local_position_v1` | `px4_msgs/msg/VehicleLocalPosition` | `foxglove.FrameTransform`: position; `foxglove.SceneUpdate`: trail and position status |
| `/uav_0/fmu/out/vehicle_global_position` | `px4_msgs/msg/VehicleGlobalPosition` | `foxglove.LocationFix`: geographic position |

Attitude and local position are required for the 3D view. Global position is optional and supplies the Map panel. The extension does not require odometry, vehicle status, time synchronization topics, or mission execution.

The converters accept `/fmu/out/...` and namespaced equivalents, with optional `_vN` topic suffixes. They also accept the selected `/px4/vehicle_*` topics in the supplied ULog-derived ROS sample. Other estimator instances sharing these schemas are ignored. For another namespace or topic version, update the topic selections and frame names in the layout.

## Coordinates and validity

The transform chain for `/uav_0/fmu/out/...` is:

```text
uav_0/local_origin (east, north, up)
  └─ uav_0/position (translation only)
      └─ uav_0/base_link (forward, left, up; full attitude)
```

PX4 local position `(north, east, down)` becomes `(east, north, up)`. The full PX4 `w,x,y,z` attitude quaternion is converted from FRD-to-NED into FLU-to-ENU; yaw is not taken from local position. Body axes keep the same meaning through a tailsitter transition. No flight-mode-dependent rotation is applied.

With invalid horizontal or vertical position, or non-finite coordinates, translation is fixed at `(0,0,0)`, attitude continues, and the 3D view shows **Position unavailable**. No breadcrumb is added at the fallback origin. Previously valid breadcrumbs expire normally. The origin is the local estimator origin, not necessarily the vehicle's home position.

The 3D trail uses cyan position markers, with one marker updated per 200 ms interval and a 30-second lifetime. It accumulates during playback; after seeking, play forward to rebuild the recent trail. This is a position history, not a reconstructed trajectory through unrecorded intervals. Position-estimate resets are preserved, not smoothed away.

The map uses valid fused `VehicleGlobalPosition` data. Invalid or non-finite fixes are omitted, leaving prior valid history visible. Altitude preserves PX4's `alt` in metres above mean sea level; it is not changed to ellipsoid height. Position covariance uses the squared `eph` and `epv` values when both are finite and nonnegative. The map does not feed or relocate the local 3D position.

Timestamps retain the source message clock, converted from microseconds to seconds/nanoseconds. This matches the supplied Pi bag with XRCE time synchronization enabled. No additional boot-to-epoch offset is applied. The converter does not repair unsynchronized clocks.

## Build and install

Use Node.js/npm on the computer used for Foxglove development:

```sh
npm ci
npm run typecheck
npm test
npm run lint
npm run package
```

Install `dist/px4-ros2-converter-0.2.0.foxe` by dragging it onto an open Foxglove visualization page. For desktop development, `npm run local-install` builds and installs it locally; reload Foxglove after updating it. A local extension requires a developer seat. See [Foxglove local development](https://docs.foxglove.dev/docs/extensions/local-development).

This is a message converter, so it adds capabilities to existing panels rather than adding a new custom panel.

## Use the supplied layout

1. Install the extension and open the MCAP.
2. Click the layout name at the top right (for example **Default**) to open the Layouts menu, then choose **Import from file...**. Select [`layouts/px4-ros2.json`](layouts/px4-ros2.json) for Pi bags, or [`layouts/px4-ros2-sample.json`](layouts/px4-ros2-sample.json) for the earlier `log_71_2026-10-25-17-03-00.mcap` sample. Import layout JSON through this menu; dragging it into the data viewer is treated as an unsupported recording format.
3. Play the recording. The 3D panel shows the aircraft's body axes and recent position markers. The map shows valid global positions up to the playback time.

The Pi layout is configured for `uav_0` and the topic names in the table above. The sample layout uses `/px4/vehicle_*` topics and `px4/` frames. The camera follows position only, so roll, pitch, and yaw remain visible. Red is body forward, green is body left, and blue is body up. The stationary grid has one-metre spacing. A custom aircraft mesh is not required.

The indoor sample has invalid horizontal position and no global-position topic: rotation at a fixed origin with **Position unavailable**, and no map track, is the expected result.

For a different namespace, select its `base_link` as the 3D display frame and its `local_origin` as the fixed/grid frame; enable its local-position topic and select `foxglove.SceneUpdate` if a conversion selector is shown. In the Map panel, select its global-position topic. Unnamespaced `/fmu/out/...` and the `/px4/...` sample use the `px4/` frame prefix.

If orientation is visible but the trail or status text is missing, enable the local-position topic in the 3D panel and select the `foxglove.SceneUpdate` conversion. Transforms are consumed separately. The trail is intentionally limited to recent playback; use the Map panel's time-range setting for a longer geographic history.

## Record the required topics on the Pi

Source the ROS environment and the built workspace before recording. Including global position is necessary for a map; the earlier indoor recording filter did not include it.

```sh
mkdir -p ~/flight-bags
bag_dir="$HOME/flight-bags/px4_visualization_$(date +%Y%m%d_%H%M%S)"
ros2 bag record -s mcap -o "$bag_dir" \
  --regex '^(/.*)?/fmu/out/(vehicle_attitude|vehicle_local_position|vehicle_global_position)(_v[0-9]+)?$'
```

Stop recording with Ctrl+C and inspect `ros2 bag info "$bag_dir"`. The recorder and extension do not start or arm a mission.

## Share with the organization

After local playback verification, an organization admin can upload the `.foxe` package in Foxglove's organization extension settings. Organization extensions are installed automatically for members. Share the imported layout separately using **Share with team...** in its menu. Installing or publishing an extension does not upload recordings. See [publishing extensions](https://docs.foxglove.dev/docs/extensions/publish) and [sharing layouts](https://docs.foxglove.dev/docs/visualization/layouts).

## Playback verification

Automated tests cover coordinate conversion, quaternion composition, invalid inputs, topic/namespace selection, trail expiration fields, map fixes, and timestamp conversion. Before distributing an update, verify in Foxglove:

- The supplied indoor bag shows roll, pitch, then yaw at the fixed origin with a position-unavailable label.
- A valid-position recording shows local movement and recent breadcrumbs; a recorded valid global-position topic produces the geographic track.
- Pause and seek backward or forward. Orientation and position follow the selected time; the short trail rebuilds as playback resumes.
- The local-position topic renders as `foxglove.SceneUpdate`, the body axes remain visible, and the map follows the intended global-position topic.

The layout uses standard Foxglove panel settings and no user scripts. UI playback still needs to be checked in the installed Foxglove version; passing converter unit tests alone does not confirm the rendered application behavior.

## Attribution

Forked from [Foxglove PX4 Converter](https://github.com/foxglove/px4_converter), licensed under the MIT license.
