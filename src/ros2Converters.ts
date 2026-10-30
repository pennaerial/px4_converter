import type { ExtensionContext } from "@foxglove/extension";
import type {
  FrameTransform,
  LocationFix,
  Quaternion,
  SceneEntity,
  SceneUpdate,
  Time,
} from "@foxglove/schemas";

import type { VehicleAttitude, VehicleGlobalPosition, VehicleLocalPosition } from "./types";
import {
  buildPositionCovariance,
  convertTimestamp,
  multiplyQuaternions,
  nedToEnuPosition,
} from "./utils";

const IDENTITY = { w: 1, x: 0, y: 0, z: 0 };
const ORIGIN = { x: 0, y: 0, z: 0 };
const TRACK_COLOR = { r: 0.1, g: 0.8, b: 0.9, a: 1 };
const TRAIL_BUCKET_US = 200_000;
const TRAIL_LIFETIME_SECONDS = 30;

// Filter by topic as well as schema: estimator instances share these schemas,
// but must not overwrite the selected vehicle estimate's transforms.
function framePrefix(topic: string, messageName: string): string | undefined {
  const match = new RegExp(`^/(?:(.+)/)?fmu/out/${messageName}(?:_v\\d+)?$`).exec(topic);
  if (match) {
    return match[1] ?? "px4";
  }
  // The supplied ULog-to-ROS MCAP sample uses these selected-estimate topics.
  return topic === `/px4/${messageName}` ? "px4" : undefined;
}

function validPosition(message: VehicleLocalPosition): boolean {
  return (
    message.xy_valid === true &&
    message.z_valid === true &&
    [message.x, message.y, message.z].every(Number.isFinite)
  );
}

function attitudeRotation(q: VehicleAttitude["q"]): Quaternion | undefined {
  const [w, x, y, z] = q;
  const norm = Math.hypot(w, x, y, z);
  if (!Number.isFinite(norm) || norm === 0) {
    return undefined;
  }

  // PX4 describes FRD body -> NED world; the display uses FLU body -> ENU world.
  // Change both bases: q_ENU_NED * q_NED_FRD * q_FRD_FLU.
  // Keep the full quaternion rather than splitting yaw from pitch and roll.
  return multiplyQuaternions(
    multiplyQuaternions(
      { w: 0, x: Math.SQRT1_2, y: Math.SQRT1_2, z: 0 },
      { w: w / norm, x: x / norm, y: y / norm, z: z / norm },
    ),
    { w: 0, x: 1, y: 0, z: 0 },
  );
}

function emptyEntity(timestamp: Time, prefix: string, id: string): SceneEntity {
  return {
    timestamp,
    frame_id: `${prefix}/local_origin`,
    id,
    lifetime: { sec: 0, nsec: 0 },
    frame_locked: false,
    metadata: [],
    arrows: [],
    cubes: [],
    spheres: [],
    cylinders: [],
    lines: [],
    triangles: [],
    texts: [],
    models: [],
  };
}

export function registerRos2Converters(context: ExtensionContext): void {
  context.registerMessageConverter<VehicleAttitude>({
    type: "schema",
    fromSchemaName: "px4_msgs/msg/VehicleAttitude",
    toSchemaName: "foxglove.FrameTransform",
    converter: (message, event): FrameTransform | undefined => {
      const prefix = framePrefix(event.topic, "vehicle_attitude");
      const rotation = attitudeRotation(message.q);
      if (prefix == undefined || rotation == undefined) {
        return undefined;
      }
      return {
        timestamp: convertTimestamp(message.timestamp),
        parent_frame_id: `${prefix}/position`,
        child_frame_id: `${prefix}/base_link`,
        translation: ORIGIN,
        rotation,
      };
    },
  });

  context.registerMessageConverter<VehicleLocalPosition>({
    type: "schema",
    fromSchemaName: "px4_msgs/msg/VehicleLocalPosition",
    toSchemaName: "foxglove.FrameTransform",
    converter: (message, event): FrameTransform | undefined => {
      const prefix = framePrefix(event.topic, "vehicle_local_position");
      if (prefix == undefined) {
        return undefined;
      }
      return {
        timestamp: convertTimestamp(message.timestamp),
        parent_frame_id: `${prefix}/local_origin`,
        child_frame_id: `${prefix}/position`,
        translation: validPosition(message)
          ? nedToEnuPosition(message.x, message.y, message.z)
          : ORIGIN,
        rotation: IDENTITY,
      };
    },
  });

  context.registerMessageConverter<VehicleLocalPosition>({
    type: "schema",
    fromSchemaName: "px4_msgs/msg/VehicleLocalPosition",
    toSchemaName: "foxglove.SceneUpdate",
    converter: (message, event): SceneUpdate | undefined => {
      const prefix = framePrefix(event.topic, "vehicle_local_position");
      if (prefix == undefined) {
        return undefined;
      }
      const timestamp = convertTimestamp(message.timestamp);
      if (!validPosition(message)) {
        const status = emptyEntity(timestamp, prefix, "position-status");
        status.texts.push({
          pose: { position: { x: 0, y: 0, z: 1 }, orientation: IDENTITY },
          billboard: true,
          font_size: 16,
          scale_invariant: true,
          color: { r: 1, g: 0.7, b: 0.2, a: 1 },
          text: "Position unavailable",
        });
        return { entities: [status], deletions: [] };
      }

      // Foxglove replaces entities with the same ID. Time buckets give a 5 Hz
      // breadcrumb trail; lifetime bounds it to 30 seconds without cached state.
      const bucket = Math.floor(Number(message.timestamp) / TRAIL_BUCKET_US);
      const breadcrumb = emptyEntity(timestamp, prefix, `trail-${bucket}`);
      breadcrumb.lifetime = { sec: TRAIL_LIFETIME_SECONDS, nsec: 0 };
      breadcrumb.spheres.push({
        pose: {
          position: nedToEnuPosition(message.x, message.y, message.z),
          orientation: IDENTITY,
        },
        size: { x: 0.1, y: 0.1, z: 0.1 },
        color: TRACK_COLOR,
      });
      return {
        entities: [breadcrumb],
        deletions: [{ timestamp, type: 0, id: "position-status" }], // MATCHING_ID
      };
    },
  });

  context.registerMessageConverter<VehicleGlobalPosition>({
    type: "schema",
    fromSchemaName: "px4_msgs/msg/VehicleGlobalPosition",
    toSchemaName: "foxglove.LocationFix",
    converter: (message, event): LocationFix | undefined => {
      const prefix = framePrefix(event.topic, "vehicle_global_position");
      const { lat, lon, alt, eph, epv } = message;
      if (
        prefix == undefined ||
        message.lat_lon_valid !== true ||
        message.alt_valid !== true ||
        ![lat, lon, alt].every(Number.isFinite) ||
        Math.abs(lat) > 90 ||
        Math.abs(lon) > 180
      ) {
        return undefined;
      }
      const hasCovariance =
        eph != undefined &&
        epv != undefined &&
        Number.isFinite(eph) &&
        Number.isFinite(epv) &&
        eph >= 0 &&
        epv >= 0;
      return {
        timestamp: convertTimestamp(message.timestamp),
        frame_id: `${prefix}/base_link`,
        latitude: lat,
        longitude: lon,
        altitude: alt, // Preserve PX4's metres above mean sea level (AMSL).
        position_covariance: hasCovariance
          ? buildPositionCovariance(eph, epv)
          : buildPositionCovariance(),
        position_covariance_type: hasCovariance ? 2 : 0, // DIAGONAL_KNOWN / UNKNOWN
        color: TRACK_COLOR,
      };
    },
  });
}
