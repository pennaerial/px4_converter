const assert = require("node:assert/strict");
const test = require("node:test");

const { registerRos2Converters } = require("../.test-dist/ros2Converters.js");
const {
  assertMatrix,
  expectedEnuMatrix,
  near,
  quaternionFromEuler,
  quaternionMatrix,
} = require("./helpers.cjs");

const registrations = [];
registerRos2Converters({
  registerMessageConverter: (registration) => registrations.push(registration),
});
const schema = (name) => `px4_msgs/msg/${name}`;
const output = (name) => `foxglove.${name}`;
const epochUs = 1793348245748157n;

function convert(inputSchema, outputSchema, message, topic) {
  const registration = registrations.find(
    (entry) =>
      entry.fromSchemaName === schema(inputSchema) &&
      entry.toSchemaName === output(outputSchema),
  );
  assert.ok(registration, `${inputSchema} -> ${outputSchema} is registered`);
  return registration.converter(message, {
    topic,
    schemaName: schema(inputSchema),
    receiveTime: { sec: 99, nsec: 0 },
    message,
    sizeInBytes: 0,
  });
}

function attitude(
  q = [1, 0, 0, 0],
  topic = "/uav_0/fmu/out/vehicle_attitude",
  timestamp = epochUs,
) {
  return convert("VehicleAttitude", "FrameTransform", { timestamp, q }, topic);
}

function local(
  overrides = {},
  outputSchema = "FrameTransform",
  topic = "/uav_0/fmu/out/vehicle_local_position_v1",
) {
  return convert(
    "VehicleLocalPosition",
    outputSchema,
    {
      timestamp: epochUs,
      x: 10,
      y: 20,
      z: -3,
      xy_valid: true,
      z_valid: true,
      heading: 0,
      ...overrides,
    },
    topic,
  );
}

function global(
  overrides = {},
  topic = "/uav_0/fmu/out/vehicle_global_position",
) {
  return convert(
    "VehicleGlobalPosition",
    "LocationFix",
    {
      timestamp: epochUs,
      lat: 39.5,
      lon: -75.5,
      alt: 100,
      alt_ellipsoid: 70,
      lat_lon_valid: true,
      alt_valid: true,
      eph: 2,
      epv: 3,
      ...overrides,
    },
    topic,
  );
}

test("registers only the four ROS visualization conversions", () => {
  assert.equal(registrations.length, 4);
  assert.ok(registrations.every((entry) => entry.type === "schema"));
  assert.equal(
    new Set(
      registrations.map(
        (entry) => `${entry.fromSchemaName}:${entry.toSchemaName}`,
      ),
    ).size,
    4,
  );
});

test("position and attitude form a continuous namespace-specific transform chain", () => {
  const position = local();
  const rotation = attitude();
  assert.equal(position.parent_frame_id, "uav_0/local_origin");
  assert.equal(position.child_frame_id, "uav_0/position");
  assert.equal(rotation.parent_frame_id, position.child_frame_id);
  assert.equal(rotation.child_frame_id, "uav_0/base_link");
  assert.deepEqual(position.translation, { x: 20, y: 10, z: 3 });
  assert.deepEqual(position.rotation, { w: 1, x: 0, y: 0, z: 0 });
  assert.deepEqual(rotation.translation, { x: 0, y: 0, z: 0 });
});

test("supports bare, nested, versioned, and converted-sample topic names without mixing vehicles", () => {
  const paths = [
    ["/fmu/out/vehicle_attitude", "px4"],
    ["/uav_3/fmu/out/vehicle_attitude_v2", "uav_3"],
    ["/fleet/uav_4/fmu/out/vehicle_attitude", "fleet/uav_4"],
    ["/px4/vehicle_attitude", "px4"],
  ];
  for (const [topic, prefix] of paths) {
    assert.equal(
      attitude(undefined, topic).child_frame_id,
      `${prefix}/base_link`,
    );
  }
  assert.notEqual(
    attitude(undefined, "/uav_0/fmu/out/vehicle_attitude").child_frame_id,
    attitude(undefined, "/uav_1/fmu/out/vehicle_attitude").child_frame_id,
  );
  assert.equal(
    local({}, "FrameTransform", "/px4/vehicle_local_position").child_frame_id,
    "px4/position",
  );
  assert.equal(
    global({}, "/px4/vehicle_global_position").frame_id,
    "px4/base_link",
  );
});

test("ignores estimator instances, command inputs, and mismatched topic names", () => {
  for (const topic of [
    "/px4/estimator_attitude_0",
    "/uav_0/fmu/out/estimator_attitude_0",
    "/uav_0/fmu/in/vehicle_attitude",
    "/uav_0/fmu/out/vehicle_attitude_setpoint",
    "/unrelated/vehicle_attitude",
  ]) {
    assert.equal(attitude(undefined, topic), undefined, topic);
  }
  for (const outputSchema of ["FrameTransform", "SceneUpdate"]) {
    assert.equal(
      local({}, outputSchema, "/px4/estimator_local_position_1"),
      undefined,
    );
  }
  assert.equal(global({}, "/px4/estimator_global_position_0"), undefined);
});

test("uses the message clock in microseconds for number and uint64 BigInt inputs", () => {
  const expected = { sec: 1793348245, nsec: 748157000 };
  assert.deepEqual(attitude().timestamp, expected);
  assert.deepEqual(
    attitude(undefined, undefined, Number(epochUs)).timestamp,
    expected,
  );
  assert.deepEqual(local().timestamp, expected);
  assert.deepEqual(global().timestamp, expected);
  assert.deepEqual(attitude(undefined, undefined, 0n).timestamp, {
    sec: 0,
    nsec: 0,
  });
  assert.deepEqual(attitude(undefined, undefined, 999999n).timestamp, {
    sec: 0,
    nsec: 999999000,
  });
  assert.deepEqual(attitude(undefined, undefined, 1000000n).timestamp, {
    sec: 1,
    nsec: 0,
  });
});

test("level north-facing PX4 attitude faces north in the ENU view", () => {
  assertMatrix(quaternionMatrix(attitude().rotation), [
    [0, -1, 0],
    [1, 0, 0],
    [0, 0, 1],
  ]);
});

test("east-facing PX4 attitude is the identity FLU-to-ENU orientation", () => {
  const q = quaternionFromEuler(0, 0, Math.PI / 2);
  assertMatrix(quaternionMatrix(attitude(q).rotation), [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ]);
});

for (const [name, angles] of [
  ["roll", [Math.PI / 6, 0, 0]],
  ["pitch", [0, Math.PI / 6, 0]],
  ["yaw", [0, 0, -Math.PI / 3]],
  ["combined steep attitude", [0.63, -1.13, 2.17]],
  ["vertical tailsitter attitude", [0.2, Math.PI / 2, 0.7]],
  ["inverted attitude", [Math.PI, 0.4, -1.4]],
]) {
  test(`preserves the complete ${name} rotation with independent matrix verification`, () => {
    const q = quaternionFromEuler(...angles);
    const converted = attitude(q).rotation;
    near(Math.hypot(converted.w, converted.x, converted.y, converted.z), 1);
    assertMatrix(quaternionMatrix(converted), expectedEnuMatrix(q));
  });
}

test("normalizes valid quaternions and accepts ROS typed arrays", () => {
  const q = quaternionFromEuler(0.3, 0.4, 0.5);
  assertMatrix(
    quaternionMatrix(attitude(q.map((value) => value * 2)).rotation),
    expectedEnuMatrix(q),
  );
  const typed = new Float32Array(q);
  assertMatrix(
    quaternionMatrix(attitude(typed).rotation),
    expectedEnuMatrix(typed),
  );
});

test("does not publish transforms for unusable attitude values", () => {
  for (const q of [
    [0, 0, 0, 0],
    [NaN, 0, 0, 0],
    [1, Infinity, 0, 0],
  ]) {
    assert.equal(attitude(q), undefined);
  }
});

test("invalid position stays at the display origin, including altitude, without changing rotation", () => {
  for (const fields of [
    { xy_valid: false },
    { z_valid: false },
    { x: NaN },
    { y: Infinity },
    { z: NaN },
  ]) {
    assert.deepEqual(local(fields).translation, { x: 0, y: 0, z: 0 });
    assert.deepEqual(local(fields).rotation, { w: 1, x: 0, y: 0, z: 0 });
    assert.equal(
      local(fields, "SceneUpdate").entities.filter(
        (entity) => entity.spheres.length > 0,
      ).length,
      0,
    );
  }
  const q = quaternionFromEuler(0.3, 0.5, 0.7);
  assertMatrix(quaternionMatrix(attitude(q).rotation), expectedEnuMatrix(q));
});

test("valid position emits an ENU breadcrumb in the fixed frame with bounded history", () => {
  const scene = local({}, "SceneUpdate");
  const breadcrumb = scene.entities.find((entity) => entity.spheres.length > 0);
  assert.ok(breadcrumb);
  assert.equal(breadcrumb.frame_id, "uav_0/local_origin");
  assert.equal(breadcrumb.spheres.length, 1);
  assert.deepEqual(breadcrumb.spheres[0].pose.position, { x: 20, y: 10, z: 3 });
  assert.deepEqual(breadcrumb.lifetime, { sec: 30, nsec: 0 });
  const idFor = (timestamp) =>
    local({ timestamp }, "SceneUpdate").entities.find(
      (entity) => entity.spheres.length > 0,
    ).id;
  assert.equal(idFor(1000000n), idFor(1199999n));
  assert.notEqual(idFor(1199999n), idFor(1200000n));
});

test("position-unavailable annotation appears when invalid and is cleared when valid", () => {
  const invalid = local({ xy_valid: false }, "SceneUpdate");
  assert.equal(invalid.entities.length, 1);
  assert.equal(invalid.entities[0].id, "position-status");
  assert.ok(
    JSON.stringify(invalid.entities[0])
      .toLowerCase()
      .includes("position unavailable"),
  );
  const valid = local({}, "SceneUpdate");
  assert.equal(
    valid.entities.filter((entity) => entity.id === "position-status").length,
    0,
  );
  assert.ok(valid.deletions.some((entry) => entry.id === "position-status"));
  // Repeat in the opposite playback order: conversion cannot rely on cached validity state.
  assert.deepEqual(local({ xy_valid: false }, "SceneUpdate"), invalid);
  assert.deepEqual(local({}, "SceneUpdate"), valid);
});

test("map output uses valid coordinates and preserves PX4 AMSL altitude", () => {
  const fix = global();
  assert.equal(fix.latitude, 39.5);
  assert.equal(fix.longitude, -75.5);
  assert.equal(fix.altitude, 100);
  assert.equal(fix.frame_id, "uav_0/base_link");
  assert.deepEqual(fix.position_covariance, [4, 0, 0, 0, 4, 0, 0, 0, 9]);
  assert.equal(fix.position_covariance_type, 2);
});

test("map output is suppressed for invalid estimates or impossible geographic coordinates", () => {
  for (const fields of [
    { lat_lon_valid: false },
    { alt_valid: false },
    { lat: NaN },
    { lon: Infinity },
    { lat: 91 },
    { lon: -181 },
    { alt: NaN },
  ]) {
    assert.equal(global(fields), undefined, JSON.stringify(fields));
  }
  assert.ok(global({ lat: 0, lon: 0, alt: 0 }));
});

test("converters do not mutate source messages or retain state across playback seeks", () => {
  const q = Object.freeze([1, 0, 0, 0]);
  const input = Object.freeze({ timestamp: epochUs, q });
  const first = convert(
    "VehicleAttitude",
    "FrameTransform",
    input,
    "/uav_0/fmu/out/vehicle_attitude",
  );
  attitude(quaternionFromEuler(1, 2, 3));
  const again = convert(
    "VehicleAttitude",
    "FrameTransform",
    input,
    "/uav_0/fmu/out/vehicle_attitude",
  );
  assert.deepEqual(first, again);
  assert.deepEqual(input.q, [1, 0, 0, 0]);
});

test("activation retains the four original ULog converters alongside ROS support", () => {
  const { activate } = require("../.test-dist/index.js");
  const all = [];
  activate({ registerMessageConverter: (entry) => all.push(entry) });
  assert.equal(all.length, 8);
  assert.deepEqual(
    all
      .filter((entry) => !entry.fromSchemaName.startsWith("px4_msgs/"))
      .map((entry) => `${entry.fromSchemaName}:${entry.toSchemaName}`)
      .sort(),
    [
      "vehicle_attitude:foxglove.FrameTransform",
      "vehicle_global_position:foxglove.LocationFix",
      "vehicle_local_position:foxglove.FrameTransform",
      "vehicle_local_position:foxglove.PosesInFrame",
    ],
  );
});

test("unknown covariance stays unknown when either error estimate is unavailable or invalid", () => {
  for (const fields of [
    { eph: undefined },
    { epv: undefined },
    { eph: NaN },
    { epv: Infinity },
    { eph: -1 },
    { epv: -1 },
  ]) {
    const fix = global(fields);
    assert.equal(fix.position_covariance_type, 0);
    assert.deepEqual(fix.position_covariance, [0, 0, 0, 0, 0, 0, 0, 0, 0]);
  }
  const precise = global({ eph: 0, epv: 0 });
  assert.equal(precise.position_covariance_type, 2);
  assert.deepEqual(precise.position_covariance, [0, 0, 0, 0, 0, 0, 0, 0, 0]);
});
