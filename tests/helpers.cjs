const assert = require("node:assert/strict");

function near(actual, expected, tolerance = 1e-10) {
  assert.ok(Number.isFinite(actual), `Expected finite value, got ${actual}`);
  assert.ok(
    Math.abs(actual - expected) <= tolerance,
    `Expected ${actual} to equal ${expected} within ${tolerance}`,
  );
}

function multiplyMatrices(left, right) {
  return left.map((row) =>
    right[0].map((_, column) =>
      row.reduce((sum, value, inner) => sum + value * right[inner][column], 0),
    ),
  );
}

function quaternionMatrix(q) {
  const { w, x, y, z } = q;
  const norm = w * w + x * x + y * y + z * z;
  return [
    [
      1 - (2 * (y * y + z * z)) / norm,
      (2 * (x * y - z * w)) / norm,
      (2 * (x * z + y * w)) / norm,
    ],
    [
      (2 * (x * y + z * w)) / norm,
      1 - (2 * (x * x + z * z)) / norm,
      (2 * (y * z - x * w)) / norm,
    ],
    [
      (2 * (x * z - y * w)) / norm,
      (2 * (y * z + x * w)) / norm,
      1 - (2 * (x * x + y * y)) / norm,
    ],
  ];
}

// Independent matrix reference: body FLU -> body FRD -> world NED -> world ENU.
// This intentionally does not use the extension's quaternion conversion helper.
function expectedEnuMatrix([w, x, y, z]) {
  return multiplyMatrices(
    multiplyMatrices(
      [
        [0, 1, 0],
        [1, 0, 0],
        [0, 0, -1],
      ],
      quaternionMatrix({ w, x, y, z }),
    ),
    [
      [1, 0, 0],
      [0, -1, 0],
      [0, 0, -1],
    ],
  );
}

function assertMatrix(actual, expected) {
  for (let row = 0; row < 3; row++) {
    for (let column = 0; column < 3; column++) {
      near(actual[row][column], expected[row][column]);
    }
  }
}

function quaternionFromEuler(roll, pitch, yaw) {
  const cr = Math.cos(roll / 2),
    sr = Math.sin(roll / 2);
  const cp = Math.cos(pitch / 2),
    sp = Math.sin(pitch / 2);
  const cy = Math.cos(yaw / 2),
    sy = Math.sin(yaw / 2);
  return [
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
  ];
}

module.exports = {
  assertMatrix,
  expectedEnuMatrix,
  near,
  quaternionFromEuler,
  quaternionMatrix,
};
