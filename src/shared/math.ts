// ===========================================================================
// Math Utilities
// Vector operations, angle math, statistical functions, transforms.
// ===========================================================================

import type { Vec2, Pose2D, BoundingBox } from "./types";

// ---------------------------------------------------------------------------
// Vec2 Operations
// ---------------------------------------------------------------------------

export function vec2Add(a: Vec2, b: Vec2): Vec2 {
  return { x: a.x + b.x, y: a.y + b.y };
}

export function vec2Sub(a: Vec2, b: Vec2): Vec2 {
  return { x: a.x - b.x, y: a.y - b.y };
}

export function vec2Scale(v: Vec2, s: number): Vec2 {
  return { x: v.x * s, y: v.y * s };
}

export function vec2Dot(a: Vec2, b: Vec2): number {
  return a.x * b.x + a.y * b.y;
}

export function vec2Cross(a: Vec2, b: Vec2): number {
  return a.x * b.y - a.y * b.x;
}

export function vec2Length(v: Vec2): number {
  return Math.sqrt(v.x * v.x + v.y * v.y);
}

export function vec2LengthSq(v: Vec2): number {
  return v.x * v.x + v.y * v.y;
}

export function vec2Distance(a: Vec2, b: Vec2): number {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  return Math.sqrt(dx * dx + dy * dy);
}

export function vec2DistanceSq(a: Vec2, b: Vec2): number {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  return dx * dx + dy * dy;
}

export function vec2Normalize(v: Vec2): Vec2 {
  const len = vec2Length(v);
  if (len < 1e-12) return { x: 0, y: 0 };
  return { x: v.x / len, y: v.y / len };
}

export function vec2Rotate(v: Vec2, angle: number): Vec2 {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return { x: v.x * c - v.y * s, y: v.x * s + v.y * c };
}

export function vec2Lerp(a: Vec2, b: Vec2, t: number): Vec2 {
  return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
}

export function vec2FromAngle(angle: number): Vec2 {
  return { x: Math.cos(angle), y: Math.sin(angle) };
}

// ---------------------------------------------------------------------------
// Angle Operations
// ---------------------------------------------------------------------------

/** Normalize angle to (-PI, PI] */
export function normalizeAngle(angle: number): number {
  let a = angle % (2 * Math.PI);
  if (a > Math.PI) a -= 2 * Math.PI;
  else if (a <= -Math.PI) a += 2 * Math.PI;
  return a;
}

/** Shortest angular distance from a to b, in [-PI, PI] */
export function angleDifference(from: number, to: number): number {
  return normalizeAngle(to - from);
}

/** Circular mean of angles weighted by given weights */
export function circularMean(angles: number[], weights: number[]): number {
  let sinSum = 0;
  let cosSum = 0;
  for (let i = 0; i < angles.length; i++) {
    const w = weights[i];
    sinSum += w * Math.sin(angles[i]);
    cosSum += w * Math.cos(angles[i]);
  }
  return Math.atan2(sinSum, cosSum);
}

// ---------------------------------------------------------------------------
// Pose Operations
// ---------------------------------------------------------------------------

/** Apply a local-frame delta to a pose */
export function poseApplyDelta(pose: Pose2D, dx: number, dy: number, dtheta: number): Pose2D {
  const c = Math.cos(pose.theta);
  const s = Math.sin(pose.theta);
  return {
    x: pose.x + dx * c - dy * s,
    y: pose.y + dx * s + dy * c,
    theta: normalizeAngle(pose.theta + dtheta),
  };
}

/** Transform a point from local frame to world frame given a pose */
export function poseTransformPoint(pose: Pose2D, localPoint: Vec2): Vec2 {
  const c = Math.cos(pose.theta);
  const s = Math.sin(pose.theta);
  return {
    x: pose.x + localPoint.x * c - localPoint.y * s,
    y: pose.y + localPoint.x * s + localPoint.y * c,
  };
}

/** Transform a point from world frame to a pose's local frame */
export function poseInverseTransformPoint(pose: Pose2D, worldPoint: Vec2): Vec2 {
  const dx = worldPoint.x - pose.x;
  const dy = worldPoint.y - pose.y;
  const c = Math.cos(-pose.theta);
  const s = Math.sin(-pose.theta);
  return {
    x: dx * c - dy * s,
    y: dx * s + dy * c,
  };
}

/** Compute the relative transform from pose A to pose B */
export function poseRelative(from: Pose2D, to: Pose2D): Pose2D {
  const dx = to.x - from.x;
  const dy = to.y - from.y;
  const c = Math.cos(-from.theta);
  const s = Math.sin(-from.theta);
  return {
    x: dx * c - dy * s,
    y: dx * s + dy * c,
    theta: normalizeAngle(to.theta - from.theta),
  };
}

/** Compose two transforms: result = a then b */
export function poseCompose(a: Pose2D, b: Pose2D): Pose2D {
  const c = Math.cos(a.theta);
  const s = Math.sin(a.theta);
  return {
    x: a.x + b.x * c - b.y * s,
    y: a.y + b.x * s + b.y * c,
    theta: normalizeAngle(a.theta + b.theta),
  };
}

// ---------------------------------------------------------------------------
// Scalar Utilities
// ---------------------------------------------------------------------------

export function clamp(value: number, min: number, max: number): number {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

export function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

// ---------------------------------------------------------------------------
// Random / Statistical
// ---------------------------------------------------------------------------

/** Gaussian random using Box-Muller transform */
export function gaussianRandom(mean: number, stddev: number): number {
  let u1: number;
  do { u1 = Math.random(); } while (u1 === 0);
  const u2 = Math.random();
  const z = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
  return mean + stddev * z;
}

/** Uniform random in [min, max) */
export function uniformRandom(min: number, max: number): number {
  return min + Math.random() * (max - min);
}

/** Weighted random selection. Returns index. */
export function weightedRandomIndex(weights: number[]): number {
  let total = 0;
  for (let i = 0; i < weights.length; i++) total += weights[i];
  let r = Math.random() * total;
  for (let i = 0; i < weights.length; i++) {
    r -= weights[i];
    if (r <= 0) return i;
  }
  return weights.length - 1;
}

// ---------------------------------------------------------------------------
// Bounding Box
// ---------------------------------------------------------------------------

export function bboxContains(box: BoundingBox, point: Vec2): boolean {
  return point.x >= box.min.x && point.x <= box.max.x &&
         point.y >= box.min.y && point.y <= box.max.y;
}

export function bboxOverlaps(a: BoundingBox, b: BoundingBox): boolean {
  return a.min.x <= b.max.x && a.max.x >= b.min.x &&
         a.min.y <= b.max.y && a.max.y >= b.min.y;
}

export function bboxExpand(box: BoundingBox, margin: number): BoundingBox {
  return {
    min: { x: box.min.x - margin, y: box.min.y - margin },
    max: { x: box.max.x + margin, y: box.max.y + margin },
  };
}

export function weightedMean(values: number[], weights: number[]): number {
  if (values.length === 0) return 0;
  let weightedSum = 0;
  let weightSum = 0;
  for (let i = 0; i < values.length; i++) {
    const w = i < weights.length ? weights[i] : 1;
    weightedSum += values[i] * w;
    weightSum += w;
  }
  return weightSum > 0 ? weightedSum / weightSum : 0;
}

export function exponentialDecay(lambda: number, t: number): number {
  return Math.exp(-lambda * t);
}

export function standardDeviation(values: number[]): number {
  if (values.length === 0) return 0;
  const mean = values.reduce((s, v) => s + v, 0) / values.length;
  const variance = values.reduce((s, v) => s + (v - mean) ** 2, 0) / values.length;
  return Math.sqrt(variance);
}
