#!/usr/bin/env node
'use strict';

// Deterministic synthetic stress test for the radius-10 occupancy detector.
// This is deliberately dependency-free so it can run with the repository's
// existing Node.js installation. It models raised rail/sleeper geometry under
// a directional Lambertian light; it is not intended to mimic a particular
// camera or layout material.

const fs = require('fs');
const path = require('path');

const SIZE = 41;
const CENTRE = 20;
const RADIUS = 10;
const TRACK_ANGLE_DEGREES = 8;
const FLOOR = 20;
const DIRECTIONS = 12;
const BANDS = 3;
const BUCKETS = DIRECTIONS * BANDS;

const clamp = (value, low, high) => Math.max(low, Math.min(high, value));
const gaussian = (distance, sigma) => Math.exp(-(distance * distance) / (2 * sigma * sigma));
const pixel = (x, y) => y * SIZE + x;

function scene(vehicle = false) {
  const height = new Float64Array(SIZE * SIZE);
  const albedo = new Float64Array(SIZE * SIZE);
  for (let y = 0; y < SIZE; y++) for (let x = 0; x < SIZE; x++) {
    const imageX = x - CENTRE;
    const imageY = y - CENTRE;
    const trackAngle = TRACK_ANGLE_DEGREES * Math.PI / 180;
    const rx = imageX * Math.cos(trackAngle) + imageY * Math.sin(trackAngle);
    const ry = -imageX * Math.sin(trackAngle) + imageY * Math.cos(trackAngle);
    const rail = gaussian(Math.abs(rx) - 5.0, 0.72);
    const sleeperDistance = Math.abs((((ry + 3.5) % 7) + 7) % 7 - 3.5);
    const sleeper = gaussian(sleeperDistance, 0.9) * gaussian(rx, 8.0);
    const ballast = 0.025 * Math.sin(x * 1.71 + y * 0.83) + 0.018 * Math.sin(x * 0.47 - y * 2.13);
    let h = 1.8 * rail + 0.72 * sleeper + ballast;
    let a = 0.34 + 0.24 * rail + 0.11 * sleeper + ballast * 0.7;
    // A simple dark vehicle body covers the central track while retaining two
    // long, rail-like vertical edges. This is a sensitivity control, not a
    // photorealistic wagon.
    if (vehicle && Math.abs(rx) <= 7 && Math.abs(ry) <= 6) {
      h = 2.7 + 0.08 * Math.cos(rx * 0.7);
      a = 0.20 + (Math.abs(rx) > 6 ? 0.13 : 0);
    }
    height[pixel(x, y)] = h;
    albedo[pixel(x, y)] = a;
  }
  return { height, albedo };
}

function render(model, azimuthDegrees, elevationDegrees, strength) {
  const raw = new Float64Array(SIZE * SIZE);
  const az = azimuthDegrees * Math.PI / 180;
  const el = elevationDegrees * Math.PI / 180;
  const lx = Math.cos(el) * Math.cos(az);
  const ly = Math.cos(el) * Math.sin(az);
  const lz = Math.sin(el);
  for (let y = 1; y < SIZE - 1; y++) for (let x = 1; x < SIZE - 1; x++) {
    const dzdx = (model.height[pixel(x + 1, y)] - model.height[pixel(x - 1, y)]) / 2;
    const dzdy = (model.height[pixel(x, y + 1)] - model.height[pixel(x, y - 1)]) / 2;
    const length = Math.hypot(dzdx, dzdy, 1);
    const diffuse = Math.max(0, (-dzdx * lx - dzdy * ly + lz) / length);
    const illumination = 0.20 + strength * 1.28 * diffuse;
    raw[pixel(x, y)] = 255 * model.albedo[pixel(x, y)] * illumination;
  }
  // Small optical blur followed by deterministic low-level sensor texture.
  const image = new Uint8Array(SIZE * SIZE);
  for (let y = 1; y < SIZE - 1; y++) for (let x = 1; x < SIZE - 1; x++) {
    let sum = 0;
    for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
      const weight = dx === 0 && dy === 0 ? 4 : (dx === 0 || dy === 0 ? 2 : 1);
      sum += weight * raw[pixel(x + dx, y + dy)];
    }
    const noise = 1.3 * Math.sin(x * 12.9898 + y * 78.233);
    image[pixel(x, y)] = Math.round(clamp(sum / 16 + noise, 0, 255));
  }
  return image;
}

function scharr(image, x, y) {
  const p = pixel(x, y);
  const rawX = -3 * image[p - SIZE - 1] + 3 * image[p - SIZE + 1]
    - 10 * image[p - 1] + 10 * image[p + 1]
    - 3 * image[p + SIZE - 1] + 3 * image[p + SIZE + 1];
  const rawY = -3 * image[p - SIZE - 1] - 10 * image[p - SIZE] - 3 * image[p - SIZE + 1]
    + 3 * image[p + SIZE - 1] + 10 * image[p + SIZE] + 3 * image[p + SIZE + 1];
  const quarter = value => value < 0 ? -Math.floor((-value + 2) / 4) : Math.floor((value + 2) / 4);
  return [quarter(rawX), quarter(rawY)];
}

function samples(image, magnitudeFunction) {
  const result = [];
  let maximum = 0;
  for (let y = CENTRE - RADIUS; y <= CENTRE + RADIUS; y++) {
    for (let x = CENTRE - RADIUS; x <= CENTRE + RADIUS; x++) {
      const dx = x - CENTRE;
      const dy = y - CENTRE;
      if (dx * dx + dy * dy > RADIUS * RADIUS) continue;
      const [gx, gy] = scharr(image, x, y);
      const magnitude = magnitudeFunction(gx, gy);
      maximum = Math.max(maximum, magnitude);
      result.push({ x, y, gx, gy, magnitude });
    }
  }
  return { result, maximum };
}

const l1Magnitude = (gx, gy) => Math.abs(gx) + Math.abs(gy);
const approximateMagnitude = (gx, gy) => {
  const hi = Math.max(Math.abs(gx), Math.abs(gy));
  const lo = Math.min(Math.abs(gx), Math.abs(gy));
  return hi + 3 * lo / 8;
};

function directionAndPosition(sample, canonicalAngle = 0) {
  let physical = Math.atan2(sample.gy, sample.gx) * 180 / Math.PI + 90;
  if (physical < 0) physical += 180;
  if (physical >= 180) physical -= 180;
  let relative = physical - canonicalAngle;
  if (relative < 0) relative += 180;
  if (relative >= 180) relative -= 180;
  const coordinate = relative / 15;
  const lower = Math.floor(coordinate) % DIRECTIONS;
  const fraction = coordinate - Math.floor(coordinate);
  const bandFor = direction => {
    const gradientAngle = (canonicalAngle + direction * 15 + 90) * Math.PI / 180;
    const rho = (sample.x - CENTRE) * Math.cos(gradientAngle)
      + (sample.y - CENTRE) * Math.sin(gradientAngle);
    return rho < -RADIUS / 3 ? 0 : (rho > RADIUS / 3 ? 2 : 1);
  };
  return { lower, upper: (lower + 1) % DIRECTIONS, fraction, bandFor };
}

function currentDescriptor(image, referenceMaximum = null) {
  const scan = samples(image, l1Magnitude);
  const cutoff = Math.max(FLOOR, (referenceMaximum === null ? scan.maximum : referenceMaximum) / 5);
  const buckets = new Float64Array(BUCKETS);
  let edges = 0;
  for (const sample of scan.result) {
    if (sample.magnitude < cutoff) continue;
    const location = directionAndPosition(sample);
    const direction = (location.lower + (location.fraction >= 0.5 ? 1 : 0)) % DIRECTIONS;
    buckets[direction * BANDS + location.bandFor(direction)]++;
    edges++;
  }
  return { buckets, total: edges, edges, maximum: scan.maximum, cutoff, textured: edges >= 8 };
}

function proposedDescriptor(image, referenceMaximum = null, canonicalAngle = 0, soft = true) {
  const scan = samples(image, approximateMagnitude);
  // Retain a noise floor and baseline-relative cutoff for this first isolated
  // comparison. Later experiments can vary this independently.
  const cutoff = Math.max(FLOOR, (referenceMaximum === null ? scan.maximum : referenceMaximum) / 5);
  const buckets = new Float64Array(BUCKETS);
  let energy = 0;
  let edges = 0;
  for (const sample of scan.result) {
    if (sample.magnitude < cutoff) continue;
    const location = directionAndPosition(sample, canonicalAngle);
    if (soft) {
      const lowerWeight = sample.magnitude * (1 - location.fraction);
      const upperWeight = sample.magnitude * location.fraction;
      buckets[location.lower * BANDS + location.bandFor(location.lower)] += lowerWeight;
      buckets[location.upper * BANDS + location.bandFor(location.upper)] += upperWeight;
    } else {
      // Centres are 0, 15, ... degrees, so bucket zero spans -7.5..+7.5.
      const direction = (location.lower + (location.fraction >= 0.5 ? 1 : 0)) % DIRECTIONS;
      buckets[direction * BANDS + location.bandFor(direction)] += sample.magnitude;
    }
    energy += sample.magnitude;
    edges++;
  }
  return { buckets, total: energy, edges, maximum: scan.maximum, cutoff, textured: edges >= 8 };
}

function calibratedCanonicalAngle(image) {
  const scan = samples(image, approximateMagnitude);
  const cutoff = Math.max(FLOOR, scan.maximum / 5);
  let bestAngle = 0;
  let bestSupport = -1;
  // Search half-degree centres. This does not assume the strongest structure
  // is semantically a rail; it only establishes a stable local coordinate.
  for (let centre = 0; centre < 180; centre += 0.5) {
    let support = 0;
    for (const sample of scan.result) {
      if (sample.magnitude < cutoff) continue;
      let physical = Math.atan2(sample.gy, sample.gx) * 180 / Math.PI + 90;
      if (physical < 0) physical += 180;
      if (physical >= 180) physical -= 180;
      let distance = Math.abs(physical - centre);
      distance = Math.min(distance, 180 - distance);
      if (distance <= 7.5) support += sample.magnitude;
    }
    if (support > bestSupport) {
      bestSupport = support;
      bestAngle = centre;
    }
  }
  return bestAngle;
}

function distributionDistance(a, b) {
  if (!a.total || !b.total) return 1;
  let sum = 0;
  for (let index = 0; index < BUCKETS; index++) {
    sum += Math.abs(a.buckets[index] / a.total - b.buckets[index] / b.total);
  }
  return Math.min(1, sum / 2);
}

function currentScore(reference, live) {
  if (!reference.textured && !live.textured) return 0;
  if (reference.textured !== live.textured) return 1;
  const density = Math.abs(reference.edges - live.edges) / Math.max(reference.edges, live.edges);
  return Math.max(distributionDistance(reference, live), density);
}

function proposedScore(reference, live) {
  if (!reference.textured && !live.textured) return 0;
  if (reference.textured !== live.textured) return 1;
  return distributionDistance(reference, live);
}

const clearScene = scene(false);
const occupiedScene = scene(true);
const baselineImage = render(clearScene, 300, 45, 1.0);
const currentReference = currentDescriptor(baselineImage);
const proposedReference = proposedDescriptor(baselineImage);
const globalHardReference = proposedDescriptor(baselineImage, null, 0, false);
const canonicalAngle = calibratedCanonicalAngle(baselineImage);
const canonicalHardReference = proposedDescriptor(baselineImage, null, canonicalAngle, false);
const canonicalSoftReference = proposedDescriptor(baselineImage, null, canonicalAngle, true);
const rows = [];

for (const occupied of [false, true]) {
  for (const elevation of [15, 25, 35, 45, 60, 75]) {
    for (let azimuth = 0; azimuth < 360; azimuth += 30) {
      for (const strength of [0.65, 0.85, 1.0, 1.15, 1.35]) {
        const image = render(occupied ? occupiedScene : clearScene, azimuth, elevation, strength);
        const current = currentDescriptor(image, currentReference.maximum);
        const proposed = proposedDescriptor(image, proposedReference.maximum);
        const globalHard = proposedDescriptor(image, globalHardReference.maximum, 0, false);
        const canonicalHard = proposedDescriptor(image, canonicalHardReference.maximum, canonicalAngle, false);
        const canonicalSoft = proposedDescriptor(image, canonicalSoftReference.maximum, canonicalAngle, true);
        rows.push({
          occupied: occupied ? 1 : 0,
          azimuth,
          elevation,
          strength,
          current_score: currentScore(currentReference, current),
          proposed_score: proposedScore(proposedReference, proposed),
          global_hard_score: proposedScore(globalHardReference, globalHard),
          canonical_hard_score: proposedScore(canonicalHardReference, canonicalHard),
          canonical_soft_score: proposedScore(canonicalSoftReference, canonicalSoft),
          current_edges: current.edges,
          proposed_edges: proposed.edges,
          current_maximum: current.maximum,
          proposed_maximum: proposed.maximum
        });
      }
    }
  }
}

function percentile(values, fraction) {
  const sorted = [...values].sort((a, b) => a - b);
  return sorted[Math.round((sorted.length - 1) * fraction)];
}

function summaryFor(key) {
  const clear = rows.filter(row => !row.occupied).map(row => row[key]);
  const occupied = rows.filter(row => row.occupied).map(row => row[key]);
  const threshold = 0.4;
  let orderedPairs = 0;
  for (const occupiedScore of occupied) for (const clearScore of clear) {
    orderedPairs += occupiedScore > clearScore ? 1 : (occupiedScore === clearScore ? 0.5 : 0);
  }
  const candidates = [...new Set([...clear, ...occupied])].sort((a, b) => a - b);
  let best = { balanced_accuracy: -1, threshold: 0, false_positive_rate: 1, true_positive_rate: 0 };
  for (const candidate of candidates) {
    const truePositiveRate = occupied.filter(score => score >= candidate).length / occupied.length;
    const falsePositiveRate = clear.filter(score => score >= candidate).length / clear.length;
    const balancedAccuracy = (truePositiveRate + 1 - falsePositiveRate) / 2;
    if (balancedAccuracy > best.balanced_accuracy) best = {
      balanced_accuracy: balancedAccuracy,
      threshold: candidate,
      false_positive_rate: falsePositiveRate,
      true_positive_rate: truePositiveRate
    };
  }
  return {
    clear_max: Math.max(...clear),
    clear_p95: percentile(clear, 0.95),
    clear_false_triggers_at_0_4: clear.filter(score => score >= threshold).length,
    clear_observations: clear.length,
    occupied_min: Math.min(...occupied),
    occupied_p05: percentile(occupied, 0.05),
    occupied_misses_at_0_4: occupied.filter(score => score < threshold).length,
    occupied_observations: occupied.length,
    auc: orderedPairs / (clear.length * occupied.length),
    best_balanced_operating_point: best
  };
}

function sliceFor(key, predicate) {
  const selected = rows.filter(row => !row.occupied && predicate(row));
  const scores = selected.map(row => row[key]);
  return {
    maximum: Math.max(...scores),
    mean: scores.reduce((sum, value) => sum + value, 0) / scores.length,
    false_triggers_at_0_4: scores.filter(score => score >= 0.4).length,
    observations: scores.length
  };
}

const result = {
  model: {
    sensor: 'round radius 10 px',
    track_rotation_degrees: TRACK_ANGLE_DEGREES,
    baseline: { azimuth: 300, elevation: 45, strength: 1.0 },
    sweep: 'azimuth 0..330/30; elevation 15,25,35,45,60,75; strength 0.65,0.85,1,1.15,1.35',
    threshold: 0.4,
    calibrated_canonical_angle_degrees: canonicalAngle,
    observations_per_class: rows.length / 2
  },
  current: {
    ...summaryFor('current_score'),
    clear_strength_only: sliceFor('current_score', row => row.azimuth === 300 && row.elevation === 45),
    clear_elevation_only: sliceFor('current_score', row => row.azimuth === 300 && row.strength === 1),
    clear_azimuth_only: sliceFor('current_score', row => row.elevation === 45 && row.strength === 1)
  },
  proposed: {
    ...summaryFor('proposed_score'),
    clear_strength_only: sliceFor('proposed_score', row => row.azimuth === 300 && row.elevation === 45),
    clear_elevation_only: sliceFor('proposed_score', row => row.azimuth === 300 && row.strength === 1),
    clear_azimuth_only: sliceFor('proposed_score', row => row.elevation === 45 && row.strength === 1)
  },
  global_hard_15: {
    ...summaryFor('global_hard_score'),
    clear_strength_only: sliceFor('global_hard_score', row => row.azimuth === 300 && row.elevation === 45),
    clear_elevation_only: sliceFor('global_hard_score', row => row.azimuth === 300 && row.strength === 1),
    clear_azimuth_only: sliceFor('global_hard_score', row => row.elevation === 45 && row.strength === 1)
  },
  canonical_hard_15: {
    ...summaryFor('canonical_hard_score'),
    clear_strength_only: sliceFor('canonical_hard_score', row => row.azimuth === 300 && row.elevation === 45),
    clear_elevation_only: sliceFor('canonical_hard_score', row => row.azimuth === 300 && row.strength === 1),
    clear_azimuth_only: sliceFor('canonical_hard_score', row => row.elevation === 45 && row.strength === 1)
  },
  canonical_soft_15: {
    ...summaryFor('canonical_soft_score'),
    clear_strength_only: sliceFor('canonical_soft_score', row => row.azimuth === 300 && row.elevation === 45),
    clear_elevation_only: sliceFor('canonical_soft_score', row => row.azimuth === 300 && row.strength === 1),
    clear_azimuth_only: sliceFor('canonical_soft_score', row => row.elevation === 45 && row.strength === 1)
  }
};

const outputDirectory = path.join(__dirname, 'results');
fs.mkdirSync(outputDirectory, { recursive: true });
const csv = [
  Object.keys(rows[0]).join(','),
  ...rows.map(row => Object.values(row).map(value => typeof value === 'number' && !Number.isInteger(value) ? value.toFixed(6) : value).join(','))
].join('\n') + '\n';
fs.writeFileSync(path.join(outputDirectory, 'synthetic-lighting-sweep.csv'), csv);
fs.writeFileSync(path.join(outputDirectory, 'synthetic-lighting-summary.json'), JSON.stringify(result, null, 2) + '\n');
process.stdout.write(JSON.stringify(result, null, 2) + '\n');
