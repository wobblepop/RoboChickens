// ===========================================================================
// Feature Detector
// Analyzes a 360° LiDAR scan to detect topological features.
// Places nodes at CORRIDOR CENTERS (navigable positions), not at wall edges.
// Works like a Hummingbird 360 sonar — one pulse maps everything in range.
// ===========================================================================

import type { Vec2, DetectedFeature, LidarScan, Pose2D } from "../../shared/types";
import { FeatureType } from "../../shared/types";
import { normalizeAngle, angleDifference, vec2Distance } from "../../shared/math";

export interface FeatureDetectorConfig {
  min_separation: number;
  range_jump_threshold: number;
}

export const DEFAULT_FEATURE_DETECTOR_CONFIG: FeatureDetectorConfig = {
  min_separation: 2.5,
  range_jump_threshold: 1.0,
};

export class FeatureDetector {
  private readonly config: FeatureDetectorConfig;
  private knownFeaturePositions: Vec2[] = [];

  constructor(config: FeatureDetectorConfig = DEFAULT_FEATURE_DETECTOR_CONFIG) {
    this.config = config;
  }

  reset(): void { this.knownFeaturePositions = []; }

  registerKnownFeature(position: Vec2): void {
    this.knownFeaturePositions.push(position);
  }

  /**
   * Full 360° scan analysis. Returns features at navigable positions.
   */
  detectFromScan(scan: LidarScan, pose: Pose2D): DetectedFeature[] {
    const features: DetectedFeature[] = [];

    // Step 1: Classify current position
    const openDirs = this.findOpenDirections(scan);
    const localFeature = this.classifyLocal(pose, openDirs);
    if (localFeature) features.push(localFeature);

    // Step 2: Place nodes along every open direction.
    for (const dir of openDirs) {
      const absAngle = pose.theta + dir.centerAngle;
      const hitsMaxRange = dir.range >= scan.max_range * 0.95;

      let nodeDist: number;
      if (hitsMaxRange) {
        nodeDist = scan.max_range * 0.75;
      } else {
        nodeDist = Math.max(1.5, dir.range - 0.5);
      }
      if (nodeDist < 1.5) continue;

      const nodePos: Vec2 = {
        x: pose.x + nodeDist * Math.cos(absAngle),
        y: pose.y + nodeDist * Math.sin(absAngle),
      };

      const centerRange = this.getRangeAtAngle(scan, dir.centerAngle);
      if (centerRange < nodeDist) continue;

      features.push({
        type: FeatureType.Corner,
        position: nodePos,
        confidence: hitsMaxRange ? 0.5 : 0.6,
        heading: absAngle,
        exit_directions: [absAngle],
        local_radius: 1.0,
      });
    }

    // Step 3: Detect side openings (range discontinuities = cross corridors)
    const sideOpenings = this.findSideOpenings(scan, pose);
    features.push(...sideOpenings);

    // Deduplicate and suppress
    return this.filterFeatures(features);
  }

  // -------------------------------------------------------------------------
  // Local position classification
  // -------------------------------------------------------------------------

  private classifyLocal(pose: Pose2D, openDirs: OpenDirection[]): DetectedFeature | null {
    const exitCount = openDirs.length;

    if (exitCount >= 3) {
      return {
        type: FeatureType.Junction,
        position: { x: pose.x, y: pose.y },
        confidence: Math.min(1.0, exitCount * 0.25),
        heading: pose.theta,
        exit_directions: openDirs.map((d) => pose.theta + d.centerAngle),
        local_radius: 1.5,
      };
    }
    if (exitCount <= 1) {
      return {
        type: FeatureType.DeadEnd,
        position: { x: pose.x, y: pose.y },
        confidence: 0.85,
        heading: pose.theta,
        exit_directions: openDirs.map((d) => pose.theta + d.centerAngle),
        local_radius: 1.0,
      };
    }
    if (exitCount === 2) {
      const between = Math.abs(angleDifference(openDirs[0].centerAngle, openDirs[1].centerAngle));
      if (between < Math.PI * 0.6) {
        return {
          type: FeatureType.Corner,
          position: { x: pose.x, y: pose.y },
          confidence: 0.8,
          heading: pose.theta,
          exit_directions: openDirs.map((d) => pose.theta + d.centerAngle),
          local_radius: 1.0,
        };
      }
    }
    return null;
  }

  // -------------------------------------------------------------------------
  // Open direction detection — groups of long-range rays
  // -------------------------------------------------------------------------

  private findOpenDirections(scan: LidarScan): OpenDirection[] {
    const threshold = 1.5;
    const groups: { angle: number; range: number }[][] = [];
    let cur: { angle: number; range: number }[] = [];

    for (let i = 0; i < scan.ranges.length; i++) {
      if (scan.ranges[i] > threshold) {
        cur.push({ angle: scan.angles[i], range: scan.ranges[i] });
      } else {
        if (cur.length >= 3) groups.push(cur);
        cur = [];
      }
    }
    if (cur.length >= 3) groups.push(cur);

    // Wrap-around merge
    if (groups.length >= 2) {
      const f = groups[0], l = groups[groups.length - 1];
      const wrap = Math.abs(normalizeAngle(f[0].angle - l[l.length - 1].angle));
      if (wrap < 0.2) { groups[0] = [...l, ...f]; groups.pop(); }
    }

    // For wide groups (>90°), find range PEAKS — each peak is a corridor
    // direction. This handles open rooms where all rays pass the threshold.
    const splitGroups: { angle: number; range: number }[][] = [];
    for (const g of groups) {
      const angularWidth = Math.abs(g[g.length - 1].angle - g[0].angle);
      if (angularWidth > Math.PI / 2 && g.length > 20) {
        const peaks = this.findRangePeaks(g);
        splitGroups.push(...peaks);
      } else {
        splitGroups.push(g);
      }
    }

    return splitGroups.map((g) => {
      const mid = Math.floor(g.length / 2);
      let maxR = 0;
      for (const r of g) if (r.range > maxR) maxR = r.range;
      const angularWidth = Math.abs(g[g.length - 1].angle - g[0].angle);
      return { centerAngle: g[mid].angle, range: maxR, angularWidth };
    });
  }

  // -------------------------------------------------------------------------
  // Side opening detection (cross corridors via range jumps)
  // -------------------------------------------------------------------------

  private findSideOpenings(scan: LidarScan, pose: Pose2D): DetectedFeature[] {
    const features: DetectedFeature[] = [];
    const jump = this.config.range_jump_threshold;

    for (let i = 3; i < scan.ranges.length - 3; i++) {
      const r = scan.ranges[i];
      const rNext = scan.ranges[i + 1];

      // Short→Long: an opening starts here
      if (rNext - r > jump && scan.ranges[i + 2] - r > jump) {
        const wallDist = r;
        if (wallDist < 1.0 || wallDist > scan.max_range * 0.9) continue;

        // Place the node along the LONG ray direction (into the opening),
        // at a distance where the ray confirms free space
        const openAngle = scan.angles[i + 1];
        const absOpenAngle = pose.theta + openAngle;
        // The node goes at the opening, at the distance where the long ray
        // shows free space (use the long ray's range, but cap it close)
        const openDist = Math.min(rNext, wallDist + 1.0);

        const nodePos: Vec2 = {
          x: pose.x + openDist * Math.cos(absOpenAngle),
          y: pose.y + openDist * Math.sin(absOpenAngle),
        };

        // Validate: the long ray actually reaches this far
        if (rNext < openDist) continue;

        features.push({
          type: FeatureType.Junction,
          position: nodePos,
          confidence: 0.6,
          heading: absOpenAngle,
          exit_directions: [absOpenAngle],
          local_radius: 1.5,
        });

        i += 5;
      }
    }

    return features;
  }

  // -------------------------------------------------------------------------
  // Split wide ray groups at local range minima
  // -------------------------------------------------------------------------

  /**
   * Find range peaks in a wide group. Each peak = a corridor direction.
   * A peak is a local maximum in the smoothed range profile.
   * Returns sub-groups centered on each peak (±15 rays around peak center).
   */
  private findRangePeaks(group: { angle: number; range: number }[]): { angle: number; range: number }[][] {
    if (group.length < 10) return [group];

    // Smooth ranges heavily to find robust peaks
    const smoothed: number[] = [];
    const w = 3;
    for (let i = 0; i < group.length; i++) {
      let sum = 0, count = 0;
      for (let j = Math.max(0, i - w); j <= Math.min(group.length - 1, i + w); j++) {
        sum += group[j].range;
        count++;
      }
      smoothed.push(sum / count);
    }

    // Find local maxima (peaks)
    const peaks: number[] = [];
    const minPeakDist = 15; // Minimum 15 rays (~15°) between peaks
    for (let i = 5; i < smoothed.length - 5; i++) {
      let isPeak = true;
      for (let j = 1; j <= 3; j++) {
        if (smoothed[i] <= smoothed[i - j] || smoothed[i] <= smoothed[i + j]) {
          isPeak = false;
          break;
        }
      }
      if (isPeak && smoothed[i] > 2.0) {
        // Check minimum distance from existing peaks
        if (peaks.length === 0 || i - peaks[peaks.length - 1] >= minPeakDist) {
          peaks.push(i);
        }
      }
    }

    if (peaks.length === 0) {
      // No peaks found — use cardinal directions as fallback
      const subGroups: { angle: number; range: number }[][] = [];
      const cardinals = [0, Math.PI / 2, -Math.PI / 2, Math.PI];
      for (const targetAngle of cardinals) {
        let bestIdx = 0;
        let bestDiff = Infinity;
        for (let i = 0; i < group.length; i++) {
          const diff = Math.abs(normalizeAngle(group[i].angle - targetAngle));
          if (diff < bestDiff) { bestDiff = diff; bestIdx = i; }
        }
        if (bestDiff < Math.PI / 4) {
          const start = Math.max(0, bestIdx - 10);
          const end = Math.min(group.length, bestIdx + 11);
          subGroups.push(group.slice(start, end));
        }
      }
      return subGroups.length > 0 ? subGroups : [group];
    }

    // Create sub-groups around each peak
    const halfWidth = 15;
    const subGroups: { angle: number; range: number }[][] = [];
    for (const peakIdx of peaks) {
      const start = Math.max(0, peakIdx - halfWidth);
      const end = Math.min(group.length, peakIdx + halfWidth + 1);
      subGroups.push(group.slice(start, end));
    }
    return subGroups;
  }

  // -------------------------------------------------------------------------
  // Filter & deduplicate
  // -------------------------------------------------------------------------

  private filterFeatures(features: DetectedFeature[]): DetectedFeature[] {
    const result: DetectedFeature[] = [];
    for (const f of features) {
      // Skip if essentially overlapping a known feature (tight threshold).
      // Direction-aware separation is handled by the PLANNING phase.
      let tooClose = false;
      for (const known of this.knownFeaturePositions) {
        if (vec2Distance(f.position, known) < 1.0) {
          tooClose = true; break;
        }
      }
      if (tooClose) continue;

      // Skip if too close to another feature in this batch (keep higher confidence)
      let dominated = false;
      for (const other of result) {
        if (vec2Distance(f.position, other.position) < 1.0) {
          if (other.confidence >= f.confidence) { dominated = true; break; }
          result.splice(result.indexOf(other), 1);
          break;
        }
      }
      if (!dominated) result.push(f);
    }
    return result;
  }

  private getRangeAtAngle(scan: LidarScan, relativeAngle: number): number {
    let bestIdx = 0;
    let bestDiff = Infinity;
    for (let i = 0; i < scan.angles.length; i++) {
      const diff = Math.abs(normalizeAngle(scan.angles[i] - relativeAngle));
      if (diff < bestDiff) {
        bestDiff = diff;
        bestIdx = i;
      }
    }
    return scan.ranges[bestIdx];
  }
}

interface OpenDirection {
  centerAngle: number;
  range: number;
  angularWidth: number;
}
