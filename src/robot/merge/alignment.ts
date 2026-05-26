// ===========================================================================
// Graph Alignment — Merge Phase 3
// Computes a 2D similarity transform (translation + rotation + uniform scale)
// from anchor-point correspondences between two independently-built graphs.
//
// Uses Umeyama's method for 3+ anchors, degrades gracefully with fewer:
//   0 anchors → error (nothing to align)
//   1 anchor  → translation only, quality "tentative"
//   2 anchors → full similarity transform, quality "low"
//   3+ anchors → full similarity transform with residual analysis
// ===========================================================================

import type { Vec2 } from "../../shared/types";
import { vec2Distance, normalizeAngle } from "../../shared/math";

// ---------------------------------------------------------------------------
// Interfaces
// ---------------------------------------------------------------------------

export interface AnchorPair {
  a_position: Vec2;
  b_position: Vec2;
  node_a_id: string;
  node_b_id: string;
}

export interface AlignmentTransform {
  translation: Vec2;
  rotation: number;
  scale: number;
  quality: "tentative" | "low" | "good" | "high";
  residual_mean: number;
  residual_max: number;
}

// ---------------------------------------------------------------------------
// Quality Thresholds (meters)
// ---------------------------------------------------------------------------

const HIGH_QUALITY_RESIDUAL_MEAN = 0.3;

// ---------------------------------------------------------------------------
// Graph Aligner
// ---------------------------------------------------------------------------

export class GraphAligner {

  // =========================================================================
  // Public API
  // =========================================================================

  /**
   * Compute a similarity transform that maps graph-B coordinates into
   * graph-A's coordinate frame, using matched anchor pairs.
   */
  align(anchors: AnchorPair[]): AlignmentTransform {
    if (!Array.isArray(anchors)) {
      throw new Error("align: anchors must be an array");
    }
    if (anchors.length === 0) {
      throw new Error("align: at least one anchor pair is required");
    }

    if (anchors.length === 1) {
      return this.alignSingleAnchor(anchors[0]);
    }

    if (anchors.length === 2) {
      const transform = this.computeSimilarityTransform(anchors);
      // Two anchors produce an exact solution — no meaningful residual
      return {
        ...transform,
        quality: "low",
        residual_mean: 0,
        residual_max: 0,
      };
    }

    // 3+ anchors: full similarity transform with residual-based quality
    const transform = this.computeSimilarityTransform(anchors);
    const residuals = this.computeResiduals(anchors, transform);

    const quality = residuals.mean <= HIGH_QUALITY_RESIDUAL_MEAN ? "high" : "good";

    return {
      ...transform,
      quality,
      residual_mean: residuals.mean,
      residual_max: residuals.max,
    };
  }

  /**
   * Apply a similarity transform to a single position:
   *   result = scale * R(rotation) * position + translation
   */
  transformPosition(position: Vec2, transform: AlignmentTransform): Vec2 {
    if (position == null) {
      throw new Error("transformPosition: position must not be null");
    }
    if (transform == null) {
      throw new Error("transformPosition: transform must not be null");
    }

    const cos = Math.cos(transform.rotation);
    const sin = Math.sin(transform.rotation);
    const s = transform.scale;

    return {
      x: s * (cos * position.x - sin * position.y) + transform.translation.x,
      y: s * (sin * position.x + cos * position.y) + transform.translation.y,
    };
  }

  /**
   * Compute how well a transform fits the anchor pairs.
   * For each pair, transforms b_position and measures distance to a_position.
   */
  computeResiduals(
    anchors: AnchorPair[],
    transform: AlignmentTransform,
  ): { mean: number; max: number } {
    if (!Array.isArray(anchors) || anchors.length === 0) {
      throw new Error("computeResiduals: anchors must be a non-empty array");
    }
    if (transform == null) {
      throw new Error("computeResiduals: transform must not be null");
    }

    let sum = 0;
    let max = 0;

    for (const anchor of anchors) {
      const transformed = this.transformPosition(anchor.b_position, transform);
      const residual = vec2Distance(transformed, anchor.a_position);
      sum += residual;
      if (residual > max) max = residual;
    }

    return {
      mean: sum / anchors.length,
      max,
    };
  }

  // =========================================================================
  // Private: Single-Anchor Alignment
  // =========================================================================

  private alignSingleAnchor(anchor: AnchorPair): AlignmentTransform {
    return {
      translation: {
        x: anchor.a_position.x - anchor.b_position.x,
        y: anchor.a_position.y - anchor.b_position.y,
      },
      rotation: 0,
      scale: 1,
      quality: "tentative",
      residual_mean: 0,
      residual_max: 0,
    };
  }

  // =========================================================================
  // Private: Umeyama Similarity Transform (2D)
  // =========================================================================

  /**
   * Compute the optimal similarity transform (rotation + uniform scale +
   * translation) that minimizes the sum of squared residuals mapping
   * b_positions → a_positions.
   *
   * Implements Umeyama's method for 2D point sets:
   *   1. Compute centroids of both point sets
   *   2. Center the points
   *   3. Compute the 2×2 cross-covariance matrix H
   *   4. Extract rotation from H (atan2 for 2D)
   *   5. Compute uniform scale from variance ratio
   *   6. Compute translation from centroids
   *
   * Reference: S. Umeyama, "Least-Squares Estimation of Transformation
   * Parameters Between Two Point Patterns," IEEE TPAMI 13(4), 1991.
   */
  private computeSimilarityTransform(anchors: AnchorPair[]): AlignmentTransform {
    const n = anchors.length;

    // --- Step 1: Compute centroids ---
    let centAx = 0, centAy = 0;
    let centBx = 0, centBy = 0;
    for (const a of anchors) {
      centAx += a.a_position.x;
      centAy += a.a_position.y;
      centBx += a.b_position.x;
      centBy += a.b_position.y;
    }
    centAx /= n;
    centAy /= n;
    centBx /= n;
    centBy /= n;

    // --- Step 2: Centered coordinates ---
    const dA: Vec2[] = [];
    const dB: Vec2[] = [];
    for (const a of anchors) {
      dA.push({ x: a.a_position.x - centAx, y: a.a_position.y - centAy });
      dB.push({ x: a.b_position.x - centBx, y: a.b_position.y - centBy });
    }

    // --- Step 3: Cross-covariance matrix H (2x2) ---
    //   H = (1/n) * sum( dA_i * dB_i^T )
    // For 2D: H = [[h00, h01], [h10, h11]]
    let h00 = 0, h01 = 0, h10 = 0, h11 = 0;
    for (let i = 0; i < n; i++) {
      h00 += dA[i].x * dB[i].x;
      h01 += dA[i].x * dB[i].y;
      h10 += dA[i].y * dB[i].x;
      h11 += dA[i].y * dB[i].y;
    }
    h00 /= n;
    h01 /= n;
    h10 /= n;
    h11 /= n;

    // --- Step 4: Rotation ---
    // For 2D, the optimal rotation angle theta satisfies:
    //   theta = atan2(h10 - h01, h00 + h11)
    // This comes from decomposing the SVD in 2D.
    const rotation = normalizeAngle(Math.atan2(h10 - h01, h00 + h11));

    // --- Step 5: Uniform scale ---
    // sigma_B^2 = (1/n) * sum(|dB_i|^2)
    // After rotation, scale = trace(D * S) / sigma_B^2
    // In 2D with proper rotation: scale = sigma_A / sigma_B
    let varA = 0, varB = 0;
    for (let i = 0; i < n; i++) {
      varA += dA[i].x * dA[i].x + dA[i].y * dA[i].y;
      varB += dB[i].x * dB[i].x + dB[i].y * dB[i].y;
    }
    varA /= n;
    varB /= n;

    // Guard against degenerate case (all B points coincident)
    const scale = varB < 1e-12 ? 1.0 : Math.sqrt(varA / varB);

    // --- Step 6: Translation ---
    // t = centA - scale * R * centB
    const cos = Math.cos(rotation);
    const sin = Math.sin(rotation);
    const translation: Vec2 = {
      x: centAx - scale * (cos * centBx - sin * centBy),
      y: centAy - scale * (sin * centBx + cos * centBy),
    };

    return {
      translation,
      rotation,
      scale,
      quality: "good",
      residual_mean: 0,
      residual_max: 0,
    };
  }
}
