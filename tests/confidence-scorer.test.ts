import { ConfidenceScorer } from "../src/robot/confidence-scorer";
import type { ObservationRecord } from "../src/shared/types";
import {
  SourceType,
  StabilityClass,
  ConfidenceTier,
  confidenceTierFromScore,
} from "../src/shared/types";

function makeObservation(overrides: Partial<ObservationRecord> = {}): ObservationRecord {
  return {
    id: overrides.id ?? `obs-${Math.random().toString(36).slice(2, 8)}`,
    source_id: overrides.source_id ?? "robot-1",
    source_type: overrides.source_type ?? SourceType.Sensor,
    timestamp: overrides.timestamp ?? Date.now(),
    confidence: overrides.confidence ?? 0.8,
    confidence_fields: overrides.confidence_fields ?? {},
    target_type: overrides.target_type ?? "node",
    target_id: overrides.target_id ?? "node-1",
    payload: overrides.payload ?? {
      geometry: { area: 10, perimeter: 12 },
    },
    sensor_metadata: overrides.sensor_metadata ?? {
      sensors_used: ["lidar"],
      fusion_confidence: 0.8,
    },
    supersedes: overrides.supersedes ?? [],
  };
}

describe("ConfidenceScorer", () => {
  let scorer: ConfidenceScorer;

  beforeEach(() => {
    scorer = new ConfidenceScorer();
  });

  // -----------------------------------------------------------------------
  // computeObservationFactor
  // -----------------------------------------------------------------------

  describe("computeObservationFactor", () => {
    it("returns ~0.22 for count=1 (1 - e^(-0.25))", () => {
      const expected = 1 - Math.exp(-0.25);
      const result = scorer.computeObservationFactor(1);
      expect(result).toBeCloseTo(expected, 2);
      expect(result).toBeCloseTo(0.22, 1);
    });

    it("returns ~0.92 for count=10 (1 - e^(-2.5))", () => {
      const expected = 1 - Math.exp(-0.25 * 10);
      const result = scorer.computeObservationFactor(10);
      expect(result).toBeCloseTo(expected, 2);
      expect(result).toBeCloseTo(0.92, 1);
    });

    it("returns 0 for count=0", () => {
      expect(scorer.computeObservationFactor(0)).toBe(0);
    });
  });

  // -----------------------------------------------------------------------
  // computeConsistencyFactor
  // -----------------------------------------------------------------------

  describe("computeConsistencyFactor", () => {
    it("returns 1.0 when consistencyScore is 1.0", () => {
      expect(scorer.computeConsistencyFactor(1.0)).toBe(1.0);
    });

    it("returns 0.1 (floor) when consistencyScore is 0.05", () => {
      expect(scorer.computeConsistencyFactor(0.05)).toBe(0.1);
    });

    it("clamps negative scores to 0.1", () => {
      expect(scorer.computeConsistencyFactor(-0.5)).toBe(0.1);
    });

    it("clamps scores above 1.0 to 1.0", () => {
      expect(scorer.computeConsistencyFactor(1.5)).toBe(1.0);
    });
  });

  // -----------------------------------------------------------------------
  // computeCorroborationFactor
  // -----------------------------------------------------------------------

  describe("computeCorroborationFactor", () => {
    it("returns 0.4 with 1 source", () => {
      const obs = [makeObservation({ source_id: "robot-1" })];
      expect(scorer.computeCorroborationFactor(obs)).toBe(0.4);
    });

    it("returns 0.65 with 2 agreeing sources", () => {
      const obs = [
        makeObservation({
          source_id: "robot-1",
          payload: { geometry: { area: 10, perimeter: 12 } },
        }),
        makeObservation({
          source_id: "robot-2",
          payload: { geometry: { area: 10, perimeter: 12 } },
        }),
      ];
      expect(scorer.computeCorroborationFactor(obs)).toBe(0.65);
    });

    it("returns 0.85 with 3+ agreeing sources", () => {
      const obs = [
        makeObservation({ source_id: "robot-1", payload: { geometry: { area: 10, perimeter: 12 } } }),
        makeObservation({ source_id: "robot-2", payload: { geometry: { area: 10, perimeter: 12 } } }),
        makeObservation({ source_id: "robot-3", payload: { geometry: { area: 10, perimeter: 12 } } }),
      ];
      expect(scorer.computeCorroborationFactor(obs)).toBe(0.85);
    });

    it("returns 0 with empty observations", () => {
      expect(scorer.computeCorroborationFactor([])).toBe(0);
    });
  });

  // -----------------------------------------------------------------------
  // computeFreshnessFactor
  // -----------------------------------------------------------------------

  describe("computeFreshnessFactor", () => {
    it("returns 1.0 for Static stability regardless of age", () => {
      const oldObs = [makeObservation({ timestamp: 0 })];
      // now = a very large number (many hours later)
      const result = scorer.computeFreshnessFactor(oldObs, StabilityClass.Static, 1_000_000_000);
      expect(result).toBe(1.0);
    });

    it("decays for Dynamic stability", () => {
      const timestamp = 1000;
      const obs = [makeObservation({ timestamp })];
      // 10 hours later
      const now = timestamp + 10 * 3_600_000;
      const result = scorer.computeFreshnessFactor(obs, StabilityClass.Dynamic, now);
      expect(result).toBeLessThan(1.0);
      expect(result).toBeGreaterThan(0);
    });

    it("returns 0 for empty observations", () => {
      expect(scorer.computeFreshnessFactor([], StabilityClass.Static, Date.now())).toBe(0);
    });
  });

  // -----------------------------------------------------------------------
  // score (composite)
  // -----------------------------------------------------------------------

  describe("score", () => {
    it("returns a composite between 0 and 1 with a correct tier", () => {
      const obs = [
        makeObservation({ source_id: "robot-1", confidence: 0.9 }),
        makeObservation({ source_id: "robot-2", confidence: 0.85 }),
      ];
      const result = scorer.score(obs, StabilityClass.Static, 1.0);

      expect(result.composite).toBeGreaterThan(0);
      expect(result.composite).toBeLessThanOrEqual(1);
      expect(Object.values(ConfidenceTier)).toContain(result.tier);
      // Tier should match what confidenceTierFromScore returns
      expect(result.tier).toBe(confidenceTierFromScore(result.composite));
    });

    it("throws when observations array is empty", () => {
      expect(() => scorer.score([], StabilityClass.Static, 1.0)).toThrow(
        "observations array must not be empty",
      );
    });
  });

  // -----------------------------------------------------------------------
  // confidenceTierFromScore (from types)
  // -----------------------------------------------------------------------

  describe("confidenceTierFromScore", () => {
    it("maps 0.05 to Dead (< 0.10)", () => {
      expect(confidenceTierFromScore(0.05)).toBe(ConfidenceTier.Dead);
    });

    it("maps 0.2 to Candidate (0.10 <= score < 0.35)", () => {
      expect(confidenceTierFromScore(0.2)).toBe(ConfidenceTier.Candidate);
    });

    it("maps 0.5 to Provisional (0.35 <= score < 0.65)", () => {
      expect(confidenceTierFromScore(0.5)).toBe(ConfidenceTier.Provisional);
    });

    it("maps 0.75 to Committed (0.65 <= score < 0.85)", () => {
      expect(confidenceTierFromScore(0.75)).toBe(ConfidenceTier.Committed);
    });

    it("maps 0.9 to Anchored (score >= 0.85)", () => {
      expect(confidenceTierFromScore(0.9)).toBe(ConfidenceTier.Anchored);
    });

    it("maps 0.0 to Dead", () => {
      expect(confidenceTierFromScore(0.0)).toBe(ConfidenceTier.Dead);
    });

    it("maps 1.0 to Anchored", () => {
      expect(confidenceTierFromScore(1.0)).toBe(ConfidenceTier.Anchored);
    });

    it("maps boundary 0.10 to Candidate", () => {
      expect(confidenceTierFromScore(0.10)).toBe(ConfidenceTier.Candidate);
    });

    it("maps boundary 0.85 to Anchored", () => {
      expect(confidenceTierFromScore(0.85)).toBe(ConfidenceTier.Anchored);
    });
  });
});
