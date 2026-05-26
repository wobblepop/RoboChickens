import { ObservationLogManager } from "../src/robot/observation-log";
import type { ObservationRecord } from "../src/shared/types";
import { SourceType } from "../src/shared/types";

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

describe("ObservationLogManager", () => {
  // -----------------------------------------------------------------------
  // addObservation
  // -----------------------------------------------------------------------

  describe("addObservation", () => {
    it("increases active count", () => {
      const log = new ObservationLogManager("entity-1");
      expect(log.getActiveCount()).toBe(0);

      log.addObservation(makeObservation({ id: "obs-1" }));
      expect(log.getActiveCount()).toBe(1);

      log.addObservation(makeObservation({ id: "obs-2" }));
      expect(log.getActiveCount()).toBe(2);
    });

    it("is idempotent — same id does not duplicate", () => {
      const log = new ObservationLogManager("entity-1");
      const obs = makeObservation({ id: "obs-dup" });

      log.addObservation(obs);
      log.addObservation(obs);
      expect(log.getActiveCount()).toBe(1);
    });
  });

  // -----------------------------------------------------------------------
  // promoteTiers
  // -----------------------------------------------------------------------

  describe("promoteTiers", () => {
    it("moves old observations from hot to warm", () => {
      const log = new ObservationLogManager("entity-1");
      const oldTimestamp = 1000;
      log.addObservation(makeObservation({ id: "obs-old", timestamp: oldTimestamp }));

      // now is well past the hot threshold (1 hour = 3_600_000 ms)
      const now = oldTimestamp + 3_600_000 + 1;
      const moved = log.promoteTiers(now);

      expect(moved).toBeGreaterThanOrEqual(1);
      expect(log.getHotObservations().length).toBe(0);
      expect(log.getWarmObservations().length).toBe(1);
    });

    it("moves warm observations to cold after warm threshold", () => {
      const log = new ObservationLogManager("entity-1");
      const oldTimestamp = 1000;
      log.addObservation(makeObservation({ id: "obs-warm", timestamp: oldTimestamp }));

      // First promote to warm
      log.promoteTiers(oldTimestamp + 3_600_001);
      expect(log.getWarmObservations().length).toBe(1);

      // Then promote warm to cold (24 hours = 86_400_000 ms)
      log.promoteTiers(oldTimestamp + 86_400_001);
      expect(log.getWarmObservations().length).toBe(0);
      expect(log.getColdObservations().length).toBe(1);
    });
  });

  // -----------------------------------------------------------------------
  // compact
  // -----------------------------------------------------------------------

  describe("compact", () => {
    it("compacts 6+ agreeing cold observations to fewer", () => {
      const log = new ObservationLogManager("entity-1");
      const baseTimestamp = 1000;

      // Add 6 observations with similar geometry (all area=10), all at same timestamp
      for (let i = 0; i < 6; i++) {
        log.addObservation(
          makeObservation({
            id: `obs-${i}`,
            timestamp: baseTimestamp,
            payload: { geometry: { area: 10, perimeter: 12 } },
          }),
        );
      }

      // Promote hot -> warm (past hot threshold of 1 hour)
      const afterHot = baseTimestamp + 3_600_000 + 100;
      log.promoteTiers(afterHot);
      expect(log.getWarmObservations().length).toBe(6);

      // Promote warm -> cold (past warm threshold of 24 hours)
      const afterWarm = baseTimestamp + 86_400_000 + 100;
      log.promoteTiers(afterWarm);
      expect(log.getColdObservations().length).toBe(6);

      // Compact with default minAgreementCount=5
      const compacted = log.compact();
      expect(compacted).toBeGreaterThan(0);
      // 6 observations compacted to 1 consolidated => 5 removed
      expect(log.getColdObservations().length).toBeLessThan(6);
    });

    it("does not compact fewer than minAgreementCount observations", () => {
      const log = new ObservationLogManager("entity-1");
      const baseTimestamp = 1000;

      // Only 3 observations (below the default 5 threshold), all at same timestamp
      for (let i = 0; i < 3; i++) {
        log.addObservation(
          makeObservation({
            id: `obs-${i}`,
            timestamp: baseTimestamp,
            payload: { geometry: { area: 10, perimeter: 12 } },
          }),
        );
      }

      // Promote hot -> warm, then warm -> cold
      const afterHot = baseTimestamp + 3_600_000 + 100;
      const afterWarm = baseTimestamp + 86_400_000 + 100;
      log.promoteTiers(afterHot);
      log.promoteTiers(afterWarm);
      const compacted = log.compact();
      expect(compacted).toBe(0);
      expect(log.getColdObservations().length).toBe(3);
    });
  });

  // -----------------------------------------------------------------------
  // anchoredCompaction
  // -----------------------------------------------------------------------

  describe("anchoredCompaction", () => {
    it("consolidates all observations into 1", () => {
      const log = new ObservationLogManager("entity-1");

      for (let i = 0; i < 5; i++) {
        log.addObservation(
          makeObservation({
            id: `obs-${i}`,
            timestamp: 1000 + i,
            payload: { geometry: { area: 10, perimeter: 12 } },
          }),
        );
      }

      expect(log.getActiveCount()).toBe(5);
      const result = log.anchoredCompaction();
      expect(log.getActiveCount()).toBe(1);
      expect(result.supersedes.length).toBe(5);
    });

    it("throws when no observations exist", () => {
      const log = new ObservationLogManager("entity-1");
      expect(() => log.anchoredCompaction()).toThrow("no active observations to compact");
    });
  });

  // -----------------------------------------------------------------------
  // archive
  // -----------------------------------------------------------------------

  describe("archive", () => {
    it("moves old cold observations to archived", () => {
      const log = new ObservationLogManager("entity-1");
      const oldTimestamp = 1000;
      log.addObservation(makeObservation({ id: "obs-archive", timestamp: oldTimestamp }));

      // Promote hot -> warm, then warm -> cold
      const afterHot = oldTimestamp + 3_600_000 + 100;
      const afterWarm = oldTimestamp + 86_400_000 + 100;
      log.promoteTiers(afterHot);
      log.promoteTiers(afterWarm);
      expect(log.getColdObservations().length).toBe(1);

      // Archive (7 days = 604_800_000 ms)
      const archived = log.archive(oldTimestamp + 604_800_001);
      expect(archived).toBe(1);
      expect(log.getColdObservations().length).toBe(0);
      expect(log.getArchivedIds()).toContain("obs-archive");
    });

    it("does not archive recent cold observations", () => {
      const log = new ObservationLogManager("entity-1");
      const recentTimestamp = 1_000_000;
      log.addObservation(makeObservation({ id: "obs-recent", timestamp: recentTimestamp }));

      // Promote to cold
      log.promoteTiers(recentTimestamp + 86_400_001);

      // Try to archive but timestamp is not old enough
      const archived = log.archive(recentTimestamp + 100_000);
      expect(archived).toBe(0);
    });
  });

  // -----------------------------------------------------------------------
  // runLifecycle
  // -----------------------------------------------------------------------

  describe("runLifecycle", () => {
    it("chains promote, compact, and archive", () => {
      const log = new ObservationLogManager("entity-1");
      const baseTimestamp = 1000;

      for (let i = 0; i < 6; i++) {
        log.addObservation(
          makeObservation({
            id: `obs-${i}`,
            timestamp: baseTimestamp + i,
            payload: { geometry: { area: 10, perimeter: 12 } },
          }),
        );
      }

      // Run lifecycle at a point far in the future (past archive threshold)
      const now = baseTimestamp + 604_800_001;
      const result = log.runLifecycle(now);

      expect(result.promoted).toBeGreaterThanOrEqual(0);
      expect(result.compacted).toBeGreaterThanOrEqual(0);
      expect(result.archived).toBeGreaterThanOrEqual(0);
    });
  });
});
