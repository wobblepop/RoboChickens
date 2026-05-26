import { SpatialIndex } from "../src/robot/spatial-index";
import type { BoundingBox, Vec2 } from "../src/shared/types";

describe("SpatialIndex", () => {
  const bounds: BoundingBox = {
    min: { x: 0, y: 0 },
    max: { x: 100, y: 100 },
  };

  // -----------------------------------------------------------------------
  // Constructor
  // -----------------------------------------------------------------------

  describe("constructor", () => {
    it("creates an index with valid bounds", () => {
      const index = new SpatialIndex(bounds);
      expect(index.size).toBe(0);
    });

    it("throws when bounds.min > bounds.max", () => {
      const bad: BoundingBox = { min: { x: 10, y: 10 }, max: { x: 5, y: 5 } };
      expect(() => new SpatialIndex(bad)).toThrow("bounds.min must be <= bounds.max");
    });
  });

  // -----------------------------------------------------------------------
  // insert
  // -----------------------------------------------------------------------

  describe("insert", () => {
    it("inserts a single point and retrieves it via nearest", () => {
      const index = new SpatialIndex(bounds);
      index.insert("a", { x: 10, y: 20 });
      expect(index.size).toBe(1);

      const results = index.nearest({ x: 10, y: 20 }, 1);
      expect(results.length).toBe(1);
      expect(results[0].id).toBe("a");
      expect(results[0].distance).toBeCloseTo(0, 5);
    });

    it("throws when position is outside bounds", () => {
      const index = new SpatialIndex(bounds);
      expect(() => index.insert("out", { x: 200, y: 50 })).toThrow("outside the spatial index bounds");
    });

    it("throws when id is empty", () => {
      const index = new SpatialIndex(bounds);
      expect(() => index.insert("", { x: 10, y: 10 })).toThrow("id must be a non-empty string");
    });
  });

  // -----------------------------------------------------------------------
  // nearest
  // -----------------------------------------------------------------------

  describe("nearest", () => {
    let index: SpatialIndex;

    beforeEach(() => {
      index = new SpatialIndex(bounds);
      index.insert("a", { x: 10, y: 10 });
      index.insert("b", { x: 20, y: 20 });
      index.insert("c", { x: 50, y: 50 });
      index.insert("d", { x: 80, y: 80 });
      index.insert("e", { x: 90, y: 90 });
    });

    it("k=1 returns the closest point", () => {
      const results = index.nearest({ x: 11, y: 11 }, 1);
      expect(results.length).toBe(1);
      expect(results[0].id).toBe("a");
    });

    it("k=3 returns 3 results sorted by distance", () => {
      const results = index.nearest({ x: 50, y: 50 }, 3);
      expect(results.length).toBe(3);
      expect(results[0].id).toBe("c"); // exact match
      expect(results[0].distance).toBeCloseTo(0, 5);
      // Remaining two should be sorted ascending
      expect(results[1].distance).toBeLessThanOrEqual(results[2].distance);
    });

    it("returns empty array from empty index", () => {
      const emptyIndex = new SpatialIndex(bounds);
      const results = emptyIndex.nearest({ x: 50, y: 50 }, 5);
      expect(results).toEqual([]);
    });
  });

  // -----------------------------------------------------------------------
  // withinRadius
  // -----------------------------------------------------------------------

  describe("withinRadius", () => {
    it("returns only points within the specified radius", () => {
      const index = new SpatialIndex(bounds);
      index.insert("close", { x: 50, y: 50 });
      index.insert("medium", { x: 55, y: 50 });
      index.insert("far", { x: 90, y: 90 });

      const results = index.withinRadius({ x: 50, y: 50 }, 10);
      const ids = results.map((r) => r.id);
      expect(ids).toContain("close");
      expect(ids).toContain("medium");
      expect(ids).not.toContain("far");
    });

    it("returns empty array from empty index", () => {
      const index = new SpatialIndex(bounds);
      const results = index.withinRadius({ x: 50, y: 50 }, 100);
      expect(results).toEqual([]);
    });
  });

  // -----------------------------------------------------------------------
  // withinBounds
  // -----------------------------------------------------------------------

  describe("withinBounds", () => {
    it("returns only points within the bounding box", () => {
      const index = new SpatialIndex(bounds);
      index.insert("inside", { x: 25, y: 25 });
      index.insert("also-inside", { x: 30, y: 30 });
      index.insert("outside", { x: 80, y: 80 });

      const bbox: BoundingBox = { min: { x: 20, y: 20 }, max: { x: 40, y: 40 } };
      const results = index.withinBounds(bbox);
      const ids = results.map((r) => r.id);
      expect(ids).toContain("inside");
      expect(ids).toContain("also-inside");
      expect(ids).not.toContain("outside");
    });
  });

  // -----------------------------------------------------------------------
  // remove
  // -----------------------------------------------------------------------

  describe("remove", () => {
    it("removes a point so it no longer appears in queries", () => {
      const index = new SpatialIndex(bounds);
      index.insert("a", { x: 10, y: 10 });
      index.insert("b", { x: 20, y: 20 });

      index.remove("a");
      expect(index.size).toBe(1);

      const results = index.nearest({ x: 10, y: 10 }, 5);
      const ids = results.map((r) => r.id);
      expect(ids).not.toContain("a");
      expect(ids).toContain("b");
    });

    it("does nothing when removing a non-existent id", () => {
      const index = new SpatialIndex(bounds);
      index.insert("a", { x: 10, y: 10 });
      index.remove("nonexistent");
      expect(index.size).toBe(1);
    });
  });

  // -----------------------------------------------------------------------
  // rayCast
  // -----------------------------------------------------------------------

  describe("rayCast", () => {
    it("returns points along a ray within corridor width", () => {
      const index = new SpatialIndex(bounds);
      // Points along x-axis
      index.insert("on-ray", { x: 30, y: 50 });
      index.insert("near-ray", { x: 50, y: 52 }); // within corridor of 5
      index.insert("off-ray", { x: 50, y: 70 }); // outside corridor of 5

      const direction: Vec2 = { x: 1, y: 0 }; // rightward
      const results = index.rayCast({ x: 10, y: 50 }, direction, 80, 5);
      const ids = results.map((r) => r.id);

      expect(ids).toContain("on-ray");
      expect(ids).toContain("near-ray");
      expect(ids).not.toContain("off-ray");
    });

    it("returns results sorted by distance along the ray", () => {
      const index = new SpatialIndex(bounds);
      index.insert("far", { x: 70, y: 50 });
      index.insert("close", { x: 30, y: 50 });
      index.insert("mid", { x: 50, y: 50 });

      const results = index.rayCast({ x: 10, y: 50 }, { x: 1, y: 0 }, 80, 5);
      expect(results[0].id).toBe("close");
      expect(results[1].id).toBe("mid");
      expect(results[2].id).toBe("far");
    });

    it("returns empty array from empty index", () => {
      const index = new SpatialIndex(bounds);
      const results = index.rayCast({ x: 0, y: 0 }, { x: 1, y: 0 }, 50, 5);
      expect(results).toEqual([]);
    });
  });
});
