// ===========================================================================
// UUID Generation
// Simple v4-style random UUID without external dependencies.
// ===========================================================================

const HEX = "0123456789abcdef";

/**
 * Generate a random UUID v4.
 * Format: xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx
 * where y is one of [8, 9, a, b].
 */
export function generateId(): string {
  const bytes = new Uint8Array(16);
  for (let i = 0; i < 16; i++) {
    bytes[i] = Math.floor(Math.random() * 256);
  }
  // Set version (4) in byte 6
  bytes[6] = (bytes[6] & 0x0f) | 0x40;
  // Set variant (10xx) in byte 8
  bytes[8] = (bytes[8] & 0x3f) | 0x80;

  let uuid = "";
  for (let i = 0; i < 16; i++) {
    uuid += HEX[bytes[i] >> 4];
    uuid += HEX[bytes[i] & 0x0f];
    if (i === 3 || i === 5 || i === 7 || i === 9) {
      uuid += "-";
    }
  }
  return uuid;
}

/**
 * Generate a short ID (8 hex chars). For cases where full UUID is overkill
 * but uniqueness within a session is needed.
 */
export function generateShortId(): string {
  let id = "";
  for (let i = 0; i < 8; i++) {
    id += HEX[Math.floor(Math.random() * 16)];
  }
  return id;
}
