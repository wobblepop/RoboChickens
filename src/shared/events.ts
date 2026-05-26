// ===========================================================================
// Typed Event Emitter
// Type-safe event system for inter-module communication.
// ===========================================================================

type EventHandler<T> = (data: T) => void;

/**
 * A strongly-typed event emitter.
 * Usage:
 *   interface MyEvents { "tick": { count: number }; "done": void; }
 *   const emitter = new TypedEventEmitter<MyEvents>();
 *   emitter.on("tick", (data) => console.log(data.count));
 *   emitter.emit("tick", { count: 1 });
 */
export class TypedEventEmitter<TEvents> {
  private handlers: Map<string, Set<EventHandler<unknown>>> = new Map();

  on<K extends keyof TEvents & string>(
    event: K,
    handler: EventHandler<TEvents[K]>,
  ): () => void {
    let set = this.handlers.get(event);
    if (!set) {
      set = new Set();
      this.handlers.set(event, set);
    }
    set.add(handler as EventHandler<unknown>);

    // Return unsubscribe function
    return () => {
      set!.delete(handler as EventHandler<unknown>);
      if (set!.size === 0) {
        this.handlers.delete(event);
      }
    };
  }

  once<K extends keyof TEvents & string>(
    event: K,
    handler: EventHandler<TEvents[K]>,
  ): () => void {
    const unsub = this.on(event, (data) => {
      unsub();
      handler(data);
    });
    return unsub;
  }

  emit<K extends keyof TEvents & string>(event: K, data: TEvents[K]): void {
    const set = this.handlers.get(event);
    if (!set) return;
    for (const handler of set) {
      handler(data);
    }
  }

  removeAllListeners(event?: string): void {
    if (event) {
      this.handlers.delete(event);
    } else {
      this.handlers.clear();
    }
  }

  listenerCount(event: string): number {
    const set = this.handlers.get(event);
    return set ? set.size : 0;
  }
}
