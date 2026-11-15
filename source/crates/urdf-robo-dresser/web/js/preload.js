// Preload-to-cache for the dresser's 3D viewer.
//
// Two tiers, because they do different jobs:
//
//   THREE.Cache   in-memory, per page. three.js's FileLoader checks it BEFORE
//                 going to the network, so a primed entry means the loader
//                 issues no request at all — better than a warm HTTP cache,
//                 which still costs a validation round trip per file.
//   Cache Storage on disk, survives closing the panel. Rehydrates THREE.Cache
//                 on the next open from local disk instead of over the tunnel.
//
// Why this exists: a single real gripper is ~2,900 files, and the catalogue as a
// whole (robots + tools + dressed combos) is ~15,000. Over a remote Kiro session
// the webview runs on the client and the server on EC2, so each of those is a
// tunnelled round trip, capped at 6 concurrent connections by the browser. The
// server bundles one entry per request; this side unpacks it and decides what
// actually needs fetching.
//
// Scope note: every entry here is one the viewer can really load — selecting a
// robot or tool previews its SOURCE urdf, and the Dressed list loads the combo.

import * as THREE from "three";
import * as api from "./api.js";

const CACHE_NAME = "dresser-preload-v1";
/** Cache key for an index entry: unique across collections. */
const keyOf = (e) => `${e.collection}/${e.name}`;
/** Cache Storage key holding the etag map, so freshness survives a reload. */
const ETAG_KEY = "/__preload_etags__";

/** Cache Storage is unavailable in some embeddings; degrade to memory-only. */
async function openCache() {
  try {
    return typeof caches !== "undefined" ? await caches.open(CACHE_NAME) : null;
  } catch {
    return null;
  }
}

async function readEtags(cache) {
  if (!cache) return {};
  try {
    const resp = await cache.match(ETAG_KEY);
    return resp ? await resp.json() : {};
  } catch {
    return {};
  }
}

async function writeEtags(cache, etags) {
  if (!cache) return;
  try {
    await cache.put(ETAG_KEY, new Response(JSON.stringify(etags), {
      headers: { "Content-Type": "application/json" },
    }));
  } catch {
    /* quota or unsupported — memory tier still works */
  }
}

/**
 * Ask the browser to exempt this origin's cache from eviction under disk
 * pressure. Best-effort: unsupported or denied is fine, the cache just becomes
 * evictable. Called once from preloadAll().
 */
async function requestPersistence() {
  try {
    if (navigator.storage?.persist && !(await navigator.storage.persisted())) {
      return await navigator.storage.persist();
    }
    return await navigator.storage?.persisted?.();
  } catch {
    return false;
  }
}

/** Bytes currently charged to this origin, or null when unavailable. */
export async function cacheUsage() {
  try {
    const { usage } = (await navigator.storage?.estimate?.()) || {};
    return typeof usage === "number" ? usage : null;
  } catch {
    return null;
  }
}

/**
 * Compare the server's index against what is cached.
 * Returns { stale, fresh, removed, staleBytes, totalBytes, index }.
 * `removed` names entries still cached but gone from the catalogue.
 */
export async function inspect() {
  const index = await api.preloadIndex();
  const cache = await openCache();
  const etags = await readEtags(cache);

  const stale = [];
  const fresh = [];
  for (const entry of index) {
    const k = keyOf(entry);
    if (etags[k] && etags[k] === entry.etag) fresh.push(entry);
    else stale.push(entry);
  }
  const live = new Set(index.map(keyOf));
  const removed = Object.keys(etags).filter((k) => !live.has(k));

  return {
    index,
    stale,
    fresh,
    removed,
    staleBytes: stale.reduce((n, e) => n + e.bytes, 0),
    totalBytes: index.reduce((n, e) => n + e.bytes, 0),
  };
}

/** Put every member of an entry into both cache tiers. */
async function store(cache, bundle) {
  for (const m of bundle.members) {
    const url = api.assetUrl(m.rel);
    // Keyed by the exact URL the loaders will request.
    THREE.Cache.add(url, m.bytes);
    if (cache) {
      try {
        await cache.put(url, new Response(m.bytes, {
          headers: { "Content-Type": "application/octet-stream" },
        }));
      } catch {
        // Out of quota: the in-memory tier still serves this session.
      }
    }
  }
}

/** Drop an entry's Cache Storage entries (used for entries deleted upstream). */
async function evict(cache, etags, key) {
  if (cache) {
    const prefix = api.assetUrl(`${key}/`);
    for (const req of await cache.keys()) {
      if (req.url.includes(prefix)) await cache.delete(req);
    }
  }
  delete etags[key];
}

/**
 * Warm the cache for every catalogue entry that needs it.
 *
 * `onProgress({phase, label, done, total, bytes})` drives the UI. Returns a
 * summary: { upToDate, updated, skipped, pruned, bytes }. `upToDate` is true
 * when the freshness check found nothing to do — the caller should say so rather
 * than implying work happened.
 */
export async function preloadAll({ onProgress = () => {}, signal } = {}) {
  onProgress({ phase: "checking" });
  const { stale, fresh, removed, staleBytes } = await inspect();
  const cache = await openCache();
  const etags = await readEtags(cache);

  // Prune entries that no longer exist so the reported size stays honest.
  for (const key of removed) await evict(cache, etags, key);
  if (removed.length) await writeEtags(cache, etags);

  if (stale.length === 0) {
    return {
      upToDate: true,
      updated: 0,
      skipped: fresh.length,
      pruned: removed.length,
      bytes: 0,
    };
  }

  await requestPersistence();

  let done = 0;
  let bytes = 0;
  for (const entry of stale) {
    if (signal?.aborted) throw new DOMException("aborted", "AbortError");
    const key = keyOf(entry);
    onProgress({
      phase: "fetching",
      label: entry.name,
      done,
      total: stale.length,
      bytes,
      totalBytes: staleBytes,
    });

    const bundle = await api.preloadBundle(
      entry.collection, entry.name, etags[key] || null, signal);
    if (bundle) {
      await store(cache, bundle);
      // Record the etag only AFTER every member is stored, so an interrupted
      // entry is reported stale next time instead of falsely complete.
      etags[key] = bundle.etag || entry.etag;
      await writeEtags(cache, etags);
      bytes += entry.bytes;
    } else {
      // 304: the server says our copy is current after all.
      etags[key] = entry.etag;
      await writeEtags(cache, etags);
    }
    done += 1;
  }

  onProgress({ phase: "done", done, total: stale.length, bytes, totalBytes: staleBytes });
  return {
    upToDate: false,
    updated: done,
    skipped: fresh.length,
    pruned: removed.length,
    bytes,
  };
}

/**
 * Warm one entry, regardless of whether the index thinks it is fresh. Backs the
 * per-row cache button, which doubles as a retry: a 304 means the copy already
 * held is current, so `updated` comes back false and nothing was transferred.
 */
export async function preloadOne(collection, name, { signal } = {}) {
  const cache = await openCache();
  const etags = await readEtags(cache);
  const key = `${collection}/${name}`;

  const bundle = await api.preloadBundle(collection, name, etags[key] || null, signal);
  if (!bundle) {
    // Still record it: a 304 proves the cached copy matches the server.
    return { updated: false, bytes: 0 };
  }
  await store(cache, bundle);
  etags[key] = bundle.etag || "";
  await writeEtags(cache, etags);
  return {
    updated: true,
    bytes: bundle.members.reduce((n, m) => n + m.bytes.byteLength, 0),
  };
}

/**
 * Refill THREE.Cache from Cache Storage. Call once at startup: without it a
 * reopened panel has a populated disk cache but an empty memory cache, and the
 * loaders would go back to the network.
 */
export async function rehydrate() {
  const cache = await openCache();
  if (!cache) return 0;
  let n = 0;
  try {
    for (const req of await cache.keys()) {
      const url = new URL(req.url).pathname;
      if (url === ETAG_KEY || THREE.Cache.get(url)) continue;
      const resp = await cache.match(req);
      if (!resp) continue;
      THREE.Cache.add(url, await resp.arrayBuffer());
      n += 1;
    }
  } catch {
    /* partial rehydrate is fine — misses just fall through to the network */
  }
  return n;
}
