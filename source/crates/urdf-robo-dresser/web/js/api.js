// Thin fetch wrappers around the /api/v1/* endpoints plus the /assets byte
// route. Every method returns parsed JSON or throws with the server's message.

async function ok(resp) {
  if (!resp.ok) {
    const txt = await resp.text();
    throw new Error(`${resp.status} ${resp.statusText}: ${txt}`);
  }
  return resp;
}

/** GET /api/v1/catalogue -> { robots: [...], tools: [...] }. */
export async function listCatalogue() {
  return (await ok(await fetch("/api/v1/catalogue"))).json();
}

/** GET /api/v1/catalogue/dressed -> [{ name, urdf_rel }]. Already-dressed robots. */
export async function listDressed() {
  return (await ok(await fetch("/api/v1/catalogue/dressed"))).json();
}

/**
 * POST /api/v1/dress. Returns { ok, dressed_urdf_rel, diagnostics }.
 * `dressed_urdf_rel` is catalogue-relative and feeds assetUrl() directly.
 */
export async function dress({ robotId, toolId, attachTo = null, orient = [0, 0, 0], hull = "medium" }) {
  return (await ok(await fetch("/api/v1/dress", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ robot_id: robotId, tool_id: toolId, attach_to: attachTo, orient, hull }),
  }))).json();
}

/** POST /api/v1/validate -> { ok, diagnostics }. */
export async function validate({ path, tool = false }) {
  return (await ok(await fetch("/api/v1/validate", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ path, tool }),
  }))).json();
}

/**
 * Map a catalogue-relative path to the /assets byte route. urdf-loader
 * fetches THIS url and resolves the URDF's own ./meshes/*.obj references
 * against it — no per-mesh rewriting needed.
 */
export function assetUrl(rel) {
  return "/assets/" + rel.split("/").map(encodeURIComponent).join("/");
}

/**
 * GET /api/v1/preload -> [{ name, etag, files, bytes }].
 * One request describing every loadable catalogue entry — robots, tools and
 * dressed combos — so the caller can decide what (if anything) is stale without
 * downloading a byte of geometry.
 */
export async function preloadIndex() {
  return (await ok(await fetch("/api/v1/preload"))).json();
}

/**
 * GET /api/v1/preload/<collection>/<name> -> { etag, members: [{ rel, bytes }] }.
 *
 * One request carrying every file that entry needs, so warming the cache costs a
 * single round trip instead of the ~2,900 a real gripper's mesh set would need
 * (a browser caps HTTP/1.1 at 6 connections per origin, so client concurrency
 * cannot substitute for this).
 *
 * Pass `knownEtag` to revalidate: a 304 resolves to `null`, meaning "what you
 * have is current". Wire format is
 * `[u32 LE manifest_len][manifest JSON][file bytes, concatenated]`, decoded here
 * so no unzip library is needed.
 */
export async function preloadBundle(collection, name, knownEtag = null, signal = undefined) {
  const headers = knownEtag ? { "If-None-Match": knownEtag } : {};
  const url = `/api/v1/preload/${encodeURIComponent(collection)}/${encodeURIComponent(name)}`;
  const resp = await fetch(url, { headers, signal });
  if (resp.status === 304) return null;
  await ok(resp);

  const buf = await resp.arrayBuffer();
  const view = new DataView(buf);
  const manifestLen = view.getUint32(0, true);
  const manifest = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 4, manifestLen)));
  const base = 4 + manifestLen;

  return {
    etag: resp.headers.get("ETag"),
    members: manifest.map((m) => ({
      rel: m.rel,
      // Slice, not subarray: each member becomes its own buffer so the 90 MB
      // bundle can be released once unpacked.
      bytes: buf.slice(base + m.off, base + m.off + m.len),
    })),
  };
}
