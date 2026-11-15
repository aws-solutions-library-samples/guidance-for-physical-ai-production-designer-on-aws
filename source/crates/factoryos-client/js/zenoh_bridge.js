// Thin wrapper around zenoh-ts for `bus_web.rs`. The page maps the bare specifier
// "@eclipse-zenoh/zenoh-ts" to a bundled copy with an import map (see web/index.html).
//
// A zenoh-ts session does not survive its WebSocket closing (the backend restarted, the network
// dropped): `zenoh_open` returns a bus that opens a new session then, declares its subscribers
// there again and calls `onLost` meanwhile. Queries and puts while it is away fail.
//
// Nothing waits forever: a query ends after QUERY_TIMEOUT_MS without a reply (an error), and
// reopening only hands zenoh-ts a backend whose health check answers. zenoh-ts' own connection
// retries back off to minutes (2 s, 4 s, … 1024 s between attempts): a restart of a minute or
// two would otherwise keep the page away for several more.
import { CancellationToken, Config, ReplyError, SampleKind, Session } from "@eclipse-zenoh/zenoh-ts";

// A query with no reply by then fails (the backend's own query timeout is 10 s).
const QUERY_TIMEOUT_MS = 15000;
// Opening a session to a backend that answers its health check, at most.
const OPEN_TIMEOUT_MS = 15000;
const HEALTH_TIMEOUT_MS = 5000;
// After this many health checks in a row without an answer, try the session anyway.
const OPEN_WITHOUT_HEALTH_AFTER = 6;

export async function zenoh_open(url, onLost) {
  const bus = { url, onLost, session: await Session.open(new Config(url)), declares: [], reopening: false };
  watch(bus);
  return bus;
}

// Drop the session and open a new one (the page saw the backend restart, and this session did
// not notice: a half-open WebSocket). No-op while reopening already.
export function zenoh_reconnect(bus) {
  const ws = bus.session?.inner?.link?.ws;
  if (bus.reopening || !ws) return;
  console.warn(`zenoh: reconnecting to ${bus.url}`);
  ws.close();
}

function watch(bus) {
  // zenoh-ts keeps its WebSocket to itself: without it (another version), no reopening.
  const ws = bus.session?.inner?.link?.ws;
  if (!ws || typeof ws.addEventListener !== "function") {
    console.warn("zenoh: cannot watch the session's WebSocket; it will not reopen");
    return;
  }
  ws.addEventListener("close", () => reopen(bus), { once: true });
}

async function reopen(bus) {
  if (bus.reopening) return;
  bus.reopening = true;
  const lost = bus.session;
  bus.session = null;
  // Queries in flight would wait for replies forever: they end with none.
  for (const [, get] of lost?.inner?.gets ?? []) {
    try { get.drop(); } catch (_) { /* gone already */ }
  }
  lost?.inner?.gets?.clear?.();
  bus.onLost();
  let unhealthy = 0;
  for (let delay = 500; ; delay = Math.min(delay * 2, 5000)) {
    await sleep(delay);
    if (!(await healthy(bus.url)) && ++unhealthy < OPEN_WITHOUT_HEALTH_AFTER) continue;
    unhealthy = 0;
    try {
      bus.session = await withTimeout(Session.open(new Config(bus.url)), OPEN_TIMEOUT_MS, `opening ${bus.url}`, (late) =>
        late.close(),
      );
      break;
    } catch (err) {
      console.warn(`zenoh: reopening ${bus.url}: ${err}`);
    }
  }
  console.warn(`zenoh: session to ${bus.url} reopened`);
  bus.reopening = false;
  watch(bus);
  for (const declare of bus.declares) {
    await declare(bus.session).catch((err) => console.error(`zenoh: declaring again: ${err}`));
  }
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

// `promise`, or an error once `ms` have passed; what it resolves to later goes to `onLate`.
function withTimeout(promise, ms, what, onLate) {
  let timer;
  let timedOut = false;
  const timeout = new Promise((_, reject) => {
    timer = setTimeout(() => {
      timedOut = true;
      reject(new Error(`timed out: ${what}: no answer within ${ms / 1000} s`));
    }, ms);
  });
  promise.then(
    (value) => {
      if (timedOut) onLate?.(value);
    },
    () => {},
  );
  return Promise.race([promise, timeout]).finally(() => clearTimeout(timer));
}

// Whether the backend behind `wsUrl` (`ws(s)://…/zenoh`) answers its health check.
async function healthy(wsUrl) {
  const url = wsUrl.replace(/^ws/, "http").replace(/\/zenoh\/?$/, "") + "/api/v1/health";
  const abort = new AbortController();
  const timer = setTimeout(() => abort.abort(), HEALTH_TIMEOUT_MS);
  try {
    const response = await fetch(url, { credentials: "same-origin", cache: "no-store", signal: abort.signal });
    return response.ok;
  } catch (_) {
    return false;
  } finally {
    clearTimeout(timer);
  }
}

function session(bus) {
  if (!bus.session) throw new Error("zenoh: not connected (reopening the session)");
  return bus.session;
}

// Declared on this session and every one reopened after it.
async function declareKept(bus, declare) {
  bus.declares.push(declare);
  if (bus.session) await declare(bus.session);
}

export async function zenoh_subscribe(bus, key, onSample) {
  await declareKept(bus, (s) => s.declareSubscriber(key, { handler: (sample) => onSample(sample.payload().toBytes()) }));
}

// `onToken(key, alive)` for liveliness tokens on `key`, the ones alive already first.
export async function zenoh_liveliness_subscribe(bus, key, onToken) {
  await declareKept(bus, (s) =>
    s.liveliness().declareSubscriber(key, {
      history: true,
      handler: (sample) => onToken(sample.keyexpr().toString(), sample.kind() === SampleKind.PUT),
    }),
  );
}

// Resolves with the first reply's payload, or `null` if nobody replied; fails without a reply
// (nor the end of replies) within QUERY_TIMEOUT_MS.
export async function zenoh_get(bus, key, payload) {
  const cancellationToken = new CancellationToken();
  try {
    return await withTimeout(firstReply(session(bus), key, payload, cancellationToken), QUERY_TIMEOUT_MS, key);
  } catch (err) {
    cancellationToken.cancel();
    throw err;
  }
}

async function firstReply(session, key, payload, cancellationToken) {
  const replies = await session.get(key, payload ? { payload, cancellationToken } : { cancellationToken });
  for await (const reply of replies) {
    const result = reply.result();
    if (result instanceof ReplyError) {
      throw new Error(`${key}: ${result.payload().toString()}`);
    }
    return result.payload().toBytes();
  }
  return null;
}

export async function zenoh_put(bus, key, payload) {
  await session(bus).put(key, payload);
}
