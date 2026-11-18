// ajuk armor controller v5 — Season 2 "Drafted" (rules s2.0.0)
// v5 = v2 + ONE surgical addition: a stall-TRIGGERED escape (NOTES-v5-build.md).
// The v4 dissection proved the SPIN STALL: routeWaypoint's adjacent-tile
// waypoint flip-flops with the far goal, steer pins at -1.00, throttle 0.15,
// heading spins monotonically (~34 units/tick) with ~0 displacement for
// 1000-3600+ ticks, while the stall watchdog's forceField cannot break it.
// v4's always-on lookahead fix LOST everywhere (H2H 51.3%, deathball 47.5%
// vs v2's 65%) because the slow approach is load-bearing for combat timing.
// v5 leaves ALL normal movement byte-identical to v2. Only when a ground
// unit is PROVABLY spinning with no displacement (trailing 120-tick
// window: >=315 deg of same-direction rotation, <3 tiles moved) does the
// escape engage:
// v4-style gradient lookahead routing, until the direct line to the goal
// stays open 30 consecutive ticks (a single open tick is the flicker that
// IS the stall), the goal is reached, 900 ticks pass, or the escape itself
// makes no progress for 120 ticks. No decision, fire-gate, post, or draft change.
// Draft UNCHANGED. Everything else is v2, byte for byte.
//
// Ideas: precomputed BFS flow fields in hello() (cheap ticks), squad-wide
// contact memory, one shared focus target, lead + shell-line-of-sight gating
// before every direct shot, range-band kiting with front armour facing the
// threat, role-based objective play (drone overwatch + early cap, scout cap /
// deny, MBTs hold the contested point), artillery-shell dodging, and
// low-HP retreats. Handlers exist for every kind so the draft can adapt.

const FP = 65536;
const ANG = 4096;
const HALF = FP / 2;

let R = null;        // rules
let MAP = null;      // map
let SIDE = 0;
let SEED = 1;
let W = 128, H = 128;
let OPEN = null;     // Uint8Array passable
let FIELDS = [];     // {tx, ty, d: Int32Array} BFS distance-to-target fields
let CENTRE = [0, 0];
let OWN_SPAWN = [0, 0];
let FOE_SPAWN = [0, 0];
let OBJ = [];        // objectives, annotated in hello
let MEM = null;      // contact memory: id -> record
let PREV_HP = null;  // unit id -> hp last tick (damage detection)
let FOCUS_ID = 0;    // sticky focus target id
let FOCUS_TICK = -1e9;
let BACKOFF = null;  // unit id -> {until, steer}
let LAST_SMOKE = -1e9;
let LAST_FIRE_TICK = -1e9; // last tick any of our units took damage
let STALL = null;          // unit id -> {x, y, tick, forceUntil} (stuck watchdog)
let SPIN = null;           // unit id -> spin-stall detector/escape state (v5)
let ASSIGN_TICK = -1e9;
let POSTS = null;    // unit id -> [x, y] role post
let LAST_BATTLE_I = -1; // battle-point hysteresis (v2)
let RNGS = 1;

// v2 draft: UNCHANGED from v1 (drone + 2xMBT + scout). A 3xMBT deathball
// was built and A/B-tested (cup meta: top bots run it) but REJECTED on
// evidence: vs house bots it collapsed to 52.5% vs brawler (gate >60%) and
// 70% vs hunter, while the v1 draft with v2 code scored 95%/85%; vs the
// parker harness both drafts went 10-2. The drone/scout vision edge is
// worth more than a 3rd MBT in this controller's architecture. Flip for
// A/B testing only.
const DRAFT_PICK = ["drone", "mbt", "mbt", "scout"];

// ---------- small integer / angle helpers ----------
function clampI(v, lo, hi) { return v < lo ? lo : v > hi ? hi : v; }
function tileOf(v) { return v >> 16; }
function tileCentre(t) { return (t << 16) + HALF; }
function hyp(ax, ay, bx, by) { return Math.hypot(bx - ax, by - ay); }
function angTo(ax, ay, bx, by) {
  return Math.round(Math.atan2(by - ay, bx - ax) / (Math.PI * 2) * ANG) & (ANG - 1);
}
function angDelta(a, b) {
  let d = (b - a) & (ANG - 1);
  return d > ANG / 2 ? d - ANG : d;
}
function angNorm(a) { return ((a % ANG) + ANG) % ANG; }
function heightAt(x, y) {
  const tx = clampI(tileOf(x), 0, W - 1), ty = clampI(tileOf(y), 0, H - 1);
  return MAP.heights[ty * W + tx];
}
function terrainAtTile(tx, ty) {
  if (tx < 0 || ty < 0 || tx >= W || ty >= H) return -1;
  return MAP.terrain[ty * W + tx];
}
function isOpenTile(tx, ty) {
  if (tx < 0 || ty < 0 || tx >= W || ty >= H) return false;
  return OPEN[ty * W + tx] === 1;
}
function rngNext() {
  RNGS |= 0; RNGS = (RNGS + 0x6D2B79F5) | 0;
  let t = Math.imul(RNGS ^ (RNGS >>> 15), 1 | RNGS);
  t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
}
function specOf(kind) { return R.units[kind]; }

// ---------- map analysis (hello only) ----------
function nearestOpen(tx, ty) {
  if (isOpenTile(tx, ty)) return [tx, ty];
  for (let r = 1; r < 24; r++) {
    for (let dy = -r; dy <= r; dy++) for (let dx = -r; dx <= r; dx++) {
      if (Math.max(Math.abs(dx), Math.abs(dy)) !== r) continue;
      if (isOpenTile(tx + dx, ty + dy)) return [tx + dx, ty + dy];
    }
  }
  return [tx, ty];
}

// BFS distance field to (tx,ty) over open tiles, 8-connected with a corner
// rule (no diagonal squeeze between two closed tiles). Uniform cost.
function buildField(tx, ty) {
  const d = new Int32Array(W * H).fill(-1);
  const start = ty * W + tx;
  d[start] = 0;
  const q = new Int32Array(W * H);
  let qh = 0, qt = 0;
  q[qt++] = start;
  while (qh < qt) {
    const i = q[qh++];
    const x = i % W, y = (i - x) / W;
    const nd = d[i] + 1;
    for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
      if (!dx && !dy) continue;
      const nx = x + dx, ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
      const j = ny * W + nx;
      if (d[j] !== -1 || OPEN[j] !== 1) continue;
      if (dx && dy && (OPEN[y * W + nx] !== 1 || OPEN[ny * W + x] !== 1)) continue;
      d[j] = nd;
      q[qt++] = j;
    }
  }
  return d;
}

// Walk down the field gradient ONE tile: the waypoint is always adjacent,
// so steering to it can never cut a corner into rock or water.
function fieldWaypoint(f, ux, uy) {
  const x = clampI(tileOf(ux), 0, W - 1), y = clampI(tileOf(uy), 0, H - 1);
  const cur = f.d[y * W + x];
  if (cur <= 0) return null;
  let bx = x, by = y, bd = cur;
  for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
    if (!dx && !dy) continue;
    const nx = x + dx, ny = y + dy;
    if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
    const v = f.d[ny * W + nx];
    if (v >= 0 && v < bd) { bd = v; bx = nx; by = ny; }
  }
  if (bx === x && by === y) return null;
  return [tileCentre(bx), tileCentre(by)];
}

// v5 (escape only): walk up to maxSteps tiles down the field gradient,
// returning the path as tile centres (start tile excluded). Used ONLY while
// a spin-stall escape is engaged; normal routing never calls this.
function fieldPath(f, ux, uy, maxSteps) {
  let x = clampI(tileOf(ux), 0, W - 1), y = clampI(tileOf(uy), 0, H - 1);
  const pts = [];
  for (let s = 0; s < maxSteps; s++) {
    const cur = f.d[y * W + x];
    if (cur <= 0) break;
    let bx = x, by = y, bd = cur;
    for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
      if (!dx && !dy) continue;
      const nx = x + dx, ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
      const v = f.d[ny * W + nx];
      if (v >= 0 && v < bd) { bd = v; bx = nx; by = ny; }
    }
    if (bx === x && by === y) break;
    x = bx; y = by;
    pts.push([tileCentre(x), tileCentre(y)]);
  }
  return pts;
}

function segmentOpen(ax, ay, bx, by) {
  // Sample every tile: a 2-tile step can jump clean over a thin rock wall
  // and leave a unit driving into it forever.
  const dist = hyp(ax, ay, bx, by);
  const steps = Math.max(1, Math.ceil(dist / FP));
  for (let i = 1; i < steps; i++) {
    const t = i / steps;
    const x = Math.round(ax + (bx - ax) * t), y = Math.round(ay + (by - ay) * t);
    if (!isOpenTile(tileOf(x), tileOf(y))) return false;
  }
  return isOpenTile(tileOf(bx), tileOf(by));
}

// Mirror the engine's direct-shell flight: the shell climbs/dives from the
// muzzle (ground + eye height) toward the aim point (ground + 3). It stops
// if the ground rises above it, at a ruin, or after too many forest tiles.
function shotClear(u, spec, tx, ty) {
  const z0 = heightAt(u.x, u.y) + (spec.eyeHeight || 0);
  const z1 = heightAt(tx, ty) + 3;
  const dist = hyp(u.x, u.y, tx, ty);
  const steps = Math.max(1, Math.ceil(dist / (FP / 2)));
  const forestCap = R.forestStopsShellsAfter || 0;
  let forest = 0, lastTile = -1;
  for (let i = 1; i < steps; i++) {
    const t = i / steps;
    const x = u.x + (tx - u.x) * t, y = u.y + (ty - u.y) * t;
    const gx = tileOf(Math.round(x)), gy = tileOf(Math.round(y));
    if (gx < 0 || gy < 0 || gx >= W || gy >= H) return false;
    const tile = gy * W + gx;
    const z = z0 + (z1 - z0) * t;
    if (MAP.heights[tile] > z + 0.5) return false;
    if (tile !== lastTile) {
      lastTile = tile;
      const terr = MAP.terrain[tile];
      if (terr === 6) return false; // ruin blocks shells
      if (terr === 1 && forestCap > 0) {
        forest++;
        if (forest >= forestCap) return false;
      }
    }
  }
  return true;
}

function effRange(spec, u, tx, ty) {
  let r = spec.gun.maxRange;
  const ha = R.heightAdvantage;
  if (ha && heightAt(u.x, u.y) - heightAt(tx, ty) >= (ha.minDelta || 20)) {
    r = Math.round(r * (ha.rangeBonus || FP) / FP);
  }
  return r;
}

// ---------- contact memory ----------
function updateMemory(o) {
  const now = o.tick;
  const live = new Set();
  for (const c of o.contacts || []) {
    live.add(c.id);
    const prev = MEM.get(c.id);
    let vx = 0, vy = 0;
    if (prev && now > prev.tick) {
      const dt = now - prev.tick;
      vx = (c.x - prev.x) / dt; vy = (c.y - prev.y) / dt;
      // mutate in place: no per-tick allocation for re-seen contacts
      prev.x = c.x; prev.y = c.y; prev.heading = c.heading; prev.speed = c.speed || 0;
      prev.hpFrac = c.hpFrac; prev.tick = now; prev.live = true; prev.vx = vx; prev.vy = vy;
    } else {
      if (c.speed) {
        const a = c.heading / ANG * Math.PI * 2;
        vx = Math.cos(a) * c.speed; vy = Math.sin(a) * c.speed;
      }
      MEM.set(c.id, { id: c.id, kind: c.kind, x: c.x, y: c.y, heading: c.heading,
        speed: c.speed || 0, hpFrac: c.hpFrac, tick: now, live: true, vx, vy });
    }
  }
  for (const [id, m] of MEM) {
    if (!live.has(id)) {
      m.live = false;
      // dead-reckon briefly, then freeze: stale ghosts must not drag us around
      if (now - m.tick <= 40) { m.x += Math.round(m.vx * 1); m.y += Math.round(m.vy * 1); }
      if (now - m.tick > 400) MEM.delete(id);
    }
  }
}

const KIND_VALUE = { artillery: 5, missile: 5, mbt: 4, apc: 3, drone: 2.6, engineer: 2.4, scout: 2 };
function targetScore(m, shooters) {
  const value = KIND_VALUE[m.kind] || 2;
  const hurt = 1.5 - (m.hpFrac || 0) / FP;
  let inRange = 0;
  for (const s of shooters) {
    const d = hyp(s.x, s.y, m.x, m.y);
    if (d <= specOf(s.kind).gun.maxRange * 1.05) inRange++;
  }
  return value * hurt * (1 + 0.35 * inRange);
}

function chooseFocus(o, shooters) {
  let best = null, bestScore = -1;
  for (const m of MEM.values()) {
    if (!m.live) continue;
    const s = targetScore(m, shooters);
    if (s > bestScore) { bestScore = s; best = m; }
  }
  // sticky: keep the current focus unless clearly beaten (avoids flapping)
  if (FOCUS_ID) {
    const cur = MEM.get(FOCUS_ID);
    if (cur && cur.live) {
      const cs = targetScore(cur, shooters);
      if (cs >= bestScore * 0.8) return cur;
    }
  }
  if (best) { FOCUS_ID = best.id; FOCUS_TICK = o.tick; }
  else FOCUS_ID = 0;
  return best;
}

// ---------- objective / role assignment (recomputed at most every 20 ticks) ----------
function classifyObjectives(h) {
  OBJ = (h.objectives || []).map((o, i) => {
    const dOwn = hyp(o.x, o.y, OWN_SPAWN[0], OWN_SPAWN[1]);
    const dFoe = hyp(o.x, o.y, FOE_SPAWN[0], FOE_SPAWN[1]);
    const dCentre = hyp(o.x, o.y, CENTRE[0], CENTRE[1]);
    return { i, x: o.x, y: o.y, radius: o.radius, weight: o.weight,
      dOwn, dFoe, dCentre };
  });
}

function assignPosts(o, mine) {
  if (o.tick - ASSIGN_TICK < 20 && POSTS) return POSTS;
  ASSIGN_TICK = o.tick;
  const states = o.objectives || [];
  const heldByUs = (ob) => states[ob.i] && states[ob.i].owner === SIDE && states[ob.i].state === "held";
  const heldByFoe = (ob) => states[ob.i] && states[ob.i].owner === 1 - SIDE && states[ob.i].state === "held";
  const myScore = (o.score || [0, 0])[SIDE] || 0;
  const foeScore = (o.score || [0, 0])[1 - SIDE] || 0;
  const trailing = myScore < foeScore;
  const late = o.tick > 2400;

  let near = null, far = null;
  for (const ob of OBJ) {
    if (!near || ob.dOwn < near.dOwn) near = ob;
    if (!far || ob.dFoe < far.dFoe) far = ob;
  }
  const isCentre = (ob) => ob.dCentre < FP * 10;
  const foeNearOb = (ob) => {
    for (const m of MEM.values()) if (m.live && hyp(m.x, m.y, ob.x, ob.y) < FP * 26) return true;
    return false;
  };
  // Battle objective: push a foe-held point when trailing/late (a parked
  // APC scores double — sitting on a 1-point hill while it scores 6 loses),
  // defend a held point under attack, else take the centre, else hold near.
  // Push-abort: taking fire while blind means the foe-held point is a
  // fortress we cannot see into — do not feed it; hold our own points
  // until vision returns.
  let anyLive = false;
  for (const m of MEM.values()) if (m.live) { anyLive = true; break; }
  const blindFire = (o.tick - LAST_FIRE_TICK < 200) && !anyLive;
  let battle = null, battleScore = -1;
  for (const ob of OBJ) {
    let s;
    if (heldByFoe(ob)) s = ob.weight * (blindFire ? 0.7 : (trailing || late ? 3 : 1.3));
    else if (heldByUs(ob) && foeNearOb(ob)) s = ob.weight * 2;
    else if (!heldByUs(ob)) s = ob.weight * (isCentre(ob) ? 1.6 : 1.1);
    else s = ob.weight * 0.5;
    if (s > battleScore) { battleScore = s; battle = ob; }
  }
  if (!battle) battle = { i: -1, x: CENTRE[0], y: CENTRE[1], radius: FP * 5, weight: 1, dOwn: 0, dFoe: 0, dCentre: 0 };
  // v2: battle hysteresis. When two points tie (or ownership flickers
  // contested<->held), the battle point flaps every recompute and the
  // squad marches back and forth across the map while the foe scores
  // (parker kiln seed 4253586831: goal flipped #1<->#2 every ~100 ticks).
  // Keep the previous battle point unless it is now held by us (defended)
  // or a rival beats its current score by 25%+.
  const battleScoreOf = (ob) => {
    if (heldByFoe(ob)) return ob.weight * (blindFire ? 0.7 : (trailing || late ? 3 : 1.3));
    if (heldByUs(ob) && foeNearOb(ob)) return ob.weight * 2;
    if (!heldByUs(ob)) return ob.weight * (isCentre(ob) ? 1.6 : 1.1);
    return ob.weight * 0.5;
  };
  if (LAST_BATTLE_I >= 0 && battle.i !== LAST_BATTLE_I) {
    const prev = OBJ.find(ob => ob.i === LAST_BATTLE_I);
    if (prev && !heldByUs(prev) && battleScoreOf(prev) >= battleScore * 0.75) {
      battle = prev; battleScore = battleScoreOf(prev);
    }
  }
  LAST_BATTLE_I = battle.i;
  // v2 assault: when trailing (or very late), a foe-held battle point must
  // be TAKEN, not ringed from range-band distance. Evidence: in every
  // ladder loss the MBTs sat 5-10 tiles off foe-held hills for 3000+ ticks
  // while APCs scored unopposed (m_05c93d4d421e, m_dc36991e3b09,
  // m_f7bf72821673). Driving onto the point contests it (stops the scoring
  // at once) and closes to point-blank, where forest no longer blinds.
  const assault = battle.i >= 0 && heldByFoe(battle) && (trailing || o.tick > 3300);
  let second = null;
  for (const ob of OBJ) {
    if (ob === battle) continue;
    if (!second || ob.weight > second.weight) second = ob;
  }
  if (!second) second = battle;

  // Who is inside each point (for sole-holder logic).
  const inside = OBJ.map(() => []);
  for (const u of mine) {
    for (const ob of OBJ) {
      if (hyp(u.x, u.y, ob.x, ob.y) <= (ob.radius || FP * 5)) inside[ob.i].push(u.id);
    }
  }
  // MBT centroid, for the scout's screen position.
  let cx = 0, cy = 0, cn = 0;
  for (const u of mine) {
    if (u.kind === "mbt" || u.kind === "apc") { cx += u.x; cy += u.y; cn++; }
  }
  if (!cn) { cx = OWN_SPAWN[0] * 1; cy = OWN_SPAWN[1] * 1; cn = 1; }
  cx = Math.round(cx / cn); cy = Math.round(cy / cn);
  // Screen point: 13 tiles ahead of the heavies toward the freshest foe.
  let screenX = battle.x, screenY = battle.y;
  let freshest = null;
  for (const m of MEM.values()) if (o.tick - m.tick < 200 && (!freshest || m.tick > freshest.tick)) freshest = m;
  if (freshest) {
    const d = Math.max(1, hyp(cx, cy, freshest.x, freshest.y));
    screenX = Math.round(cx + (freshest.x - cx) / d * FP * 13);
    screenY = Math.round(cy + (freshest.y - cy) / d * FP * 13);
  }

  POSTS = new Map();
  let mbtSlot = 0;
  for (const u of mine) {
    if (u.kind === "drone") {
      POSTS.set(u.id, { kind: "overwatch", ob: battle });
    } else if (u.kind === "scout") {
      // Scout: cap/hold the near point; once someone else holds it, deny
      // the foe-held point or screen ahead of the heavies. Never suicide
      // into a point the whole foe squad is sitting on.
      // v2: if 2+ live foes are closing on the scout with no friendly heavy
      // nearby, fall back toward the heavies instead of sitting on the
      // point or screening forward into them (m_05c93d4d421e g1: the scout
      // died alone on obj#1 to APC secondaries).
      let foeClose = 0;
      for (const m of MEM.values()) if (m.live && hyp(m.x, m.y, u.x, u.y) < FP * 20) foeClose++;
      let heavyClose = false;
      for (const f of mine) {
        if ((f.kind === "mbt" || f.kind === "apc") && hyp(f.x, f.y, u.x, u.y) < FP * 25) { heavyClose = true; break; }
      }
      if (foeClose >= 2 && !heavyClose) {
        POSTS.set(u.id, { kind: "screen", ob: near,
          px: Math.round((u.x + OWN_SPAWN[0]) / 2), py: Math.round((u.y + OWN_SPAWN[1]) / 2) });
        continue;
      }
      let ob = near, px = 0, py = 0, kind = "cap";
      const nearHeld = near && heldByUs(near);
      const scoutInsideNear = near && inside[near.i].includes(u.id);
      const othersInsideNear = near ? inside[near.i].length - (scoutInsideNear ? 1 : 0) : 0;
      const foeAt = (ob2) => {
        let n = 0;
        for (const m of MEM.values()) if (m.live && hyp(m.x, m.y, ob2.x, ob2.y) < FP * 20) n++;
        return n;
      };
      if (near && !nearHeld && foeAt(near) >= 2) {
        kind = "screen"; px = screenX; py = screenY;
      } else if (nearHeld && othersInsideNear >= 1) {
        // free: deny the foe-held point if the push is on or it is weak
        let deny = null;
        for (const ob2 of OBJ) if (heldByFoe(ob2) && (!deny || ob2.weight > deny.weight)) deny = ob2;
        if (deny && (deny === battle || foeAt(deny) <= 1)) {
          ob = deny; kind = "deny";
          px = deny.x; py = deny.y;
        } else { kind = "screen"; px = screenX; py = screenY; }
      } else if (near) {
        ob = near;
        const a = (SIDE ? Math.PI : 0) + 0.9;
        px = Math.round(ob.x + Math.cos(a) * FP * 2); py = Math.round(ob.y + Math.sin(a) * FP * 2);
      } else { kind = "screen"; px = screenX; py = screenY; }
      POSTS.set(u.id, { kind, ob, px, py });
    } else if (u.kind === "artillery") {
      POSTS.set(u.id, { kind: "arty",
        px: Math.round(OWN_SPAWN[0] + (battle.x - OWN_SPAWN[0]) * 0.4),
        py: Math.round(OWN_SPAWN[1] + (battle.y - OWN_SPAWN[1]) * 0.4) });
    } else {
      // MBT / APC / missile / engineer: assaulting heavies go ONTO the
      // point (small spread, inside the cap radius); otherwise slots ring
      // the battle point, inside the cap radius but spread out (splash +
      // collision safety). Posts are snapped to open tiles: a post on a
      // rock/ruin tile can never be routed to and stalls the unit forever.
      const slot = mbtSlot++;
      const a = (slot * 2.4) + (SIDE ? 1.1 : 0.3);
      const rr = assault
        ? Math.min(FP * 2, (battle.radius || FP * 5) * 0.4)
        : Math.min(FP * 3.5, (battle.radius || FP * 5) * 0.65);
      let px = Math.round(battle.x + Math.cos(a) * rr), py = Math.round(battle.y + Math.sin(a) * rr);
      const [sx, sy] = nearestOpen(tileOf(px), tileOf(py));
      px = tileCentre(sx); py = tileCentre(sy);
      POSTS.set(u.id, { kind: assault ? "assault" : "battle", ob: battle, px, py });
    }
  }
  POSTS.battle = battle; POSTS.second = second; POSTS.assault = assault;
  return POSTS;
}

// ---------- movement ----------
function driveOrder(u, spec, wx, wy, opts) {
  // opts: {slow, holdFace} — returns {steer, throttle} integers.
  const want = angTo(u.x, u.y, wx, wy);
  let aim = want;
  // Probe ahead: if the tile 1.5 tiles out is closed, fan out.
  if (!spec.air) {
    const probe = (ang) => {
      const r = ang / ANG * Math.PI * 2;
      const px = u.x + Math.cos(r) * FP * 1.5, py = u.y + Math.sin(r) * FP * 1.5;
      return isOpenTile(tileOf(Math.round(px)), tileOf(Math.round(py)));
    };
    if (!probe(want)) {
      const offs = [341, -341, 683, -683, 1024, -1024, 1365, -1365];
      for (const off of offs) {
        const a = angNorm(want + off);
        if (probe(a)) { aim = a; break; }
      }
    }
  }
  const d = angDelta(u.heading, aim);
  const steer = clampI(Math.round(d * FP / Math.max(1, spec.turnRate)), -FP, FP);
  const ad = Math.abs(d);
  let throttle;
  if (opts && opts.hold) throttle = 0;
  else if (ad > 512) throttle = Math.round(FP * 0.15);
  else if (ad > 256) throttle = Math.round(FP * 0.5);
  else throttle = (opts && opts.slow) ? Math.round(FP * 0.6) : FP;
  return { steer, throttle };
}

// Route a ground unit toward (gx,gy): direct when the segment is open,
// otherwise down the cheapest precomputed flow field, then direct again.
// forceField skips the direct check (used after a stall: the direct line
// looked open but the unit could not progress along it).
// escape (v5): while a spin-stall escape is engaged, use gradient
// lookahead (the v4 routing) instead of the adjacent-tile waypoint.
// With escape=false this is byte-identical to v2.
function routeWaypoint(u, spec, gx, gy, forceField, escape) {
  if (spec.air) return [gx, gy];
  // v2: snap a goal on a closed tile (rock/ruin) to the nearest open tile.
  // An unreachable goal defeats every flow field (goal distance -1), so
  // the unit falls back to driving straight into the obstacle forever.
  if (!isOpenTile(tileOf(gx), tileOf(gy))) {
    const [nx, ny] = nearestOpen(tileOf(gx), tileOf(gy));
    gx = tileCentre(nx); gy = tileCentre(ny);
  }
  const d = hyp(u.x, u.y, gx, gy);
  if (d < FP * 2.5) return [gx, gy];
  if (!forceField && segmentOpen(u.x, u.y, gx, gy)) return [gx, gy];
  const ux = clampI(tileOf(u.x), 0, W - 1), uy = clampI(tileOf(u.y), 0, H - 1);
  const gxT = clampI(tileOf(gx), 0, W - 1), gyT = clampI(tileOf(gy), 0, H - 1);
  let bestF = null, bestScore = Infinity;
  for (const f of FIELDS) {
    const du = f.d[uy * W + ux], dg = f.d[gyT * W + gxT];
    // v5: in escape mode, skip a field whose target the unit already sits
    // on (du <= 0, the v4 rule); in normal mode keep v2's exact gate.
    if ((escape ? du <= 0 : du < 0) || dg < 0) continue;
    const s = du + dg;
    if (s < bestScore) { bestScore = s; bestF = f; }
  }
  if (bestF) {
    if (escape) {
      // v5 escape: lookahead. Take the FARTHEST gradient point with an
      // open straight segment from the unit (tried far-to-near), so the
      // bearing stays stable across ticks. Falls back to the adjacent
      // point only when no farther point has an open segment.
      const pts = fieldPath(bestF, u.x, u.y, 6);
      for (let i = pts.length - 1; i >= 0; i--) {
        if (segmentOpen(u.x, u.y, pts[i][0], pts[i][1])) return pts[i];
      }
      if (pts.length) return pts[0];
    } else {
      const du = bestF.d[uy * W + ux];
      if (du > 3) {
        const wp = fieldWaypoint(bestF, u.x, u.y);
        if (wp) return wp;
      }
    }
  }
  return [gx, gy];
}

function dodgeGoal(u, o) {
  for (const inc of o.incoming || []) {
    const d = hyp(u.x, u.y, inc.x, inc.y);
    if (d < FP * 4.5 && inc.ticks < 45) {
      const a = angTo(inc.x, inc.y, u.x, u.y);
      const r = a / ANG * Math.PI * 2;
      return [Math.round(u.x + Math.cos(r) * FP * 7), Math.round(u.y + Math.sin(r) * FP * 7)];
    }
  }
  return null;
}

// ---------- firing ----------
function predictPoint(m, flightTicks) {
  const px = m.x + Math.round(m.vx * flightTicks);
  const py = m.y + Math.round(m.vy * flightTicks);
  return [clampI(px, FP, (W << 16) - FP), clampI(py, FP, (H << 16) - FP)];
}

function directFire(u, spec, target, order) {
  // target: live memory record. Aims with lead; fires only when aligned,
  // in (height-adjusted) range, and the shell line is clear.
  const d = hyp(u.x, u.y, target.x, target.y);
  const range = effRange(spec, u, target.x, target.y);
  const flight = Math.max(1, Math.round(d / Math.max(1, spec.gun.shellSpeed)));
  const [px, py] = predictPoint(target, flight);
  order.aimAt = [px, py];
  if (u.reload !== 0) return;
  if (d > range) return;
  if (d < (spec.gun.minRange || 0)) return;
  const want = angTo(u.x, u.y, px, py);
  if (Math.abs(angDelta(u.turret, want)) > spec.gun.aimTolerance) return;
  if (!shotClear(u, spec, px, py)) return;
  order.fire = [px, py];
}

// ---------- per-unit orders ----------
function unitOrder(u, o, ctx) {
  const spec = specOf(u.kind);
  const order = { id: u.id };
  const hpFrac = u.hp / Math.max(1, u.maxHp);
  const post = ctx.posts.get(u.id);
  const focus = ctx.focus;
  // Nearest live foe to THIS unit (memory records are squad-shared vision).
  let foeHere = null;
  for (const m of MEM.values()) {
    if (!m.live) continue;
    const d = hyp(u.x, u.y, m.x, m.y);
    if (!foeHere || d < foeHere.d) foeHere = { x: m.x, y: m.y, kind: m.kind, d };
  }

  // Backoff after driving into something, with a grace period so a unit
  // stuck against a friendly does not reverse-spin forever.
  const bo = BACKOFF.get(u.id);
  if (bo && o.tick < bo.until) {
    order.throttle = Math.round(-FP * 0.7);
    order.steer = bo.steer;
    return order;
  }
  if (u.blocked && o.canMove && (!bo || o.tick >= bo.until + 10)) {
    BACKOFF.set(u.id, { until: o.tick + 12, steer: (u.id % 2 ? FP : -FP) });
    order.throttle = Math.round(-FP * 0.7);
    order.steer = (u.id % 2 ? FP : -FP);
    return order;
  }

  // Per-unit lateral offset for shared goals (probe/chase): heavies attack
  // on parallel bearings instead of converging on one point and colliding.
  const spreadGoal = (gx, gy) => {
    const off = ctx.spread.get(u.id) || 0;
    if (!off) return [gx, gy];
    const dx = gx - u.x, dy = gy - u.y;
    const len = Math.max(1, Math.hypot(dx, dy));
    return [Math.round(gx + (-dy / len) * off * FP), Math.round(gy + (dx / len) * off * FP)];
  };

  // 1) Dodge incoming artillery — highest movement priority.
  let goal = dodgeGoal(u, o);
  let mode = goal ? "dodge" : null;
  let hold = false, slow = false, faceX = null, faceY = null;

  // 2) Retreat when badly hurt and threatened (a live foe close by, or
  // taking damage from an unseen shooter).
  const nearestFoe = foeHere;
  const threatened = (nearestFoe && nearestFoe.d < FP * 26) ||
    (ctx.tookFire.has(u.id) && !(focus && focus.live));
  if (!mode && threatened) {
    const limit = u.kind === "scout" || u.kind === "drone" ? 0.45 : 0.3;
    if (hpFrac < limit) {
      mode = "retreat";
      // fall back toward our spawn, but stay near the fight if allies hold it
      goal = [Math.round((u.x + OWN_SPAWN[0]) / 2), Math.round((u.y + OWN_SPAWN[1]) / 2)];
      if (hpFrac < 0.18) goal = [OWN_SPAWN[0], OWN_SPAWN[1]];
    }
  }

  // 3) Ammo: finite-ammo units with empty magazines go home to resupply.
  if (!mode && spec.ammo !== undefined && u.ammo === 0) {
    mode = "resupply";
    const dh = hyp(u.x, u.y, OWN_SPAWN[0], OWN_SPAWN[1]);
    if (dh < FP * 2.5) { hold = true; }
    else goal = [OWN_SPAWN[0], OWN_SPAWN[1]];
  }

  // 3b) Probe: blind but with a fresh memory — advance to re-acquire.
  // Sitting still while unseen foes shoot is how squads die for 0 damage.
  if (!mode && u.kind !== "drone" && u.kind !== "artillery" &&
      !(focus && focus.live) && ctx.memoryTarget && !ctx.soleHold.has(u.id)) {
    const age = o.tick - ctx.memoryTarget.tick;
    const maxAge = ctx.tookFire.has(u.id) ? 300 : 150;
    const md = hyp(u.x, u.y, ctx.memoryTarget.x, ctx.memoryTarget.y);
    if (age <= maxAge && md <= FP * 55) {
      mode = "probe";
      goal = spreadGoal(ctx.memoryTarget.x, ctx.memoryTarget.y);
      faceX = ctx.memoryTarget.x; faceY = ctx.memoryTarget.y;
    }
  }

  // 4) Role behaviour.
  if (!mode) {
    if (u.kind === "drone") {
      // v2 scouting anchor: when the squad is blind, the drone checks the
      // objectives themselves. Forest shreds even its 36-tile sensor at
      // range (x0.85 per forest tile), so it must fly close overhead to
      // see parkers; sitting on the point also contests it. Foe-held
      // highest-weight first, else nearest unheld. (v1 anchored 20 tiles
      // ahead of the heavies toward the foe spawn and never checked the
      // hills — m_05c93d4d421e: the drone wandered the north forest while
      // APCs scored unopposed.)
      let anchor = null;
      const squadBlind = ctx.liveCount === 0;
      const states = o.objectives || [];
      if ((ctx.blindUnderFire || (o.tick > 500 && squadBlind)) && !ctx.soleHold.has(u.id)) {
        let bestOb = null, bestScore = -1;
        for (const ob of OBJ) {
          const st = states[ob.i];
          if (st && st.owner === SIDE && st.state === "held") continue;
          const foeHeld = st && st.owner === 1 - SIDE && st.state === "held";
          const d = hyp(u.x, u.y, ob.x, ob.y);
          const s = ob.weight * (foeHeld ? 3 : 1) - d / FP * 0.05;
          if (s > bestScore) { bestScore = s; bestOb = ob; }
        }
        if (bestOb) anchor = [bestOb.x, bestOb.y];
        else {
          let hx = 0, hy = 0, hn = 0;
          for (const f of o.units) {
            if (f.alive && f.kind !== "drone" && f.kind !== "scout") { hx += f.x; hy += f.y; hn++; }
          }
          if (!hn) { hx = u.x; hy = u.y; hn = 1; }
          hx /= hn; hy /= hn;
          const tx = ctx.memoryTarget ? ctx.memoryTarget.x : FOE_SPAWN[0];
          const ty = ctx.memoryTarget ? ctx.memoryTarget.y : FOE_SPAWN[1];
          const dd = Math.max(1, hyp(hx, hy, tx, ty));
          anchor = [Math.round(hx + (tx - hx) / dd * FP * 20), Math.round(hy + (ty - hy) / dd * FP * 20)];
        }
      }
      // Threat-scaled standoff: only missile/scout/drone spotters can lock
      // (37); heavies out-sensor us (31); APCs/engineers have short sensors
      // and weak secondaries (9) — and while scouting an objective at
      // healthy HP the drone does not flee them at all: it must get close
      // to see through forest, and contesting the point is worth the risk.
      let fleeX = null, fleeY = null, fleeD = Infinity;
      for (const m of MEM.values()) {
        if (!m.live) continue;
        const lowThreat = m.kind === "apc" || m.kind === "engineer";
        if (lowThreat && anchor && hpFrac >= 0.5) continue;
        const req = (m.kind === "scout" || m.kind === "drone" || m.kind === "missile") ? FP * 37
          : lowThreat ? FP * 9 : FP * 31;
        const d = hyp(u.x, u.y, m.x, m.y);
        if (d < req - FP * 2 && d < fleeD) { fleeD = d; fleeX = m.x; fleeY = m.y; }
      }
      if (fleeX !== null) {
        const a = angTo(fleeX, fleeY, u.x, u.y), r = a / ANG * Math.PI * 2;
        goal = [Math.round(u.x + Math.cos(r) * FP * 8), Math.round(u.y + Math.sin(r) * FP * 8)];
      } else if (ctx.soleHold.has(u.id) && !ctx.blindUnderFire) {
        hold = true; // sole holder of a point we own: stay and score
      } else if (anchor) {
        goal = anchor;
        if (hyp(u.x, u.y, goal[0], goal[1]) < FP * 3) hold = true;
      } else {
        goal = post && post.ob ? [post.ob.x, post.ob.y] : [CENTRE[0], CENTRE[1]];
        if (hyp(u.x, u.y, goal[0], goal[1]) < FP * 3) hold = true;
      }
      mode = "drone";
    } else if (u.kind === "artillery") {
      // Stand off at ~40% of the way to the fight; keep foes outside min range.
      const foe = foeHere;
      if (foe && foe.d < (spec.gun.minRange || 0) + FP * 6) {
        const a = angTo(foe.x, foe.y, u.x, u.y), r = a / ANG * Math.PI * 2;
        goal = [Math.round(u.x + Math.cos(r) * FP * 8), Math.round(u.y + Math.sin(r) * FP * 8)];
      } else if (post) {
        goal = [post.px, post.py];
        if (hyp(u.x, u.y, goal[0], goal[1]) < FP * 3) hold = true;
      }
      mode = "arty";
    } else if (focus && focus.live) {
      // Combat: range-band positioning around the focus target.
      const d = hyp(u.x, u.y, focus.x, focus.y);
      const range = effRange(spec, u, focus.x, focus.y);
      const lo = range * 0.55, hi = range * 0.88;
      const leash = (ctx.hunt || ctx.assault) ? FP * 200 : FP * 42;
      const postD = post && post.ob ? hyp(focus.x, focus.y, post.ob.x, post.ob.y) : 0;
      const onAssaultPoint = ctx.assault && post && post.ob &&
        hyp(u.x, u.y, post.ob.x, post.ob.y) <= (post.ob.radius || FP * 5) + FP * 4;
      // v2b: the no-kite rule extends to ALL heavies for the whole assault,
      // not just units already on the point. Evidence (parker apc4 seed
      // 4253586831): contacts flicker live/off through forest, so the MBTs
      // probed one tick and kited the next — net movement zero for 3600+
      // ticks, 8-12 tiles from parked APCs, with every shell line blocked
      // by >4 forest tiles. Closing is the mission; heavies hold and fight.
      const heavy = u.kind === "mbt" || u.kind === "apc" || u.kind === "missile";
      const noKite = onAssaultPoint || (ctx.assault && heavy);
      if (u.kind === "scout" && (focus.kind === "mbt" || focus.kind === "apc") && ctx.mbtNear) {
        // Scouts do ~1 damage to MBT fronts: leave the duel, go cap/deny.
        if (post) { goal = [post.px, post.py]; if (hyp(u.x, u.y, goal[0], goal[1]) < FP * 3) hold = true; }
        mode = "cap";
      } else if (d > hi && postD < leash) {
        goal = spreadGoal(focus.x, focus.y);
        mode = "chase";
      } else if (d < lo && !noKite) {
        // back away while facing the threat: front armour stays toward it
        const a = angTo(focus.x, focus.y, u.x, u.y), r = a / ANG * Math.PI * 2;
        goal = [Math.round(u.x + Math.cos(r) * FP * 6), Math.round(u.y + Math.sin(r) * FP * 6)];
        faceX = focus.x; faceY = focus.y;
        mode = "kite";
      } else {
        // In the band: hold and shoot — unless a ridge/ruin blocks the
        // shell line, in which case advance to clear the obstruction.
        let blocked = false;
        if (spec.gun.mode === "direct") {
          const flight = Math.max(1, Math.round(d / Math.max(1, spec.gun.shellSpeed)));
          const [px, py] = predictPoint(focus, flight);
          blocked = !shotClear(u, spec, px, py);
        }
        if (blocked) { goal = spreadGoal(focus.x, focus.y); mode = "chase"; }
        else { hold = true; faceX = focus.x; faceY = focus.y; mode = "fight"; }
      }
    } else if (!ctx.memoryTarget && o.tick - LAST_FIRE_TICK < 100 &&
        u.kind !== "drone" && u.kind !== "artillery" && !ctx.soleHold.has(u.id)) {
      // Blind and recently shot, never saw the shooter: advance toward
      // the foe side to gain vision instead of sitting on the post.
      goal = [Math.round((u.x + FOE_SPAWN[0]) / 2), Math.round((u.y + FOE_SPAWN[1]) / 2)];
      mode = "advance";
    } else {
      // No live focus: go to the role post (cap / hold / hunt memory).
      if (ctx.hunt && ctx.memoryTarget) {
        goal = [ctx.memoryTarget.x, ctx.memoryTarget.y];
        mode = "hunt";
      } else if (post) {
        if (post.kind === "overwatch") { goal = [post.ob.x, post.ob.y]; }
        else { goal = [post.px, post.py]; }
        if (hyp(u.x, u.y, goal[0], goal[1]) < FP * 2.5) hold = true;
        mode = "post";
      }
      if (ctx.memoryTarget) { faceX = ctx.memoryTarget.x; faceY = ctx.memoryTarget.y; }
    }
  }

  // 5) Separation: don't stack on a friendly hull (splash + collisions).
  if (goal && !hold && !spec.air) {
    for (const f of o.units) {
      if (f.id === u.id || !f.alive || specOf(f.kind).air) continue;
      const fd = hyp(u.x, u.y, f.x, f.y);
      if (fd < FP * 3.0 && fd > 0) {
        const a = angTo(f.x, f.y, u.x, u.y) + (u.id % 2 ? 512 : -512);
        const r = angNorm(a) / ANG * Math.PI * 2;
        goal = [Math.round(goal[0] + Math.cos(r) * FP * 2), Math.round(goal[1] + Math.sin(r) * FP * 2)];
        break;
      }
    }
  }

  // Keep every goal inside the map.
  if (goal) goal = [clampI(goal[0], FP, (W << 16) - FP), clampI(goal[1], FP, (H << 16) - FP)];

  // Stall watchdog: if the unit has barely moved in 80 ticks while trying
  // to reach a distant goal, force flow-field routing for a while (the
  // direct line looked open but the unit could not progress along it).
  let forceField = false;
  if (goal && !hold && o.canMove) {
    const gd = hyp(u.x, u.y, goal[0], goal[1]);
    let rec = STALL.get(u.id);
    if (!rec) { rec = { x: u.x, y: u.y, tick: o.tick, forceUntil: -1 }; STALL.set(u.id, rec); }
    forceField = o.tick < rec.forceUntil;
    if (gd > FP * 6) {
      if (o.tick - rec.tick >= 80) {
        if (hyp(u.x, u.y, rec.x, rec.y) < FP * 1.5) rec.forceUntil = o.tick + 300;
        rec.x = u.x; rec.y = u.y; rec.tick = o.tick;
      }
    } else { rec.x = u.x; rec.y = u.y; rec.tick = o.tick; }
  }

  // v5: spin-stall detector + escape. Pure observation until triggered:
  // while untriggered it changes NO order. A ground unit trying to reach
  // a distant goal whose heading rotates ~monotonically >=315 deg over a
  // trailing 120-tick window while displacing <3 tiles is in the proven
  // spin stall (NOTES-v4-build.md §2); calibration on 8 cup replays
  // (reps-v5/calib3-v5.mjs) fires on every genuine stall (4347 g1/g2,
  // 36db g1/g2, aa59 g1) and never on healthy driving (a deliberate
  // about-face is <=180 deg and displaces). The escape engages v4-style
  // lookahead routing until the direct line to the goal opens, the goal
  // is reached, 900 ticks pass, or the escape itself makes no progress
  // for 120 ticks. Disengage resets the window: a re-trigger needs a
  // fresh 120 ticks of spinning, so the escape cannot flap.
  let escaping = false;
  {
    let sp = SPIN.get(u.id);
    if (!sp) {
      sp = { deltas: new Int16Array(120), px: new Int32Array(120), py: new Int32Array(120),
             idx: 0, n: 0, cum: 0, abs: 0, prevH: u.heading,
             esc: false, escAt: 0, lx: u.x, ly: u.y, lt: o.tick, openN: 0 };
      SPIN.set(u.id, sp);
    }
    const trying = goal && !hold && o.canMove && !spec.air;
    const resetWin = () => { sp.n = 0; sp.cum = 0; sp.abs = 0; sp.idx = 0; sp.prevH = u.heading; };
    if (!trying) {
      sp.esc = false; resetWin();
    } else {
      const gd = hyp(u.x, u.y, goal[0], goal[1]);
      if (sp.esc) {
        // Disengage: goal reached, escape budget spent, the escape makes
        // no progress, or the direct line to the goal stays open for 30
        // consecutive ticks (v2 routing is safe again). The open-line
        // test MUST be consecutive: segmentOpen flickers tick-to-tick at
        // obstacle edges, and that flicker is part of the stall — a
        // single open tick must not kill the escape (36db g1 lesson).
        if (segmentOpen(u.x, u.y, goal[0], goal[1])) sp.openN++; else sp.openN = 0;
        let done = gd <= FP * 6 || o.tick - sp.escAt > 900 || sp.openN >= 30;
        if (!done && o.tick - sp.lt >= 120) {
          if (hyp(u.x, u.y, sp.lx, sp.ly) < FP * 1.0) done = true;
          sp.lx = u.x; sp.ly = u.y; sp.lt = o.tick;
        }
        if (done) { sp.esc = false; resetWin(); }
        else escaping = true;
        sp.prevH = u.heading;
      } else if (gd <= FP * 6) {
        resetWin();
      } else {
        // Feed the trailing 120-tick window (O(1): subtract the outgoing
        // delta, add the new one; displacement from the outgoing sample).
        const slot = sp.idx;
        let oldX = u.x, oldY = u.y;
        if (sp.n === 120) {
          sp.cum -= sp.deltas[slot]; sp.abs -= Math.abs(sp.deltas[slot]);
          oldX = sp.px[slot]; oldY = sp.py[slot];
        }
        const dh = angDelta(sp.prevH, u.heading);
        sp.prevH = u.heading;
        sp.deltas[slot] = dh; sp.px[slot] = u.x; sp.py[slot] = u.y;
        sp.idx = (slot + 1) % 120;
        if (sp.n < 120) sp.n++;
        sp.cum += dh; sp.abs += Math.abs(dh);
        if (sp.n === 120 && Math.abs(sp.cum) >= 3584 && sp.abs > 0 &&
            Math.abs(sp.cum) / sp.abs >= 0.8 && hyp(oldX, oldY, u.x, u.y) < FP * 3) {
          sp.esc = true; sp.escAt = o.tick; sp.lx = u.x; sp.ly = u.y; sp.lt = o.tick; sp.openN = 0;
          escaping = true;
        }
      }
    }
  }

  // 6) Movement orders.
  if (o.canMove) {
    if (hold) {
      order.throttle = 0;
      if (faceX !== null) {
        const want = angTo(u.x, u.y, faceX, faceY);
        const dd = angDelta(u.heading, want);
        order.steer = clampI(Math.round(dd * FP / Math.max(1, spec.turnRate)), -FP, FP);
      } else if (ctx.memoryTarget) {
        const want = angTo(u.x, u.y, ctx.memoryTarget.x, ctx.memoryTarget.y);
        const dd = angDelta(u.heading, want);
        order.steer = clampI(Math.round(dd * FP / Math.max(1, spec.turnRate)), -FP, FP);
      }
    } else if (goal) {
      if (mode === "kite" && faceX !== null && spec.gun.mode === "direct" && !spec.fixedGun && u.kind !== "scout") {
        // reversing MBT: keep the hull facing the threat, drive backwards
        const want = angTo(u.x, u.y, faceX, faceY);
        const dd = angDelta(u.heading, want);
        order.steer = clampI(Math.round(dd * FP / Math.max(1, spec.turnRate)), -FP, FP);
        order.throttle = Math.abs(dd) > 256 ? 0 : Math.round(-FP * 0.8);
      } else {
        const [wx, wy] = routeWaypoint(u, spec, goal[0], goal[1], forceField, escaping);
        const dv = driveOrder(u, spec, wx, wy, { slow: slow || mode === "fight" });
        order.steer = dv.steer; order.throttle = dv.throttle;
      }
    }
  }

  // 7) Weapons.
  if (spec.gun.lockTicks) {
    // Missile: keep the lock on the focus target while it is visible.
    if (focus && focus.live) {
      const d = hyp(u.x, u.y, focus.x, focus.y);
      order.target = focus.id;
      order.aimAt = [focus.x, focus.y];
      const worthIt = u.ammo > 1 || focus.kind === "mbt" || focus.kind === "artillery" || focus.kind === "missile" || focus.kind === "apc";
      if (u.lockTarget === focus.id && u.lockTicks >= spec.gun.lockTicks &&
          u.reload === 0 && u.ammo !== 0 && worthIt &&
          d >= (spec.gun.minRange || 0) && d <= spec.gun.maxRange) {
        order.fire = true;
      }
    }
  } else if (spec.gun.mode === "direct") {
    // Fire the focus target if in range, else the best in-range target.
    let tgt = null;
    if (focus && focus.live) {
      const d = hyp(u.x, u.y, focus.x, focus.y);
      if (d <= effRange(spec, u, focus.x, focus.y)) tgt = focus;
    }
    if (!tgt) {
      let bs = -1;
      for (const m of MEM.values()) {
        if (!m.live) continue;
        const d = hyp(u.x, u.y, m.x, m.y);
        if (d > effRange(spec, u, m.x, m.y)) continue;
        const s = targetScore(m, [u]);
        if (s > bs) { bs = s; tgt = m; }
      }
    }
    if (!tgt) {
      // v2: fire at fresh STATIONARY memories. Parked enemies (APCs sitting
      // on hills) barely move, so a recently-seen position stays true even
      // when forest flicker hides them this tick. Evidence: v1 aimed at
      // remembered APC positions for 3000+ ticks and never fired once
      // (m_05c93d4d421e orders show aimAt with no fire field). Gated hard:
      // seen within 120 ticks and essentially motionless (velocity from
      // position deltas near zero); directFire re-checks range, turret
      // alignment and the shell line before any shot.
      let bs = -1;
      for (const m of MEM.values()) {
        if (m.live) continue;
        if (o.tick - m.tick > 120) continue;
        if (Math.abs(m.vx || 0) + Math.abs(m.vy || 0) >= 500) continue;
        const d = hyp(u.x, u.y, m.x, m.y);
        if (d > effRange(spec, u, m.x, m.y)) continue;
        const s = targetScore(m, [u]);
        if (s > bs) { bs = s; tgt = m; }
      }
    }
    if (tgt) directFire(u, spec, tgt, order);
    else if (focus && focus.live) order.aimAt = [focus.x, focus.y];
    else if (ctx.memoryTarget) order.aimAt = [ctx.memoryTarget.x, ctx.memoryTarget.y];
  } else if (spec.gun.mode === "indirect") {
    // Artillery: fire at live contacts (led for the long flight), or at a
    // fresh memory position; conserve the 10 rounds for valuable targets.
    let shoot = null;
    if (focus && focus.live) shoot = focus;
    else if (ctx.memoryTarget && o.tick - ctx.memoryTarget.tick < 40) shoot = ctx.memoryTarget;
    if (shoot && u.reload === 0 && u.ammo !== 0) {
      const d = hyp(u.x, u.y, shoot.x, shoot.y);
      if (d >= (spec.gun.minRange || 0) && d <= spec.gun.maxRange) {
        const valuable = shoot.kind === "mbt" || shoot.kind === "artillery" ||
          shoot.kind === "missile" || shoot.kind === "apc" || u.ammo > 4;
        if (valuable) {
          const flight = Math.round(2 * (R.tickHz || 20) + d / FP * ((R.tickHz || 20) * 0.05));
          const [px, py] = shoot.live ? predictPoint(shoot, flight) : [shoot.x, shoot.y];
          order.fire = [px, py];
        }
      }
    }
    // Secondary MG at close range.
    if (!order.fire && spec.secondary && u.reload2 === 0 && nearestFoeIn(o, u, spec.secondary.maxRange)) {
      const c = nearestFoeIn(o, u, spec.secondary.maxRange);
      const want = angTo(u.x, u.y, c.x, c.y);
      if (Math.abs(angDelta(u.turret, want)) <= spec.secondary.aimTolerance) order.fire = true;
      else order.aimAt = [c.x, c.y];
    }
  } else if (spec.secondary) {
    // Drone has no weapon; engineer MG.
    if (u.kind !== "drone" && u.reload2 === 0) {
      const c = nearestFoeIn(o, u, spec.secondary.maxRange);
      if (c) {
        const want = angTo(u.x, u.y, c.x, c.y);
        if (Math.abs(angDelta(u.turret, want)) <= spec.secondary.aimTolerance) order.fire = true;
        else order.aimAt = [c.x, c.y];
      }
    }
  }

  // 8) Engineer abilities.
  if (u.kind === "engineer") {
    if (nearestFoe && nearestFoe.d < FP * 16 && u.charges.smoke > 0 && o.tick - LAST_SMOKE > 200) {
      order.ability = "smoke";
      LAST_SMOKE = o.tick;
    } else if (u.charges.mine > 0 && post && post.ob &&
        hyp(u.x, u.y, post.ob.x, post.ob.y) < FP * 9 && o.tick % 30 === 0) {
      order.ability = "mine";
    }
  }
  return order;
}

function nearestFoeIn(o, u, range) {
  let best = null, bd = Infinity;
  for (const c of o.contacts || []) {
    const d = hyp(u.x, u.y, c.x, c.y);
    if (d <= range && d < bd) { bd = d; best = c; }
  }
  return best;
}

// ---------- controller ----------
globalThis.controller = {
  draft(h) {
    const d = h.draft;
    if (!d) return undefined;
    // v2: draft unchanged from v1 (3xMBT tested and rejected on evidence).
    const pick = DRAFT_PICK.slice();
    const cost = pick.reduce((c, k) => c + (d.costs[k] ?? 99), 0);
    if (cost > d.budget || pick.length < d.min || pick.length > d.max) return undefined;
    if (!pick.every(k => d.roster.includes(k))) return undefined;
    return pick;
  },

  hello(h) {
    R = h.rules; MAP = h.map; SIDE = h.side; SEED = h.seed >>> 0;
    W = MAP.width; H = MAP.height;
    RNGS = (SEED ^ (SIDE * 2654435761)) >>> 0 || 1;
    MEM = new Map(); BACKOFF = new Map(); PREV_HP = new Map(); STALL = new Map(); SPIN = new Map();
    LAST_FIRE_TICK = -1e9;
    FOCUS_ID = 0; FOCUS_TICK = -1e9; LAST_SMOKE = -1e9; ASSIGN_TICK = -1e9; POSTS = null; LAST_BATTLE_I = -1;

    CENTRE = [tileCentre(W >> 1), tileCentre(H >> 1)];
    const s0 = MAP.spawns[0], s1 = MAP.spawns[1];
    OWN_SPAWN = [tileCentre(s0[0]), tileCentre(s0[1])];
    FOE_SPAWN = [tileCentre(s1[0]), tileCentre(s1[1])];
    if (SIDE === 1) { const t = OWN_SPAWN; OWN_SPAWN = FOE_SPAWN; FOE_SPAWN = t; }

    // Passability straight from the rules (covers future terrain classes).
    OPEN = new Uint8Array(W * H);
    const speeds = R.terrainSpeed || {};
    for (let i = 0; i < W * H; i++) {
      const c = MAP.terrain[i];
      const v = speeds[c] ?? speeds[String(c)];
      OPEN[i] = (v !== undefined ? v > 0 : (c !== 2 && c !== 4 && c !== 6 && c !== 8)) ? 1 : 0;
    }

    classifyObjectives(h);
    // Flow fields: every objective, the map centre, both spawns.
    FIELDS = [];
    const targets = [];
    for (const ob of OBJ) targets.push([tileOf(ob.x), tileOf(ob.y)]);
    targets.push([W >> 1, H >> 1]);
    targets.push([s0[0], s0[1]], [s1[0], s1[1]]);
    const seen = new Set();
    for (const [tx, ty] of targets) {
      const [nx, ny] = nearestOpen(tx, ty);
      const key = ny * W + nx;
      if (seen.has(key)) continue;
      seen.add(key);
      FIELDS.push({ tx: nx, ty: ny, d: buildField(nx, ny) });
    }
  },

  tick(o) {
    updateMemory(o);
    const mine = (o.units || []).filter(u => u.alive);
    const shooters = mine.filter(u => specOf(u.kind).gun.mode === "direct");
    const focus = chooseFocus(o, shooters.length ? shooters : mine);
    const posts = assignPosts(o, mine);

    // Damage detection: a unit whose hp fell since last tick is under fire
    // (even when the shooter is unseen — the classic blind-missile case).
    const tookFire = new Set();
    for (const u of o.units || []) {
      const prev = PREV_HP.get(u.id);
      if (u.alive && prev !== undefined && u.hp < prev) { tookFire.add(u.id); LAST_FIRE_TICK = o.tick; }
      PREV_HP.set(u.id, u.hp);
    }
    // Sole holders: the only friendly unit inside a point we hold. They
    // must not abandon it to chase ghosts.
    const soleHold = new Set();
    const states = o.objectives || [];
    for (const ob of OBJ) {
      const st = states[ob.i];
      if (!st || st.owner !== SIDE || st.state !== "held") continue;
      const insideIds = [];
      for (const u of mine) {
        if (hyp(u.x, u.y, ob.x, ob.y) <= (ob.radius || FP * 5)) insideIds.push(u.id);
      }
      if (insideIds.length === 1) soleHold.add(insideIds[0]);
    }
    // Freshest memory target for hunts / probes.
    let memoryTarget = null;
    for (const m of MEM.values()) {
      if (o.tick - m.tick < 300 && (!memoryTarget || m.tick > memoryTarget.tick)) memoryTarget = m;
    }
    const myScore = (o.score || [0, 0])[SIDE] || 0;
    const foeScore = (o.score || [0, 0])[1 - SIDE] || 0;
    const hunt = o.tick > 3300 && myScore <= foeScore;
    const mbtNear = mine.some(u => u.kind === "mbt");
    // Spread index for combat units: probe/chase goals get a per-unit
    // lateral offset so heavies never converge on one point and deadlock.
    const combat = mine.filter(u => u.kind !== "drone" && u.kind !== "artillery");
    const spread = new Map();
    combat.forEach((u, i) => spread.set(u.id, (i - (combat.length - 1) / 2) * 5));
    let liveCount = 0;
    for (const m of MEM.values()) if (m.live) liveCount++;
    const blindUnderFire = tookFire.size > 0 && !focus?.live &&
      (!memoryTarget || o.tick - memoryTarget.tick > 150);
    const ctx = { focus, posts, memoryTarget, hunt, mbtNear, tookFire, soleHold,
      spread, liveCount, blindUnderFire, assault: !!posts.assault };

    const units = [];
    for (const u of o.units || []) {
      if (!u.alive) continue;
      units.push(unitOrder(u, o, ctx));
    }
    return { units };
  },
};
