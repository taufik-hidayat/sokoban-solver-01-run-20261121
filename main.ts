'use strict';
/*
 * Sokoban solver for make.fun sokoban-solver-01-run-20261121.
 *
 * Idea (from the competition rules): every level is a 1-cell-wide hallway
 * with 1-4 rooms hanging off it, 3-4 boxes per room, and a box pushed out
 * of its room can never come back.  So a level's push-optimal count is the
 * sum of its rooms' optima, and each room can be solved on its own from its
 * door, then the rooms are stitched together with plain walks along the
 * hallway.
 *
 * Per room we run A* over *pushes* (player position canonicalised to its
 * reachable region), with:
 *   - heuristic: minimum-cost assignment of boxes to goals using
 *     box-ignoring BFS distances (admissible, consistent),
 *   - dead-cell pruning: cells from which no goal is reachable in the
 *     "pull graph" (a box may move c -> m only if the cell beyond c
 *     opposite m is floor),
 *   - 2x2 freeze pruning: a 2x2 block of walls/boxes holding a box that
 *     is not on a goal can never move again,
 *   - an exit-constrained variant: the final state must leave the door
 *     free and reachable, so the player can leave the room.  The room
 *     whose exit costs the most extra pushes is solved last (it needs no
 *     exit), every other room uses its exit-constrained optimum.
 *
 * Work is budgeted by node expansions, never by the wall clock, so two
 * runs are byte-identical.  No randomness anywhere.
 */

const fs = require('node:fs');
const path = require('node:path');

const DIRS = [
  { dx: 0, dy: -1, ch: 'u' },
  { dx: 0, dy: 1, ch: 'd' },
  { dx: -1, dy: 0, ch: 'l' },
  { dx: 1, dy: 0, ch: 'r' },
];
const OPP = [1, 0, 3, 2];

// Work budgets (node expansions / stored nodes).  These bound the total
// work, not the clock, so output is deterministic.
const ASTAR_EXPAND_CAP = 1000000;
const ASTAR_NODE_CAP = 1500000;
const GREEDY_EXPAND_CAP = 1000000;
const GREEDY_NODE_CAP = 1500000;

// ---------------------------------------------------------------- heap ---
// Min-heap ordered by (f, h, seq); seq makes every tie-break deterministic.
function heapCmp(a, b) {
  if (a.f !== b.f) return a.f - b.f;
  if (a.h !== b.h) return a.h - b.h;
  return a.seq - b.seq;
}
class Heap {
  constructor() { this.a = []; }
  get size() { return this.a.length; }
  push(item) {
    const a = this.a;
    a.push(item);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (heapCmp(a[i], a[p]) < 0) {
        const t = a[i]; a[i] = a[p]; a[p] = t; i = p;
      } else break;
    }
  }
  pop() {
    const a = this.a;
    const top = a[0];
    const last = a.pop();
    if (a.length > 0) {
      a[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1;
        let m = i;
        if (l < a.length && heapCmp(a[l], a[m]) < 0) m = l;
        if (r < a.length && heapCmp(a[r], a[m]) < 0) m = r;
        if (m === i) break;
        const t = a[i]; a[i] = a[m]; a[m] = t; i = m;
      }
    }
    return top;
  }
}

// ------------------------------------------------------------ BFS helpers --
function dirCharFromTo(a, b, W) {
  const ax = a % W, ay = (a / W) | 0;
  const bx = b % W, by = (b / W) | 0;
  if (bx === ax && by === ay - 1) return 'u';
  if (bx === ax && by === ay + 1) return 'd';
  if (bx === ax - 1 && by === ay) return 'l';
  return 'r';
}

// Shortest walk (list of move chars) from `from` to `to` over floor cells,
// never stepping on a cell in `blocked` (a Set of global cell indices).
// Deterministic: fixed neighbour order.  Returns null when unreachable.
function globalPath(grid, from, to, blocked) {
  const W = grid.W, H = grid.H;
  if (from === to) return [];
  const prev = new Int32Array(W * H).fill(-2);
  const prevDir = new Int8Array(W * H);
  const q = [from];
  prev[from] = -1;
  let head = 0;
  while (head < q.length) {
    const c = q[head++];
    const cx = c % W, cy = (c / W) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DIRS[d].dx, ny = cy + DIRS[d].dy;
      if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
      const m = ny * W + nx;
      if (!grid.floor[m] || blocked.has(m) || prev[m] !== -2) continue;
      prev[m] = c;
      prevDir[m] = d;
      if (m === to) {
        const moves = [];
        let cur = m;
        while (cur !== from) {
          moves.push(DIRS[prevDir[cur]].ch);
          cur = prev[cur];
        }
        moves.reverse();
        return moves;
      }
      q.push(m);
    }
  }
  return null;
}

// ------------------------------------------------------------ level parse --
function parseLevel(text) {
  const lines = text.split(/\r?\n/).filter((l) => l.length > 0);
  if (lines.length === 0) return null;
  const H = lines.length;
  const W = Math.max.apply(null, lines.map((l) => l.length));
  const floor = new Uint8Array(W * H);
  const isGoal = new Uint8Array(W * H);
  const boxes = [];
  let player = -1;
  for (let y = 0; y < H; y++) {
    const row = lines[y];
    for (let x = 0; x < W; x++) {
      const ch = x < row.length ? row[x] : '#';
      const c = y * W + x;
      if (ch !== '#') floor[c] = 1;
      if (ch === '.' || ch === '*' || ch === '+') isGoal[c] = 1;
      if (ch === '$' || ch === '*') boxes.push(c);
      if (ch === '@' || ch === '+') player = c;
    }
  }
  if (player < 0) return null;
  const hy = (player / W) | 0;
  return { W, H, floor, isGoal, boxes, player, hy };
}

// Split the level into rooms: connected components of floor cells outside
// the hallway row.  A room's doors are its cells adjacent to the hallway.
function findRooms(grid) {
  const { W, H, floor, isGoal, boxes, hy } = grid;
  const comp = new Int32Array(W * H).fill(-1);
  const rooms = [];
  for (let c = 0; c < W * H; c++) {
    const y = (c / W) | 0;
    if (!floor[c] || y === hy || comp[c] !== -1) continue;
    const cells = [];
    const q = [c];
    comp[c] = rooms.length;
    let head = 0;
    while (head < q.length) {
      const cur = q[head++];
      cells.push(cur);
      const cx = cur % W, cy = (cur / W) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = cx + DIRS[d].dx, ny = cy + DIRS[d].dy;
        if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
        const m = ny * W + nx;
        if (!floor[m] || ny === hy || comp[m] !== -1) continue;
        comp[m] = rooms.length;
        q.push(m);
      }
    }
    const rboxes = boxes.filter((b) => comp[b] === rooms.length);
    if (rboxes.length === 0) continue;
    const rgoals = cells.filter((cell) => isGoal[cell]);
    const doors = [];
    const doorSeen = new Set();
    for (const cell of cells) {
      const x = cell % W, y = (cell / W) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = x + DIRS[d].dx, ny = y + DIRS[d].dy;
        if (ny === hy && nx >= 0 && nx < W && floor[ny * W + nx] && !doorSeen.has(cell)) {
          doorSeen.add(cell);
          doors.push({ door: cell, entry: ny * W + nx });
        }
      }
    }
    if (doors.length === 0) return null;
    rooms.push({ cells, boxes: rboxes, goals: rgoals, doors });
  }
  return rooms;
}

// ------------------------------------------------------------ room context -
// Local indices 0..n-1 for the room's walkable cells (its floor plus the
// hallway cells its doors open onto).  All per-room tables are built once
// and shared by the unconstrained and the exit-constrained searches.
function buildRoomCtx(grid, room, startPlayerGlobal) {
  const { W, H, floor, isGoal, hy } = grid;
  const cells = [];
  const seen = new Set();
  for (const c of room.cells) {
    if (!seen.has(c)) { seen.add(c); cells.push(c); }
  }
  for (const dr of room.doors) {
    if (!seen.has(dr.entry)) { seen.add(dr.entry); cells.push(dr.entry); }
  }
  const n = cells.length;
  const locOf = new Int32Array(W * H).fill(-1);
  cells.forEach((c, i) => { locOf[c] = i; });
  const nbr = new Int16Array(n * 4);
  const isEntry = new Uint8Array(n);
  for (let i = 0; i < n; i++) {
    const c = cells[i];
    const x = c % W, y = (c / W) | 0;
    if (y === hy) isEntry[i] = 1;
    for (let d = 0; d < 4; d++) {
      const nx = x + DIRS[d].dx, ny = y + DIRS[d].dy;
      if (nx < 0 || ny < 0 || nx >= W || ny >= H) { nbr[i * 4 + d] = -1; continue; }
      nbr[i * 4 + d] = locOf[ny * W + nx];
    }
  }
  const boxes0 = room.boxes.map((c) => locOf[c]).sort((a, b) => a - b);
  const goals = room.goals.map((c) => locOf[c]).sort((a, b) => a - b);
  const k = boxes0.length;
  const isGoalLocal = new Uint8Array(n);
  for (const g of goals) isGoalLocal[g] = 1;

  // BFS distances from each goal over walkable cells, ignoring boxes.
  const dist = [];
  for (let gi = 0; gi < k; gi++) {
    const d = new Int16Array(n).fill(-1);
    const q = [goals[gi]];
    d[goals[gi]] = 0;
    let head = 0;
    while (head < q.length) {
      const c = q[head++];
      for (let dir = 0; dir < 4; dir++) {
        const m = nbr[c * 4 + dir];
        if (m < 0 || d[m] !== -1) continue;
        d[m] = d[c] + 1;
        q.push(m);
      }
    }
    dist.push(d);
  }

  // Dead cells: a box can be pushed c -> m only when the cell on the far
  // side of c is floor.  Reverse-reachability from the goals in that
  // "pull graph" marks every cell a box could ever reach a goal from.
  const rev = [];
  for (let i = 0; i < n; i++) rev.push([]);
  for (let c = 0; c < n; c++) {
    for (let d = 0; d < 4; d++) {
      const m = nbr[c * 4 + d];
      if (m >= 0 && nbr[c * 4 + OPP[d]] >= 0) rev[m].push(c);
    }
  }
  const live = new Uint8Array(n);
  const lq = [];
  for (const g of goals) {
    if (!live[g]) { live[g] = 1; lq.push(g); }
  }
  {
    let head = 0;
    while (head < lq.length) {
      const c = lq[head++];
      for (const p of rev[c]) {
        if (!live[p]) { live[p] = 1; lq.push(p); }
      }
    }
  }
  const dead = new Uint8Array(n);
  for (let i = 0; i < n; i++) dead[i] = live[i] ? 0 : 1;

  // 2x2 blocks each local cell takes part in (global cell indices).
  const blocksOf = [];
  for (let i = 0; i < n; i++) {
    const c = cells[i];
    const x = c % W, y = (c / W) | 0;
    const list = [];
    const corners = [[x - 1, y - 1], [x, y - 1], [x - 1, y], [x, y]];
    for (const [bx, by] of corners) {
      if (bx < 0 || by < 0 || bx >= W - 1 || by >= H - 1) continue;
      list.push([by * W + bx, by * W + bx + 1, (by + 1) * W + bx, (by + 1) * W + bx + 1]);
    }
    blocksOf.push(list);
  }

  // All permutations of the goal indices, for the assignment heuristic.
  const perms = [];
  (function permute(arr, i) {
    if (i === arr.length) { perms.push(arr.slice()); return; }
    for (let j = i; j < arr.length; j++) {
      const t = arr[i]; arr[i] = arr[j]; arr[j] = t;
      permute(arr, i + 1);
      const t2 = arr[i]; arr[i] = arr[j]; arr[j] = t2;
    }
  })(goals.map((_, i) => i), 0);

  // Encoding base: smallest power of two >= n, so a sorted box tuple and
  // a player-region representative pack into one exact integer key.
  let base = 1;
  while (base < n) base *= 2;

  const doors = room.doors.map((dr) => ({
    doorLocal: locOf[dr.door],
    entryLocal: locOf[dr.entry],
    doorGlobal: dr.door,
    entryGlobal: dr.entry,
  }));
  const startPlayer = locOf[startPlayerGlobal];

  return {
    W, H, floor, isGoal, hy, cells, locOf, n, nbr, isEntry,
    boxes0, goals, k, isGoalLocal, dist, dead, blocksOf, perms,
    base, doors, startPlayer,
  };
}

// Cells reachable by the player from `player` (local) without moving boxes.
function regionFrom(ctx, player, boxSet) {
  const seen = new Uint8Array(ctx.n);
  const q = [player];
  seen[player] = 1;
  let head = 0;
  let rep = player;
  while (head < q.length) {
    const c = q[head++];
    if (c < rep) rep = c;
    for (let d = 0; d < 4; d++) {
      const m = ctx.nbr[c * 4 + d];
      if (m < 0 || seen[m] || boxSet.has(m)) continue;
      seen[m] = 1;
      q.push(m);
    }
  }
  return { cells: q, rep };
}

// Shortest walk inside the room from `from` to `to` (local indices),
// never stepping on a box.  Returns move chars or null.
function localPath(ctx, from, to, boxSet) {
  if (from === to) return [];
  const prev = new Int32Array(ctx.n).fill(-2);
  const prevDir = new Int8Array(ctx.n);
  const q = [from];
  prev[from] = -1;
  let head = 0;
  while (head < q.length) {
    const c = q[head++];
    for (let d = 0; d < 4; d++) {
      const m = ctx.nbr[c * 4 + d];
      if (m < 0 || prev[m] !== -2 || boxSet.has(m)) continue;
      prev[m] = c;
      prevDir[m] = d;
      if (m === to) {
        const moves = [];
        let cur = m;
        while (cur !== from) {
          moves.push(DIRS[prevDir[cur]].ch);
          cur = prev[cur];
        }
        moves.reverse();
        return moves;
      }
      q.push(m);
    }
  }
  return null;
}

// True when the box that just moved onto `movedLocal` is caught in a 2x2
// block of walls/boxes while not sitting on a goal.
function frozen(ctx, boxesLocal, movedLocal) {
  const boxSet = new Set(boxesLocal);
  for (const block of ctx.blocksOf[movedLocal]) {
    let allBlocked = true;
    let nonGoalBox = false;
    for (let i = 0; i < 4; i++) {
      const cell = block[i];
      if (!ctx.floor[cell]) continue; // wall: blocked
      const li = ctx.locOf[cell];
      if (li >= 0) {
        if (boxSet.has(li)) {
          if (!ctx.isGoalLocal[li]) nonGoalBox = true;
          continue; // box: blocked
        }
        allBlocked = false; break; // open floor in the room
      }
      allBlocked = false; break; // open floor outside the room
    }
    if (allBlocked && nonGoalBox) return true;
  }
  return false;
}

// Push-optimal (A*) or greedy best-first search of one room.
// requireExit: the goal state must also let the player reach a free door.
// Returns {moves, pushes, endPlayer, exitDoor} or null.
function searchRoom(ctx, requireExit, greedy, maxExpand, maxNodes) {
  const n = ctx.n, k = ctx.k;
  const base = ctx.base;
  const perms = ctx.perms, dist = ctx.dist;

  function boxCode(boxes) {
    let code = 0;
    for (let i = 0; i < k; i++) code = code * base + boxes[i];
    return code;
  }
  const hCache = new Map();
  function heuristic(boxes) {
    const code = boxCode(boxes);
    let h = hCache.get(code);
    if (h !== undefined) return h;
    h = Infinity;
    for (let p = 0; p < perms.length; p++) {
      const perm = perms[p];
      let s = 0;
      let ok = true;
      for (let i = 0; i < k; i++) {
        const d = dist[perm[i]][boxes[i]];
        if (d < 0) { ok = false; break; }
        s += d;
      }
      if (ok && s < h) h = s;
    }
    hCache.set(code, h);
    return h;
  }
  function isSolved(boxes) {
    for (let i = 0; i < k; i++) if (!ctx.isGoalLocal[boxes[i]]) return false;
    return true;
  }
  // Which door (if any) can the player reach from `player` with `boxes`
  // fixed, with that door cell free?  Returns the door object or null.
  function exitDoor(boxes, player) {
    const boxSet = new Set(boxes);
    const reg = regionFrom(ctx, player, boxSet);
    const inReg = new Set(reg.cells);
    for (const dr of ctx.doors) {
      if (!boxSet.has(dr.doorLocal) && inReg.has(dr.doorLocal)) return dr;
    }
    return null;
  }

  const startBoxes = ctx.boxes0.slice();
  const h0 = heuristic(startBoxes);
  if (h0 === Infinity) return null;
  const startReg = regionFrom(ctx, ctx.startPlayer, new Set(startBoxes));
  const startKey = boxCode(startBoxes) * base + startReg.rep;
  const nodes = new Map();
  nodes.set(startKey, {
    boxes: startBoxes, player: ctx.startPlayer, g: 0, h: h0,
    parent: null, pushBox: -1, pushDir: -1, closed: false,
  });
  const heap = new Heap();
  let seq = 0;
  heap.push({ f: h0, h: h0, seq: seq++, key: startKey, g: 0 });

  function finish(node, door) {
    const pushes = [];
    let cur = node;
    while (cur.parent !== null) {
      pushes.push({ box: cur.pushBox, dir: cur.pushDir });
      cur = nodes.get(cur.parent);
    }
    pushes.reverse();
    let player = ctx.startPlayer;
    let boxes = ctx.boxes0.slice();
    const moves = [];
    for (const p of pushes) {
      const pusher = ctx.nbr[p.box * 4 + OPP[p.dir]];
      const walk = localPath(ctx, player, pusher, new Set(boxes));
      if (walk === null) return null;
      for (const ch of walk) moves.push(ch);
      moves.push(DIRS[p.dir].ch);
      const bi = boxes.indexOf(p.box);
      boxes[bi] = ctx.nbr[p.box * 4 + p.dir];
      boxes.sort((a, b) => a - b);
      player = p.box;
    }
    return {
      moves: moves.join(''),
      pushes: pushes.length,
      endPlayer: ctx.cells[player],
      exitDoor: door || null,
    };
  }

  let expansions = 0;
  while (heap.size > 0) {
    const e = heap.pop();
    const node = nodes.get(e.key);
    if (!node || node.closed || node.g !== e.g) continue;
    node.closed = true;
    if (isSolved(node.boxes)) {
      if (!requireExit) {
        const done = finish(node, null);
        if (done) return done;
        return null;
      }
      const dr = exitDoor(node.boxes, node.player);
      if (dr) {
        const done = finish(node, dr);
        if (done) return done;
        return null;
      }
      // Solved but the player cannot leave: keep searching through this
      // state (a box may have to step off its goal and back so the
      // player can get out).  Do not return, do not skip expansion.
    }
    if (expansions >= maxExpand) return null;
    if (nodes.size > maxNodes) return null;
    expansions++;
    const boxSet = new Set(node.boxes);
    const reg = regionFrom(ctx, node.player, boxSet);
    const inReg = new Set(reg.cells);
    for (let bi = 0; bi < k; bi++) {
      const b = node.boxes[bi];
      for (let d = 0; d < 4; d++) {
        const t = ctx.nbr[b * 4 + d];
        if (t < 0 || boxSet.has(t) || ctx.isEntry[t]) continue;
        const pusher = ctx.nbr[b * 4 + OPP[d]];
        if (pusher < 0 || !inReg.has(pusher)) continue;
        if (ctx.dead[t]) continue;
        const nb = node.boxes.slice();
        nb[bi] = t;
        nb.sort((a, b2) => a - b2);
        if (frozen(ctx, nb, t)) continue;
        const h = heuristic(nb);
        if (h === Infinity) continue;
        const nreg = regionFrom(ctx, b, new Set(nb));
        const key = boxCode(nb) * base + nreg.rep;
        const g2 = node.g + 1;
        const ex = nodes.get(key);
        if (ex) {
          if (ex.closed || ex.g <= g2) continue;
          ex.g = g2;
          ex.parent = e.key;
          ex.pushBox = b;
          ex.pushDir = d;
          ex.boxes = nb;
          ex.player = b;
          ex.h = h;
        } else {
          nodes.set(key, {
            boxes: nb, player: b, g: g2, h,
            parent: e.key, pushBox: b, pushDir: d, closed: false,
          });
        }
        heap.push({ f: greedy ? h : g2 + h, h, seq: seq++, key, g: g2 });
      }
    }
  }
  return null;
}

// Does an already-found room solution let the player leave through a door?
// (Final boxes are the goals; the door must not be a goal and must be
// reachable from the player's final cell.)
function solutionExits(ctx, sol) {
  const playerLocal = ctx.locOf[sol.endPlayer];
  if (playerLocal < 0) return null;
  const boxSet = new Set(ctx.goals);
  const reg = regionFrom(ctx, playerLocal, boxSet);
  const inReg = new Set(reg.cells);
  for (const dr of ctx.doors) {
    if (!ctx.isGoalLocal[dr.doorLocal] && inReg.has(dr.doorLocal)) return dr;
  }
  return null;
}

// Solve one room, returning {U, C, ctx} where U is the unconstrained
// optimum and C the exit-constrained optimum (C === U when the unconstrained
// solution already lets the player out).
function solveOneRoom(grid, room) {
  const ctx = buildRoomCtx(grid, room, room.doors[0].entry);
  let U = searchRoom(ctx, false, false, ASTAR_EXPAND_CAP, ASTAR_NODE_CAP);
  if (!U) U = searchRoom(ctx, false, true, GREEDY_EXPAND_CAP, GREEDY_NODE_CAP);
  if (!U) return null;
  const dr = solutionExits(ctx, U);
  if (dr) {
    U.exitDoor = dr;
    return { room, ctx, U, C: U };
  }
  let C = searchRoom(ctx, true, false, ASTAR_EXPAND_CAP, ASTAR_NODE_CAP);
  if (!C) C = searchRoom(ctx, true, true, GREEDY_EXPAND_CAP, GREEDY_NODE_CAP);
  return { room, ctx, U, C };
}

// Fallback for levels that do not decompose into rooms (a box sitting in
// the hallway, or a room whose box/goal counts differ): treat the whole
// board as one room and try the same search machinery on it.
function solveGlobalFallback(grid) {
  const cells = [];
  for (let c = 0; c < grid.W * grid.H; c++) if (grid.floor[c]) cells.push(c);
  const room = {
    cells,
    boxes: grid.boxes.slice(),
    goals: cells.filter((c) => grid.isGoal[c]),
    doors: [],
  };
  if (room.boxes.length !== room.goals.length) return '';
  const ctx = buildRoomCtx(grid, room, grid.player);
  ctx.doors = [];
  let sol = searchRoom(ctx, false, false, ASTAR_EXPAND_CAP, ASTAR_NODE_CAP);
  if (!sol) sol = searchRoom(ctx, false, true, GREEDY_EXPAND_CAP, GREEDY_NODE_CAP);
  return sol ? sol.moves : '';
}

// ------------------------------------------------------------- whole level --
function solveLevel(text) {
  const grid = parseLevel(text);
  if (!grid) return '';
  const rooms = findRooms(grid);
  if (!rooms || rooms.length === 0) return solveGlobalFallback(grid);
  for (const b of grid.boxes) {
    if (((b / grid.W) | 0) === grid.hy) return solveGlobalFallback(grid);
  }
  for (const r of rooms) {
    if (r.boxes.length !== r.goals.length) return solveGlobalFallback(grid);
  }

  const solved = [];
  for (const room of rooms) {
    const s = solveOneRoom(grid, room);
    if (!s) return '';
    solved.push(s);
  }

  // The room whose exit costs the most extra pushes goes last (it needs
  // no exit); every other room uses its exit-constrained optimum.
  let lastIdx = 0;
  let lastDelta = -Infinity;
  for (let i = 0; i < solved.length; i++) {
    const s = solved[i];
    const delta = s.C ? s.C.pushes - s.U.pushes : Infinity;
    if (delta > lastDelta || (delta === lastDelta && i > lastIdx)) {
      lastDelta = delta;
      lastIdx = i;
    }
  }
  const order = [];
  for (let i = 0; i < solved.length; i++) if (i !== lastIdx) order.push(i);
  order.push(lastIdx);

  let moves = '';
  let player = grid.player;
  const curBoxes = new Set(grid.boxes);
  for (let oi = 0; oi < order.length; oi++) {
    const s = solved[order[oi]];
    const isLast = oi === order.length - 1;
    const sol = isLast ? s.U : (s.C || s.U);
    const entryGlobal = s.room.doors[0].entry;
    const walkIn = globalPath(grid, player, entryGlobal, curBoxes);
    if (walkIn === null) return '';
    moves += walkIn.join('');
    moves += sol.moves;
    player = sol.endPlayer;
    for (const b of s.room.boxes) curBoxes.delete(b);
    for (const g of s.room.goals) curBoxes.add(g);
    if (!isLast) {
      const dr = sol.exitDoor;
      if (!dr) return '';
      const walkOut = globalPath(grid, player, dr.doorGlobal, curBoxes);
      if (walkOut === null) return '';
      moves += walkOut.join('');
      moves += dirCharFromTo(dr.doorGlobal, dr.entryGlobal, grid.W);
      player = dr.entryGlobal;
    }
  }
  return moves;
}

// ------------------------------------------------------------------- main --
function main() {
  const inputDir = process.env.MAKEFUN_INPUT_DIR || '/input';
  const outputDir = process.env.MAKEFUN_OUTPUT_DIR || '/output';
  const ids = fs.readdirSync(inputDir)
    .filter((d) => {
      try { return fs.statSync(path.join(inputDir, d)).isDirectory(); }
      catch (e) { return false; }
    })
    .sort();
  for (const id of ids) {
    const levelPath = path.join(inputDir, id, 'level.txt');
    let moves = '';
    try {
      const text = fs.readFileSync(levelPath, 'utf8');
      moves = solveLevel(text);
    } catch (e) {
      moves = '';
    }
    const outDir = path.join(outputDir, id);
    fs.mkdirSync(outDir, { recursive: true });
    fs.writeFileSync(path.join(outDir, 'solution.txt'), moves);
  }
}

main();
