// Sokoban solver for make.fun: sokoban-solver-01-run-20261121.
// "Build the best Sokoban solver" — one program answers 5 instances in a set.
//
// How it works: every generated level is a hallway with 1-4 rooms hanging off
// it (one door each, 3-4 boxes per room), and a box pushed through a door can
// never come back, so a level's push-optimal count is the sum of its rooms'
// optima. The solver finds the rooms by flood fill from each door, then solves
// each room on its own with a push-based A* (state = player square + sorted
// box squares; cost = pushes), and stitches the room solutions together with
// player walks along the hallway. The A* heuristic is the exact min-cost
// assignment of boxes to goals over BFS distances, and states are pruned by
// dead-square analysis (reverse pull reachability from the goals) and 2x2
// freeze detection. Budgets are counted in expansions, never in wall-clock
// time, so two runs on the same set write the same bytes.
//
// Node 22, erasable TypeScript only, stdlib only, no clock reads, no random.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const LETTER = ['u', 'd', 'l', 'r'];

// Deterministic work budgets (expansions, not seconds): a room that needs more
// than this is given up on, so the whole set still finishes inside its wall.
const ROOM_BUDGET = 200_000;
const TOTAL_BUDGET = 1_200_000;
let spent = 0;
// Expansions the last pushAStar call used; a room that failed only because it
// hit its budget is retried once with a bigger one.
let lastExpansions = 0;

type Level = {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
};

type SearchResult = { moves: string; endPlayer: number; expansions: number };

// One step from cell c in direction d, or -1 when it leaves the board.
function step(c: number, d: number, w: number, h: number): number {
  const x = c % w;
  const y = (c / w) | 0;
  const nx = x + DX[d];
  const ny = y + DY[d];
  if (nx < 0 || nx >= w || ny < 0 || ny >= h) return -1;
  return ny * w + nx;
}

function parseLevel(text: string): Level {
  const clean = text.replace(/\r/g, '');
  const rows = clean.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = 0;
  rows.forEach((row, y) => {
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#';
      const i = y * w + x;
      if (ch !== '#') wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

// BFS distances from start over the cells where floor[c] is 1 (-1 = unreachable).
function bfsDist(floor: Uint8Array, w: number, h: number, start: number): Int32Array {
  const dist = new Int32Array(w * h).fill(-1);
  const q: number[] = [start];
  dist[start] = 0;
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi];
    for (let d = 0; d < 4; d++) {
      const n = step(c, d, w, h);
      if (n < 0 || !floor[n] || dist[n] !== -1) continue;
      dist[n] = dist[c] + 1;
      q.push(n);
    }
  }
  return dist;
}

// Player-walk region from start over floor cells, avoiding blocked cells.
// prev[c] is the previous cell on the walk (-1 at start, -2 unreached);
// prevDir[c] is the direction taken from prev[c] to c.
function bfsRegion(
  floor: Uint8Array,
  w: number,
  h: number,
  start: number,
  blocked: Uint8Array,
): { prev: Int32Array; prevDir: Int32Array } {
  const prev = new Int32Array(w * h).fill(-2);
  const prevDir = new Int32Array(w * h).fill(-1);
  const q: number[] = [start];
  prev[start] = -1;
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi];
    for (let d = 0; d < 4; d++) {
      const n = step(c, d, w, h);
      if (n < 0 || !floor[n] || blocked[n] || prev[n] !== -2) continue;
      prev[n] = c;
      prevDir[n] = d;
      q.push(n);
    }
  }
  return { prev, prevDir };
}

function walkPath(prev: Int32Array, prevDir: Int32Array, from: number, to: number): string {
  if (from === to) return '';
  let out = '';
  let c = to;
  while (c !== from) {
    out = LETTER[prevDir[c]] + out;
    c = prev[c];
  }
  return out;
}

// Walk from `from` to `to` over floor cells, treating blockedBoxes as walls;
// null when there is no walk.
function walkBetween(
  floor: Uint8Array,
  w: number,
  h: number,
  from: number,
  to: number,
  blockedBoxes: number[],
): string | null {
  const blocked = new Uint8Array(w * h);
  for (const b of blockedBoxes) blocked[b] = 1;
  const r = bfsRegion(floor, w, h, from, blocked);
  if (r.prev[to] === -2) return null;
  return walkPath(r.prev, r.prevDir, from, to);
}

// Cells a box can stand on and still reach a goal by pushes (reverse pull
// reachability from the goals, ignoring other boxes). A box off a goal on a
// cell outside this set can never be saved.
function liveCells(
  boxFloor: Uint8Array,
  walkFloor: Uint8Array,
  w: number,
  h: number,
  goals: number[],
): Uint8Array {
  const live = new Uint8Array(w * h);
  const q: number[] = [];
  for (const g of goals) {
    live[g] = 1;
    q.push(g);
  }
  for (let qi = 0; qi < q.length; qi++) {
    const x = q[qi];
    const xx = x % w;
    const xy = (x / w) | 0;
    for (let d = 0; d < 4; d++) {
      // A box at x could have been pushed here from p = x - d by a player
      // standing at x - 2d.
      const px = xx - DX[d];
      const py = xy - DY[d];
      if (px < 0 || px >= w || py < 0 || py >= h) continue;
      const p = py * w + px;
      if (!boxFloor[p] || live[p]) continue;
      const qx = px - DX[d];
      const qy = py - DY[d];
      if (qx < 0 || qx >= w || qy < 0 || qy >= h) continue;
      if (!walkFloor[qy * w + qx]) continue;
      live[p] = 1;
      q.push(p);
    }
  }
  return live;
}

// True when a 2x2 block of wall/box cells holds a box that is not on a goal:
// no box in such a block can ever move, so the position is dead.
function frozen2x2(
  boxFloor: Uint8Array,
  boxAt: Uint8Array,
  goalSet: Uint8Array,
  w: number,
  h: number,
): boolean {
  const occ = (i: number): boolean => !boxFloor[i] || boxAt[i] === 1;
  for (let y = 0; y + 1 < h; y++) {
    for (let x = 0; x + 1 < w; x++) {
      const a = y * w + x;
      const b = a + 1;
      const c = a + w;
      const d = a + w + 1;
      if (!occ(a) || !occ(b) || !occ(c) || !occ(d)) continue;
      for (const i of [a, b, c, d]) {
        if (boxAt[i] === 1 && goalSet[i] !== 1) return true;
      }
    }
  }
  return false;
}

// Lower bound on the pushes still needed: the cheapest assignment of boxes to
// goals over BFS distances (exact for up to 5 boxes; sum of per-box minima
// above that). Infinity when some box cannot reach any goal.
function heuristic(boxes: number[], goals: number[], distFromGoal: Int32Array[]): number {
  const n = boxes.length;
  if (n === 0) return 0;
  if (n <= 5) {
    let best = Infinity;
    const used = new Array<boolean>(n).fill(false);
    const rec = (i: number, sum: number): void => {
      if (sum >= best) return;
      if (i === n) {
        best = sum;
        return;
      }
      for (let j = 0; j < n; j++) {
        if (used[j]) continue;
        const d = distFromGoal[j][boxes[i]];
        if (d < 0) continue;
        used[j] = true;
        rec(i + 1, sum + d);
        used[j] = false;
      }
    };
    rec(0, 0);
    return best;
  }
  let s = 0;
  for (const b of boxes) {
    let m = Infinity;
    for (let j = 0; j < n; j++) {
      const d = distFromGoal[j][b];
      if (d >= 0 && d < m) m = d;
    }
    if (m === Infinity) return Infinity;
    s += m;
  }
  return s;
}

// Push-based A* over one area. walkFloor is where the player may stand,
// boxFloor is where a box may stand (boxes never leave boxFloor). The goal is
// every box on a goal; when exitCell >= 0 the player must also be able to
// walk out to exitCell from the final position. Returns null on failure or
// when the expansion budget runs out.
function pushAStar(
  walkFloor: Uint8Array,
  boxFloor: Uint8Array,
  w: number,
  h: number,
  boxes0: number[],
  goals: number[],
  startPlayer: number,
  exitCell: number,
  budget: number,
): SearchResult | null {
  const goalSet = new Uint8Array(w * h);
  for (const g of goals) goalSet[g] = 1;
  const distFromGoal = goals.map((g) => bfsDist(boxFloor, w, h, g));
  const live = liveCells(boxFloor, walkFloor, w, h, goals);
  const startBoxes = [...boxes0].sort((a, b) => a - b);
  for (const b of startBoxes) {
    if (goalSet[b] !== 1 && live[b] !== 1) return null;
  }
  const boxAt0 = new Uint8Array(w * h);
  for (const b of startBoxes) boxAt0[b] = 1;
  if (frozen2x2(boxFloor, boxAt0, goalSet, w, h)) return null;
  const h0 = heuristic(startBoxes, goals, distFromGoal);
  if (h0 === Infinity) return null;

  const pl: number[] = [startPlayer];
  const bx: number[][] = [startBoxes];
  const gg: number[] = [0];
  const ff: number[] = [h0];
  const parent: number[] = [-1];
  const trans: string[] = [''];
  const seq: number[] = [0];
  let seqCounter = 1;
  const key = (p: number, bs: number[]): string => p + '|' + bs.join(',');
  const best = new Map<string, number>();
  best.set(key(startPlayer, startBoxes), 0);
  const heap: number[] = [0];
  const less = (a: number, b: number): boolean =>
    ff[a] < ff[b] ||
    (ff[a] === ff[b] && (gg[a] > gg[b] || (gg[a] === gg[b] && seq[a] < seq[b])));
  const heapPush = (n: number): void => {
    let i = heap.length;
    heap.push(n);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      const t = heap[i];
      heap[i] = heap[p];
      heap[p] = t;
      i = p;
    }
  };
  const heapPop = (): number => {
    const top = heap[0];
    const last = heap.pop() as number;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1], heap[c])) c++;
        if (!less(heap[c], heap[i])) break;
        const t = heap[i];
        heap[i] = heap[c];
        heap[c] = t;
        i = c;
      }
    }
    return top;
  };
  const reconstruct = (s: number): string => {
    let out = '';
    for (let t = s; t > 0; t = parent[t]) out = trans[t] + out;
    return out;
  };

  const boxAt = new Uint8Array(w * h);
  let expansions = 0;
  lastExpansions = 0;
  while (heap.length > 0) {
    const s = heapPop();
    if (best.get(key(pl[s], bx[s])) !== s) continue;
    const bs = bx[s];
    const p = pl[s];
    let solved = true;
    for (const b of bs) {
      if (goalSet[b] !== 1) {
        solved = false;
        break;
      }
    }
    if (solved) {
      if (exitCell < 0) return { moves: reconstruct(s), endPlayer: p, expansions };
      for (const b of bs) boxAt[b] = 1;
      const r = bfsRegion(walkFloor, w, h, p, boxAt);
      for (const b of bs) boxAt[b] = 0;
      if (r.prev[exitCell] !== -2) {
        return { moves: reconstruct(s), endPlayer: p, expansions };
      }
      // Solved but the player cannot leave: keep searching for a way out.
    }
    if (expansions >= budget) {
      lastExpansions = expansions;
      return null;
    }
    expansions++;
    for (const b of bs) boxAt[b] = 1;
    const region = bfsRegion(walkFloor, w, h, p, boxAt);
    for (const b of bs) boxAt[b] = 0;
    // Generate every push, then insert them best-first: among equal-f nodes
    // the heap pops the earliest-inserted first, so ordering by the heuristic
    // steers the search toward promising pushes sooner.
    type Cand = { nb: number[]; b: number; from: number; h2: number; d: number };
    const cands: Cand[] = [];
    for (let bi = 0; bi < bs.length; bi++) {
      const b = bs[bi];
      const bx = b % w;
      const by = (b / w) | 0;
      for (let d = 0; d < 4; d++) {
        // Push direction d: the player stands at b - d, the box moves to b + d.
        const fx = bx - DX[d];
        const fy = by - DY[d];
        const tx = bx + DX[d];
        const ty = by + DY[d];
        if (fx < 0 || fx >= w || fy < 0 || fy >= h) continue;
        if (tx < 0 || tx >= w || ty < 0 || ty >= h) continue;
        const from = fy * w + fx;
        const to = ty * w + tx;
        if (!walkFloor[from] || !boxFloor[to]) continue;
        if (region.prev[from] === -2) continue;
        if (bs.includes(to)) continue;
        const nb = bs.slice();
        nb[bi] = to;
        nb.sort((a, b2) => a - b2);
        let dead = false;
        for (const x of nb) {
          if (goalSet[x] !== 1 && live[x] !== 1) {
            dead = true;
            break;
          }
        }
        if (dead) continue;
        for (const x of nb) boxAt[x] = 1;
        const fr = frozen2x2(boxFloor, boxAt, goalSet, w, h);
        for (const x of nb) boxAt[x] = 0;
        if (fr) continue;
        const h2 = heuristic(nb, goals, distFromGoal);
        if (h2 === Infinity) continue;
        cands.push({ nb, b, from, h2, d });
      }
    }
    cands.sort((a, b2) => a.h2 - b2.h2);
    for (const cand of cands) {
      const g2 = gg[s] + 1;
      const k = key(cand.b, cand.nb);
      const seen = best.get(k);
      if (seen !== undefined && gg[seen] <= g2) continue;
      const t = pl.length;
      pl.push(cand.b);
      bx.push(cand.nb);
      gg.push(g2);
      ff.push(g2 + cand.h2);
      parent.push(s);
      trans.push(walkPath(region.prev, region.prevDir, p, cand.from) + LETTER[cand.d]);
      seq.push(seqCounter++);
      best.set(k, t);
      heapPush(t);
    }
  }
  lastExpansions = expansions;
  return null;
}

type Room = {
  cells: Uint8Array;
  entry: number;
  boxes: number[];
  goals: number[];
};

// Split a level into its rooms: the hallway is the player's row; each room
// hangs off it through one door cell in the neighbouring row. Returns null
// when the level does not decompose cleanly (then the caller falls back to a
// whole-level search).
function findRooms(lv: Level): Room[] | null {
  const w = lv.w;
  const h = lv.h;
  const hallwayRow = (lv.player / w) | 0;
  const inHall = (c: number): boolean => ((c / w) | 0) === hallwayRow;
  const seen = new Uint8Array(w * h);
  const rooms: Room[] = [];
  for (let c = 0; c < w * h; c++) {
    if (lv.wall[c] || inHall(c) || seen[c]) continue;
    let entry = -1;
    for (let d = 0; d < 4; d++) {
      const n = step(c, d, w, h);
      if (n >= 0 && !lv.wall[n] && inHall(n)) {
        entry = n;
        break;
      }
    }
    if (entry < 0) continue;
    // Flood fill the room, never crossing the hallway row.
    const cells = new Uint8Array(w * h);
    const q: number[] = [c];
    cells[c] = 1;
    seen[c] = 1;
    for (let qi = 0; qi < q.length; qi++) {
      const x = q[qi];
      for (let d = 0; d < 4; d++) {
        const n = step(x, d, w, h);
        if (n < 0 || lv.wall[n] || inHall(n) || cells[n]) continue;
        cells[n] = 1;
        seen[n] = 1;
        q.push(n);
      }
    }
    const boxes = lv.boxes.filter((b) => cells[b] === 1);
    const goals = lv.goals.filter((g) => cells[g] === 1);
    rooms.push({ cells, entry, boxes, goals });
  }
  if (rooms.length === 0) return null;
  const covered = new Uint8Array(w * h);
  for (const r of rooms) {
    if (r.boxes.length === 0 || r.boxes.length !== r.goals.length) return null;
    for (const b of r.boxes) covered[b] = 1;
    for (const g of r.goals) covered[g] = 1;
  }
  for (const b of lv.boxes) if (!covered[b]) return null;
  for (const g of lv.goals) if (!covered[g]) return null;
  return rooms;
}

function solveRooms(lv: Level): string | null {
  const rooms = findRooms(lv);
  if (rooms === null) return null;
  const w = lv.w;
  const h = lv.h;
  const floor = new Uint8Array(w * h);
  for (let c = 0; c < w * h; c++) if (!lv.wall[c]) floor[c] = 1;
  let moves = '';
  let curPlayer = lv.player;
  const curBoxes = [...lv.boxes].sort((a, b) => a - b);
  for (let i = 0; i < rooms.length; i++) {
    const r = rooms[i];
    const last = i === rooms.length - 1;
    const walk = walkBetween(floor, w, h, curPlayer, r.entry, curBoxes);
    if (walk === null) return null;
    moves += walk;
    curPlayer = r.entry;
    const walkFloor = new Uint8Array(w * h);
    const boxFloor = new Uint8Array(w * h);
    for (let c = 0; c < w * h; c++) {
      if (r.cells[c] === 1) {
        walkFloor[c] = 1;
        boxFloor[c] = 1;
      }
    }
    walkFloor[r.entry] = 1;
    let res = pushAStar(
      walkFloor,
      boxFloor,
      w,
      h,
      r.boxes,
      r.goals,
      r.entry,
      last ? -1 : r.entry,
      ROOM_BUDGET,
    );
    // A room that failed only because it ran out of expansions gets one more
    // try with a bigger budget, while the set's total budget still allows it.
    // The failed attempt's work counts against the total either way.
    if (
      res === null &&
      lastExpansions >= ROOM_BUDGET &&
      spent + lastExpansions + ROOM_BUDGET * 3 <= TOTAL_BUDGET
    ) {
      spent += lastExpansions;
      res = pushAStar(
        walkFloor,
        boxFloor,
        w,
        h,
        r.boxes,
        r.goals,
        r.entry,
        last ? -1 : r.entry,
        ROOM_BUDGET * 3,
      );
    }
    if (res === null) {
      spent += lastExpansions;
      return null;
    }
    spent += res.expansions;
    if (spent > TOTAL_BUDGET) return null;
    moves += res.moves;
    curPlayer = res.endPlayer;
    if (!last) {
      const out = walkBetween(walkFloor, w, h, curPlayer, r.entry, r.goals);
      if (out === null) return null;
      moves += out;
      curPlayer = r.entry;
    }
    const roomBoxSet = new Set(r.boxes);
    const nextBoxes = curBoxes.filter((b) => !roomBoxSet.has(b));
    for (const g of r.goals) nextBoxes.push(g);
    nextBoxes.sort((a, b) => a - b);
    curBoxes.length = 0;
    curBoxes.push(...nextBoxes);
  }
  return moves;
}

function solve(text: string): string {
  try {
    const lv = parseLevel(text);
    const rooms = solveRooms(lv);
    if (rooms !== null) return rooms;
    const floor = new Uint8Array(lv.w * lv.h);
    for (let c = 0; c < lv.w * lv.h; c++) if (!lv.wall[c]) floor[c] = 1;
    const res = pushAStar(
      floor,
      floor,
      lv.w,
      lv.h,
      lv.boxes,
      lv.goals,
      lv.player,
      -1,
      Math.max(0, TOTAL_BUDGET - spent),
    );
    if (res === null) return '';
    spent += res.expansions;
    return res.moves;
  } catch {
    return '';
  }
}

for (const id of readdirSync(I).sort()) {
  const moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
