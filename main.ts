// Sokoban solver — push-optimal per room, stitched through the hallway.
//
// Strategy: a generated level is a 1-wide hallway (player start row) plus
// 1-4 rooms, each opening onto the hallway through exactly one door. Boxes
// can never leave their room in an optimal solution (a box pushed into the
// hallway can never be pushed back), so the level's push-optimum is the sum
// of the rooms' optima. Each room is solved independently with A* over push
// states (boxes + canonical player region), Manhattan-assignment heuristic,
// dead-square + corner pruning, then the per-room move strings are stitched
// with shortest player walks along the hallway. A whole-level A* is kept as
// a fallback for levels whose structure does not decompose. Everything is
// deterministic: fixed iteration order, heap tie-break by insertion counter,
// node-count budgets only (the wall clock is never read).
import * as fs from 'node:fs';
import * as path from 'node:path';

// ---------------------------------------------------------------- constants
const DIRS = [
  { dr: -1, dc: 0, ch: 'u' },
  { dr: 1, dc: 0, ch: 'd' },
  { dr: 0, dc: -1, ch: 'l' },
  { dr: 0, dc: 1, ch: 'r' },
];
// Total push-expansion budget across the whole set (deterministic cutoff).
const TOTAL_BUDGET = 4000000;
let budgetLeft = TOTAL_BUDGET;

// ------------------------------------------------------------------- heap
// Minimal binary heap keyed by (f, insertion counter).
class Heap {
  constructor() {
    this.a = [];
  }
  get size() {
    return this.a.length;
  }
  push(item) {
    const a = this.a;
    a.push(item);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (cmp(a[p], a[i]) <= 0) break;
      [a[p], a[i]] = [a[i], a[p]];
      i = p;
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
        const l = 2 * i + 1;
        const r = l + 1;
        let m = i;
        if (l < a.length && cmp(a[l], a[m]) < 0) m = l;
        if (r < a.length && cmp(a[r], a[m]) < 0) m = r;
        if (m === i) break;
        [a[m], a[i]] = [a[i], a[m]];
        i = m;
      }
    }
    return top;
  }
}
function cmp(x, y) {
  if (x.f !== y.f) return x.f - y.f;
  return x.seq - y.seq;
}

// ------------------------------------------------------------------ level
function parseLevel(text) {
  const lines = text.replace(/\r/g, '').split('\n');
  while (lines.length && lines[lines.length - 1] === '') lines.pop();
  const H = lines.length;
  const W = Math.max(...lines.map((l) => l.length));
  const grid = lines.map((l) => l.padEnd(W, '#').split(''));
  const walls = new Set();
  const goals = new Set();
  const boxes = [];
  let player = -1;
  for (let r = 0; r < H; r++) {
    for (let c = 0; c < W; c++) {
      const ch = grid[r][c];
      const idx = r * W + c;
      if (ch === '#') walls.add(idx);
      if (ch === '.' || ch === '*' || ch === '+') goals.add(idx);
      if (ch === '$' || ch === '*') boxes.push(idx);
      if (ch === '@' || ch === '+') player = idx;
    }
  }
  boxes.sort((a, b) => a - b);
  return { H, W, walls, goals, boxes, player, grid };
}

function rc(idx, W) {
  return [Math.floor(idx / W), idx % W];
}

// BFS walk avoiding walls and boxes; returns the move string or null.
function walkPath(start, target, passable, boxSet) {
  if (start === target) return '';
  const prev = new Map();
  const q = [start];
  prev.set(start, -1);
  let qi = 0;
  while (qi < q.length) {
    const cur = q[qi++];
    const [r, c] = rc(cur, passable.W);
    for (const d of DIRS) {
      const nr = r + d.dr;
      const nc = c + d.dc;
      if (nr < 0 || nr >= passable.H || nc < 0 || nc >= passable.W) continue;
      const nidx = nr * passable.W + nc;
      if (!passable.cells.has(nidx)) continue;
      if (boxSet.has(nidx)) continue;
      if (prev.has(nidx)) continue;
      prev.set(nidx, cur * 4 + dirIndex(d));
      if (nidx === target) {
        let s = '';
        let node = nidx;
        while (node !== start) {
          const code = prev.get(node);
          const pd = DIRS[code % 4];
          s = pd.ch + s;
          node = Math.floor(code / 4);
        }
        return s;
      }
      q.push(nidx);
    }
  }
  return null;
}
function dirIndex(d) {
  return DIRS.indexOf(d);
}

// Reachable floor cells for the player given box positions.
function reachable(start, passable, boxSet) {
  const seen = new Set([start]);
  const q = [start];
  let qi = 0;
  while (qi < q.length) {
    const cur = q[qi++];
    const [r, c] = rc(cur, passable.W);
    for (const d of DIRS) {
      const nr = r + d.dr;
      const nc = c + d.dc;
      if (nr < 0 || nr >= passable.H || nc < 0 || nc >= passable.W) continue;
      const nidx = nr * passable.W + nc;
      if (!passable.cells.has(nidx)) continue;
      if (boxSet.has(nidx)) continue;
      if (seen.has(nidx)) continue;
      seen.add(nidx);
      q.push(nidx);
    }
  }
  return seen;
}

// ------------------------------------------------------------------ A*
// Push-optimal A* over one box set inside one floor region.
//   floorCells : cells the player may stand on
//   boxCells   : cells a box may occupy (subset of floorCells)
//   goals      : goal cells (subset of boxCells)
//   dead       : precomputed dead box cells (non-goal, never useful)
//   requireExit: final state must leave `exitCell` box-free and reachable
// Returns the move string, or null.
function solvePush(floorCells, boxCells, goals, boxes, player, exitCell, requireExit, W, H) {
  const goalArr = [...goals];
  const boxSet0 = new Set(boxes);
  // Dead cells: reverse flood from goals through boxCells (walls only block).
  const live = new Set(goalArr);
  const q = [...goalArr];
  let qi = 0;
  while (qi < q.length) {
    const cur = q[qi++];
    const [r, c] = rc(cur, W);
    for (const d of DIRS) {
      const nr = r + d.dr;
      const nc = c + d.dc;
      if (nr < 0 || nr >= H || nc < 0 || nc >= W) continue;
      const nidx = nr * W + nc;
      if (!boxCells.has(nidx) || live.has(nidx)) continue;
      live.add(nidx);
      q.push(nidx);
    }
  }
  const dead = new Set();
  for (const cell of boxCells) {
    if (!live.has(cell)) dead.add(cell);
  }
  // Corner dead cells: two perpendicular blocked neighbours, not a goal.
  const blocked = (idx) => !boxCells.has(idx);
  for (const cell of boxCells) {
    if (goals.has(cell) || dead.has(cell)) continue;
    const [r, c] = rc(cell, W);
    const up = r - 1 < 0 || blocked((r - 1) * W + c);
    const dn = r + 1 >= H || blocked((r + 1) * W + c);
    const lf = c - 1 < 0 || blocked(r * W + c - 1);
    const rt = c + 1 >= W || blocked(r * W + c + 1);
    if ((up || dn) && (lf || rt)) dead.add(cell);
  }

  const passable = { cells: floorCells, W, H };
  const startReach = reachable(player, passable, boxSet0);
  const canon = (reach) => {
    let m = Infinity;
    for (const cell of reach) if (cell < m) m = cell;
    return m;
  };
  const keyOf = (bx, pl) => bx.join(',') + '|' + pl;

  // Min-cost assignment (Manhattan) of boxes to goals. Exact by permutation
  // for small sets; for larger sets (whole-level fallback) the sum of each
  // box's distance to its nearest goal, which stays an admissible bound.
  const heur = (bx) => {
    const n = bx.length;
    if (n === 0) return 0;
    const dist = bx.map((b) => {
      const [br, bc] = rc(b, W);
      return goalArr.map((g) => {
        const [gr, gc] = rc(g, W);
        return Math.abs(br - gr) + Math.abs(bc - gc);
      });
    });
    if (n > 6) {
      let sum = 0;
      for (let i = 0; i < n; i++) sum += Math.min(...dist[i]);
      return sum;
    }
    let best = Infinity;
    const used = new Array(n).fill(false);
    const assign = (i, sum) => {
      if (sum >= best) return;
      if (i === n) {
        best = sum;
        return;
      }
      for (let j = 0; j < n; j++) {
        if (used[j]) continue;
        used[j] = true;
        assign(i + 1, sum + dist[i][j]);
        used[j] = false;
      }
    };
    assign(0, 0);
    return best;
  };

  const isGoalState = (bx) => bx.every((b) => goals.has(b));
  const startKey = keyOf(boxes, canon(startReach));
  const heap = new Heap();
  let seq = 0;
  const startNode = {
    f: heur(boxes),
    seq: seq++,
    boxes: boxes.slice(),
    player,
    g: 0,
    parent: null,
    edge: '',
  };
  heap.push(startNode);
  const best = new Map([[startKey, 0]]);
  let expansions = 0;

  while (heap.size > 0) {
    if (budgetLeft <= 0) return null;
    const node = heap.pop();
    const nodeKey = keyOf(node.boxes, canon(reachable(node.player, passable, new Set(node.boxes))));
    if ((best.get(nodeKey) ?? Infinity) < node.g) continue;
    if (isGoalState(node.boxes)) {
      if (requireExit && exitCell !== null) {
        const boxSet = new Set(node.boxes);
        if (boxSet.has(exitCell)) continue;
        const reach = reachable(node.player, passable, boxSet);
        if (!reach.has(exitCell)) continue;
      }
      // Reconstruct moves.
      let s = '';
      let n = node;
      const parts = [];
      while (n && n.edge) {
        parts.push(n.edge);
        n = n.parent;
      }
      parts.reverse();
      return parts.join('');
    }
    expansions++;
    budgetLeft--;
    const boxSet = new Set(node.boxes);
    const reach = reachable(node.player, passable, boxSet);
    for (let bi = 0; bi < node.boxes.length; bi++) {
      const b = node.boxes[bi];
      const [br, bc] = rc(b, W);
      for (const d of DIRS) {
        const destR = br + d.dr;
        const destC = bc + d.dc;
        if (destR < 0 || destR >= H || destC < 0 || destC >= W) continue;
        const dest = destR * W + destC;
        if (!boxCells.has(dest) || boxSet.has(dest)) continue;
        if (dead.has(dest)) continue;
        const pushR = br - d.dr;
        const pushC = bc - d.dc;
        if (pushR < 0 || pushR >= H || pushC < 0 || pushC >= W) continue;
        const pusher = pushR * W + pushC;
        if (!reach.has(pusher)) continue;
        const walk = walkPath(node.player, pusher, passable, boxSet);
        if (walk === null) continue;
        const nb = node.boxes.slice();
        nb[bi] = dest;
        nb.sort((x, y) => x - y);
        const nReach = reachable(b, passable, new Set(nb));
        const nKey = keyOf(nb, canon(nReach));
        const ng = node.g + 1;
        if ((best.get(nKey) ?? Infinity) <= ng) continue;
        best.set(nKey, ng);
        heap.push({
          f: ng + heur(nb),
          seq: seq++,
          boxes: nb,
          player: b,
          g: ng,
          parent: node,
          edge: walk + d.ch,
        });
      }
    }
  }
  return null;
}

// ------------------------------------------------------- room decomposition
function decompose(level) {
  const { H, W, walls, goals, boxes, player } = level;
  const [pr] = rc(player, W);
  const seen = new Set();
  const rooms = [];
  for (let r = 0; r < H; r++) {
    for (let c = 0; c < W; c++) {
      const idx = r * W + c;
      if (r === pr || walls.has(idx) || seen.has(idx)) continue;
      // Flood fill this component (hallway row excluded).
      const cells = new Set([idx]);
      const q = [idx];
      seen.add(idx);
      let qi = 0;
      while (qi < q.length) {
        const cur = q[qi++];
        const [cr, cc] = rc(cur, W);
        for (const d of DIRS) {
          const nr = cr + d.dr;
          const nc = cc + d.dc;
          if (nr < 0 || nr >= H || nc < 0 || nc >= W || nr === pr) continue;
          const nidx = nr * W + nc;
          if (walls.has(nidx) || seen.has(nidx)) continue;
          seen.add(nidx);
          cells.add(nidx);
          q.push(nidx);
        }
      }
      const rBoxes = boxes.filter((b) => cells.has(b));
      const rGoals = [...goals].filter((g) => cells.has(g));
      if (rBoxes.length === 0 || rBoxes.length !== rGoals.length) return null;
      // Unique door: the room cell adjacent to a hallway floor cell.
      const doors = [...cells].filter((cell) => {
        const [cr, cc] = rc(cell, W);
        const hr = cr + 1 === pr ? cr + 1 : cr - 1 === pr ? cr - 1 : -1;
        return hr >= 0 && !walls.has(hr * W + cc);
      });
      if (doors.length !== 1) return null;
      const door = doors[0];
      const [dr, dc] = rc(door, W);
      const entry = (dr + 1 === pr ? dr + 1 : dr - 1) * W + dc;
      rooms.push({ cells, boxes: rBoxes, goals: new Set(rGoals), door, entry });
    }
  }
  if (rooms.length === 0) return null;
  const totalBoxes = rooms.reduce((s, r) => s + r.boxes.length, 0);
  if (totalBoxes !== boxes.length) return null;
  return rooms;
}

// ------------------------------------------------------------- whole level
function solveLevel(level) {
  const { H, W, walls, goals, boxes, player } = level;
  const allCells = new Set();
  for (let r = 0; r < H; r++)
    for (let c = 0; c < W; c++) {
      const idx = r * W + c;
      if (!walls.has(idx)) allCells.add(idx);
    }
  const rooms = decompose(level);
  if (rooms) {
    // Rooms whose door cell is a goal go last: ending with a box on the
    // door traps the player inside, which only the final room can afford.
    const order = rooms
      .map((room, i) => ({ room, i, doorGoal: room.goals.has(room.door) }))
      .sort((a, b) => Number(a.doorGoal) - Number(b.doorGoal) || a.i - b.i)
      .map((x) => x.room);
    let moves = '';
    let curPlayer = player;
    const curBoxes = new Set(boxes);
    const passableAll = { cells: allCells, W, H };
    const queue = order.slice();
    let guard = 0;
    while (queue.length > 0 && guard++ < 20) {
      const room = queue.shift();
      const isLast = queue.length === 0;
      // Walk to this room's entry cell.
      const walk = walkPath(curPlayer, room.entry, passableAll, curBoxes);
      if (walk === null) return null;
      moves += walk;
      curPlayer = room.entry;
      const floor = new Set(room.cells);
      floor.add(room.entry);
      const roomBoxes = room.boxes.filter((b) => curBoxes.has(b));
      const sol = solvePush(floor, room.cells, room.goals, roomBoxes, curPlayer, room.door, !isLast, W, H);
      if (sol === null) {
        if (!isLast) {
          // Retry this room last, without the exit constraint.
          queue.push(room);
          continue;
        }
        return null;
      }
      // Replay the room solution to update global box/player positions.
      const replay = replayMoves(level, curBoxes, curPlayer, sol);
      if (!replay) return null;
      curPlayer = replay.player;
      curBoxes.clear();
      for (const b of replay.boxes) curBoxes.add(b);
      moves += sol;
    }
    if (queue.length > 0) return null;
    // Sanity: every box on a goal.
    if (![...curBoxes].every((b) => goals.has(b))) return null;
    return moves;
  }
  // Fallback: whole-level push A*.
  return solvePush(allCells, allCells, goals, boxes, player, null, false, W, H);
}

// Replay a move string on the full level; returns final {boxes, player} or null.
function replayMoves(level, boxSetIn, playerIn, moves) {
  const { H, W, walls } = level;
  const boxes = new Set(boxSetIn);
  let player = playerIn;
  for (const ch of moves) {
    const d = DIRS.find((x) => x.ch === ch);
    if (!d) return null;
    const [r, c] = rc(player, W);
    const nr = r + d.dr;
    const nc = c + d.dc;
    if (nr < 0 || nr >= H || nc < 0 || nc >= W) return null;
    const nidx = nr * W + nc;
    if (walls.has(nidx)) return null;
    if (boxes.has(nidx)) {
      const br = nr + d.dr;
      const bc = nc + d.dc;
      if (br < 0 || br >= H || bc < 0 || bc >= W) return null;
      const bidx = br * W + bc;
      if (walls.has(bidx) || boxes.has(bidx)) return null;
      boxes.delete(nidx);
      boxes.add(bidx);
    }
    player = nidx;
  }
  return { boxes, player };
}

// --------------------------------------------------------------------- main
function main() {
  const args = process.argv.slice(2);
  // The harness runs with /input and /output mounted; outside a container
  // (local runs, grade-local --driver process) the same two directories
  // arrive as MAKEFUN_INPUT_DIR / MAKEFUN_OUTPUT_DIR.
  let inputDir = process.env.MAKEFUN_INPUT_DIR ?? '/input';
  let outputDir = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';
  for (let i = 0; i < args.length; i++) {
    if (args[i] === '--input' && args[i + 1]) inputDir = args[++i];
    else if (args[i] === '--output' && args[i + 1]) outputDir = args[++i];
  }
  const ids = fs
    .readdirSync(inputDir, { withFileTypes: true })
    .filter((e) => e.isDirectory())
    .map((e) => e.name)
    .sort();
  for (const id of ids) {
    const levelPath = path.join(inputDir, id, 'level.txt');
    let moves = '';
    try {
      const level = parseLevel(fs.readFileSync(levelPath, 'utf8'));
      const sol = solveLevel(level);
      if (sol !== null) moves = sol;
    } catch {
      moves = '';
    }
    const outDir = path.join(outputDir, id);
    fs.mkdirSync(outDir, { recursive: true });
    fs.writeFileSync(path.join(outDir, 'solution.txt'), moves + '\n', 'utf8');
  }
}

main();
