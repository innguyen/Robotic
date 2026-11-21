// Run: node --test docs/checks/benchmark-contract.test.cjs
// These are P0 document/reference checks, not scenario or runner acceptance tests.
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const { canonicalLayout, layoutHash, checkLayoutRegistry } = require('./layout-contract.cjs');
const root = path.resolve(__dirname, '../..');
const read = name => fs.readFileSync(path.join(root, 'docs', name), 'utf8');
const spec = read('benchmark-t1-spec.md');
const report = read('benchmark-report.md');
const jsonExamples = text => [...text.matchAll(/```json\r?\n([\s\S]*?)\r?\n```/g)]
  .map(match => JSON.parse(match[1]));
const suite = jsonExamples(spec)[0];
const original = () => structuredClone(suite.episodes[0]);
const rehash = episode => ({ ...episode, layout_hash: layoutHash(episode) });

test('known canonical bytes and SHA-256 vector', () => {
  assert.equal(canonicalLayout(original()), '{"bounds_um":[5000000,4000000],"slabs":[[1800000,2200000,[[2800000,3200000]]]],"version":"rect-union-um-v1"}');
  assert.equal(layoutHash(original()), 'eb085274e87be582ce14b082a7479a0c7e3b97f1ec7f428f278e3597806f56fb');
});
test('declared example hash is computed, not trusted', () => {
  assert.equal(original().layout_hash, layoutHash(original()));
  checkLayoutRegistry(suite.episodes);
});
test('obstacle ordering and names do not affect geometry', () => {
  const a = original();
  a.obstacles.push({ id: 'second', bounds_m: [3, 0, 3.5, 1] });
  const b = structuredClone(a);
  b.obstacles.reverse().forEach(box => { box.id = 'renamed'; });
  assert.equal(layoutHash(a), layoutHash(b));
});
test('union ignores overlap, duplicates and rectangle partitioning', () => {
  const a = original(), b = original();
  b.obstacles = [
    { bounds_m: [.8, .8, 1, 1.2] }, { bounds_m: [1, .8, 1.2, 1.2] },
    { bounds_m: [.9, .9, 1.1, 1.1] }, { bounds_m: [.8, .8, 1, 1.2] }
  ];
  assert.equal(layoutHash(a), layoutHash(b));
});
test('both-axis partitions and touching Y intervals preserve an exact union', () => {
  const a = original(), b = original();
  a.obstacles = [{ bounds_m: [0, 0, 2, 2] }];
  b.obstacles = [
    { bounds_m: [0, 0, 1, 1] }, { bounds_m: [0, 1, 1, 2] },
    { bounds_m: [1, 0, 2, 1] }, { bounds_m: [1, 1, 2, 2] }
  ];
  assert.equal(layoutHash(a), layoutHash(b));
});
test('empty slabs do not merge separated boxes', () => {
  const a = original(), b = original();
  a.obstacles = [{ bounds_m: [0, 0, 1, 1] }, { bounds_m: [2, 0, 3, 1] }];
  b.obstacles = [{ bounds_m: [0, 0, 3, 1] }];
  assert.notEqual(layoutHash(a), layoutHash(b));
});
test('translation is normalized', () => {
  const a = original(), b = original();
  const shift = box => box.map((value, i) => Number((value + (i % 2 ? 7 : 10)).toFixed(6)));
  b.bounds_m = shift(b.bounds_m);
  b.obstacles.forEach(box => { box.bounds_m = shift(box.bounds_m); });
  assert.equal(layoutHash(a), layoutHash(b));
});
test('start, person, seed and labels are not static layout', () => {
  const a = original(), b = original();
  b.start_pose.x_m = 1; b.targets = []; b.seed = 123; b.scenario_id = 'S5';
  assert.equal(layoutHash(a), layoutHash(b));
});
test('bounds and obstacle changes affect the hash', () => {
  const a = original(), b = original(), c = original();
  b.bounds_m[2] += 1; c.obstacles[0].bounds_m[2] += .1;
  assert.notEqual(layoutHash(a), layoutHash(b));
  assert.notEqual(layoutHash(a), layoutHash(c));
});
test('obstacles outside mission bounds are retained', () => {
  const a = original(), b = original();
  b.obstacles.push({ bounds_m: [10, 10, 11, 11] });
  assert.notEqual(layoutHash(a), layoutHash(b));
});
test('invalid geometry precision, finite values and extents are rejected', () => {
  for (const value of [NaN, Infinity, .0000001, 1e10]) {
    const e = original(); e.bounds_m[0] = value;
    assert.throws(() => layoutHash(e));
  }
  for (const bounds of [[0, 0, 0, 1], [1, 0, 0, 1], [0, 0, 1]]) {
    const e = original(); e.bounds_m = bounds;
    assert.throws(() => layoutHash(e));
  }
});
test('one layout can have multiple starts in one split', () => {
  const a = original(), b = original(); b.episode_id = 'other'; b.start_pose.y_m = 1;
  assert.doesNotThrow(() => checkLayoutRegistry([a, b]));
});
test('wrong declared hash is rejected', () => {
  const e = original(); e.layout_hash = '0'.repeat(64);
  assert.throws(() => checkLayoutRegistry([e]), /hash mismatch/);
});
test('same environment ID cannot change geometry', () => {
  const a = original(), b = original(); b.bounds_m[2] += 1;
  assert.throws(() => checkLayoutRegistry([a, rehash(b)]), /geometry mismatch/);
});
test('renaming identical geometry cannot bypass identity', () => {
  const a = original(), b = original(); b.environment_id = 'alias'; b.split = 'test';
  assert.throws(() => checkLayoutRegistry([a, b]), /layout alias/);
});
test('same environment cannot cross dev and test', () => {
  const a = original(), b = original(); b.split = 'test';
  assert.throws(() => checkLayoutRegistry([a, b]), /split leakage/);
});
test('related layouts with different hashes cannot cross dev and test', () => {
  const a = original(), b = original();
  b.environment_id = 'variant'; b.bounds_m[2] += 1; b.split = 'test';
  assert.throws(() => checkLayoutRegistry([a, rehash(b)]), /family split leakage/);
});
test('environment family cannot change; malformed registry IDs/splits are rejected', () => {
  const a = original(), b = original(); b.layout_family_id = 'another';
  assert.throws(() => checkLayoutRegistry([a, b]), /family mismatch/);
  for (const patch of [{ environment_id: '../escape' }, { environment_id: 123 }, { layout_family_id: '' }, { split: 'train' }]) {
    assert.throws(() => checkLayoutRegistry([{ ...a, ...patch }]));
  }
});
test('JSON examples parse and protocol references agree', () => {
  const examples = jsonExamples(spec);
  assert.equal(examples.length, 3);
  assert.equal(examples[0].schema_version, 't1-0.2');
  assert.equal(examples[2].schema_version, 't1-0.2');
  assert.match(report, /Đặc tả T1 phiên bản 0\.2/);
  assert.doesNotMatch(spec, /t1-0\.1/);
});
test('example config paths resolve from repository root, independently of cwd', () => {
  for (const name of ['navigation_yaml', 'monitor_yaml']) {
    const value = suite.shared[name];
    assert(!path.win32.isAbsolute(value)); assert(!value.includes('\\'));
    assert(!value.split('/').includes('..'));
    const target = fs.realpathSync(path.resolve(root, value));
    const relative = path.relative(root, target);
    assert(!relative.startsWith('..') && !path.isAbsolute(relative));
    assert(fs.statSync(target).isFile());
  }
});
test('scenario table documents all five predicates and counterexamples (static only)', () => {
  const rows = spec.split(/\r?\n/).filter(line => /^\| S[1-5] \|/.test(line));
  assert.equal(rows.length, 5);
  for (let i = 0; i < rows.length; i++) {
    const cells = rows[i].split('|').slice(1, -1).map(cell => cell.trim());
    assert.equal(cells[0], `S${i + 1}`); assert(cells[1].length > 40); assert(cells[2].length > 30);
  }
  assert.match(spec, /UNVERIFIED/); assert.match(spec, /scenario_evidence\.gates/);
});
test('B0 feasibility and independent trajectories are explicit (static only)', () => {
  for (const term of ['fixed-order-v2', 'POLICY_STALLED', 'positive controls', 'fallback_goals', 'không dùng chung quỹ đạo']) {
    assert(spec.includes(term), term);
  }
  assert.doesNotMatch(spec, /không còn điểm đủ điều kiện[^\n]*trả `SEARCH_EXHAUSTED`/);
  assert.match(spec, /raw-visible theo camera thực ở pose cuối/);
  assert.match(spec, /Có confirmed target mới\/hợp lệ thì vào approach ngay/);
  assert.match(spec, /x_next = x \+ \(cos\(yaw\)\*vx - sin\(yaw\)\*vy\)\*dt/);
  assert.match(spec, /yaw đi theo góc tăng `wz\*dt` chưa wrap/);
});
test('literature distinguishes comparison from ablation and sources episode count (static only)', () => {
  const barn = report.split(/\r?\n/).find(line => line.startsWith('| BARN') && line.includes('ablation'));
  assert(barn?.includes('không phải ablation'));
  assert.match(report, /1\.000 episode[^\n]*Participation Guidelines[^\n]*https:\/\/aihabitat\.org\/challenge\/2023\//);
});
test('local Markdown links and anchors exist in report, spec, plan and audit', () => {
  const slug = heading => heading.toLowerCase().replace(/[^\p{L}\p{N}_\- ]/gu, '').replace(/ /g, '-');
  for (const name of ['benchmark-report.md', 'benchmark-t1-spec.md', 'benchmark-plan.md', 'benchmark-p0-audit.md']) {
    const document = read(name);
    for (const match of document.matchAll(/\[[^\]\n]+\]\(([^)\s]+)\)/g)) {
      const link = match[1]; if (/^https?:\/\//.test(link)) continue;
      const [file, fragment] = link.split('#');
      const target = path.resolve(root, 'docs', decodeURIComponent(file || name));
      assert(fs.existsSync(target), `${name}: ${link}`);
      if (fragment) {
        const headings = [...fs.readFileSync(target, 'utf8').matchAll(/^#{1,6} (.+)$/gm)]
          .map(heading => slug(heading[1].trim()));
        assert(headings.includes(decodeURIComponent(fragment)), `${name}: missing #${fragment}`);
      }
    }
    assert.equal((document.match(/^```/gm) || []).length % 2, 0, `${name}: unclosed fence`);
  }
});
