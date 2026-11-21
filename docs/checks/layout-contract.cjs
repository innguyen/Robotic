// P0 reference for the documented hash/split contract; not a T1 fixture validator.
const { createHash } = require('node:crypto');

function micrometres(value) {
  if (!Number.isFinite(value)) throw new Error('coordinate must be finite');
  const scaled = Math.round(value * 1e6);
  if (!Number.isSafeInteger(scaled) || scaled / 1e6 !== value) {
    throw new Error('coordinate must have exact micrometre precision');
  }
  return scaled === 0 ? 0 : scaled;
}

function rectangle(bounds) {
  if (!Array.isArray(bounds) || bounds.length !== 4) throw new Error('invalid bounds');
  const result = bounds.map(micrometres);
  if (result[0] >= result[2] || result[1] >= result[3]) throw new Error('inverted bounds');
  return result;
}

function canonicalLayout(episode) {
  const [xmin, ymin, xmax, ymax] = rectangle(episode.bounds_m);
  const subtract = (a, b) => {
    const result = a - b;
    if (!Number.isSafeInteger(result)) throw new Error('coordinate span too large');
    return result;
  };
  const boxes = episode.obstacles.map(obstacle => {
    const box = rectangle(obstacle.bounds_m);
    return [subtract(box[0], xmin), subtract(box[1], ymin),
      subtract(box[2], xmin), subtract(box[3], ymin)];
  });
  const xs = [...new Set(boxes.flatMap(box => [box[0], box[2]]))].sort((a, b) => a - b);
  const slabs = [];
  for (let i = 0; i + 1 < xs.length; i++) {
    const left = xs[i], right = xs[i + 1];
    const intervals = boxes.filter(box => box[0] <= left && box[2] >= right)
      .map(box => [box[1], box[3]]).sort((a, b) => a[0] - b[0] || a[1] - b[1]);
    const merged = [];
    for (const interval of intervals) {
      const last = merged.at(-1);
      if (last && interval[0] <= last[1]) last[1] = Math.max(last[1], interval[1]);
      else merged.push([...interval]);
    }
    if (!merged.length) continue;
    const last = slabs.at(-1);
    if (last && last[1] === left && JSON.stringify(last[2]) === JSON.stringify(merged)) {
      last[1] = right;
    } else slabs.push([left, right, merged]);
  }
  // Key order is part of rect-union-um-v1. All numbers are safe integers.
  return JSON.stringify({ bounds_um: [subtract(xmax, xmin), subtract(ymax, ymin)],
    slabs, version: 'rect-union-um-v1' });
}

function layoutHash(episode) {
  return createHash('sha256').update(canonicalLayout(episode), 'utf8').digest('hex');
}

function checkLayoutRegistry(episodes) {
  const ids = new Map(), hashes = new Map(), families = new Map();
  const idPattern = /^[A-Za-z0-9][A-Za-z0-9_-]{0,79}$/;
  for (const episode of episodes) {
    for (const field of ['environment_id', 'layout_family_id']) {
      if (typeof episode[field] !== 'string' || !idPattern.test(episode[field])) {
        throw new Error(`invalid ${field}`);
      }
    }
    if (!['dev', 'test'].includes(episode.split)) throw new Error('invalid split');
    const hash = layoutHash(episode);
    if (hash !== episode.layout_hash) throw new Error('layout_hash mismatch');
    const previous = ids.get(episode.environment_id);
    if (previous && previous.hash !== hash) throw new Error('environment geometry mismatch');
    if (previous && previous.family !== episode.layout_family_id) throw new Error('environment family mismatch');
    if (previous && previous.split !== episode.split) throw new Error('environment split leakage');
    const owner = hashes.get(hash);
    if (owner && owner.id !== episode.environment_id) throw new Error('layout alias');
    if (owner && owner.split !== episode.split) throw new Error('hash split leakage');
    const familySplit = families.get(episode.layout_family_id);
    if (familySplit && familySplit !== episode.split) throw new Error('family split leakage');
    ids.set(episode.environment_id, { hash, split: episode.split, family: episode.layout_family_id });
    hashes.set(hash, { id: episode.environment_id, split: episode.split });
    families.set(episode.layout_family_id, episode.split);
  }
}

module.exports = { canonicalLayout, layoutHash, checkLayoutRegistry };
