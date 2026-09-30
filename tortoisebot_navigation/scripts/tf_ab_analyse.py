#!/usr/bin/env python3
"""Summarise tf_ab_launch.sh runs. No ROS; reads the files it leaves behind.

    tf_ab_analyse.py <dir> <tag> [<tag> ...]

For each launch, and for each probe window inside it, this reports two
buffers side by side on one clock:

  controller_server  -- from its "Transform data too old" lines. Each carries a
                        Data time (the query) and a Transform time (the newest
                        map->odom it holds). No lines during a goal means its
                        lookups were within tolerance: healthy.
  tf_watch           -- an independent buffer in a third process, sampled once
                        a second (wall, buffer_newest, wire_newest).

The distinction MODULE.md needs is whether one froze while the other did not:

  both stale at once      -> Cartographer stopped publishing   (a)
  only controller stale   -> controller_server's listener       (b)
  neither                 -> no freeze in this window

"Frozen" means one Transform time held while Data time advanced more than
3 s, which is this repository's definition (MODULE.md): a frozen Transform
time is a hang, a moving one is lag.
"""

import csv
import os
import re
import sys

PAIR = re.compile(r'Data time: (\d+)s (\d+)ns, Transform time: (\d+)s (\d+)ns')
NOTE = re.compile(r'^(\d+\.\d+) (.*)$')
FROZEN_S = 3.0


def t(sec, ns):
    return int(sec) + int(ns) * 1e-9


def controller_pairs(path):
    """[(data_time, transform_time)] from controller_server's tf_help lines."""
    out = []
    with open(path, errors='replace') as f:
        for line in f:
            if 'controller_server' not in line:
                continue
            m = PAIR.search(line)
            if m:
                out.append((t(m.group(1), m.group(2)), t(m.group(3), m.group(4))))
    return out


def frozen_spans(pairs):
    """{transform_time: data-time span it was held for}."""
    spans = {}
    for d, x in pairs:
        lo, hi = spans.get(x, (d, d))
        spans[x] = (min(lo, d), max(hi, d))
    return {x: hi - lo for x, (lo, hi) in spans.items()}


def tfwatch_rows(path):
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                w = float(r['wall'])
                b = float(r['buffer_newest']) if r['buffer_newest'] else None
                rows.append((w, b))
            except (KeyError, ValueError):
                pass
    return rows


def probes(meta_path):
    """[(label, start, end, result_line)] from the harness's notes."""
    starts, out = {}, []
    with open(meta_path) as f:
        for line in f:
            m = NOTE.match(line.strip())
            if not m:
                continue
            wall, text = float(m.group(1)), m.group(2)
            pm = re.match(r'probe (\S+) (start|end)(.*)', text)
            if not pm:
                continue
            label, kind, rest = pm.groups()
            if kind == 'start':
                starts[label] = wall
            elif label in starts:
                res = rest.split('RESULT', 1)
                out.append((label, starts[label], wall,
                            ('RESULT' + res[1]).strip() if len(res) > 1 else
                            rest.strip()))
    return out


def meta_value(meta_path, key):
    with open(meta_path) as f:
        for line in f:
            if key in line:
                return line.split(' ', 1)[1].strip()
    return ''


def summarise(d, tag):
    meta = os.path.join(d, tag + '.meta')
    if not os.path.exists(meta):
        print('%s: no meta file' % tag)
        return
    pairs = controller_pairs(os.path.join(d, tag + '.compute.log'))
    rows = tfwatch_rows(os.path.join(d, tag + '.tfwatch.csv'))
    spans = frozen_spans(pairs)
    worst = max(spans.values()) if spans else 0.0

    print('=' * 72)
    print('%s   %s' % (tag, meta_value(meta, 'rate requested')))
    for key in ('nav2 active', 'settled', 'tf_watch done', 'end load',
                'teardown', 'FAIL'):
        v = meta_value(meta, key)
        if v:
            print('  %s' % v[:150])
    print('  controller_server: %d stale-TF lines, %d distinct Transform '
          'times, longest held %.1f s -> %s'
          % (len(pairs), len(spans), worst,
             'FROZEN' if worst > FROZEN_S else
             ('lagging' if pairs else 'healthy')))

    print('  %-8s %-7s %-22s %-24s %s' % ('probe', 'lines', 'controller stalest',
                                          'tf_watch stalest', 'goal'))
    for label, s, e, res in probes(meta):
        win = [(dd, x) for dd, x in pairs if s <= dd <= e]
        c_age = max((dd - x for dd, x in win), default=None)
        held = frozen_spans(win)
        c_frozen = max(held.values()) if held else 0.0
        w_ages = [w - b for w, b in rows if s <= w <= e and b is not None]
        w_age = max(w_ages) if w_ages else None
        print('  %-8s %-7d %-22s %-24s %s' % (
            label, len(win),
            ('%.1f s%s' % (c_age, ' FROZEN' if c_frozen > FROZEN_S else ''))
            if c_age is not None else 'healthy',
            ('%.2f s (%d samples)' % (w_age, len(w_ages)))
            if w_age is not None else 'no samples',
            res[:60]))


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    for tag in sys.argv[2:]:
        summarise(sys.argv[1], tag)


if __name__ == '__main__':
    main()
