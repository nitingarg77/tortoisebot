"""Score Needle's tool selection against the cell's real interfaces.

This is the spike's verdict. It decodes with `complete()`, which returns the
call WITHOUT running it -- verified: the tool body never executes -- so nothing
here can touch a cell even if pointed at the live backend by mistake.

Scoring is exact match on the ordered list of (name, arguments), with defaulted
arguments the utterance never mentioned allowed to be absent or at their default.
A near miss is a miss: filling `arm` with "left" when the operator said right is
not partial credit, it is the wrong arm.

The out-of-scope cases are the ones worth watching. Seven utterances name things
this console genuinely cannot do -- move an arm, change the belt speed, stop the
cell -- and the right answer is an empty call list. A model that invents a plausible
call there is more dangerous on a production cell than one that scores lower overall.

    python run_eval.py [--json out.json] [--verbose]
"""

import argparse
import json
import statistics
import sys
import time

sys.path.insert(0, ".")

import backend as backend_mod
import tb_tools as cobot_tools


DEFAULTS = {("turn", "degrees"): 90, ("drive", "distance"): 0.5, }


def normalise(name, arguments):
    """Drop arguments left at a documented default so they do not count against
    a call that was otherwise right."""
    out = {}
    for key, value in (arguments or {}).items():
        if DEFAULTS.get((name, key)) == value:
            continue
        out[key] = round(value, 6) if isinstance(value, float) else value
    return out


def same(expected, actual):
    if len(expected) != len(actual):
        return False
    for want, got in zip(expected, actual):
        if want["name"] != got.get("name"):
            return False
        if normalise(want["name"], want.get("arguments")) != \
           normalise(got.get("name"), got.get("arguments")):
            return False
    return True


def render(calls):
    return " ; ".join(
        f"{c.get('name')}({', '.join(f'{k}={v!r}' for k, v in (c.get('arguments') or {}).items())})"
        for c in calls) or "(no call)"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", dest="json_out")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--evalset", default="evalset.jsonl")
    args = parser.parse_args()

    cases = [json.loads(line) for line in open(args.evalset) if line.strip()]

    mock = backend_mod.MockBackend()
    load_start = time.time()
    agent = cobot_tools.agent(mock)
    load_seconds = time.time() - load_start

    results, latencies = [], []
    for case in cases:
        # MUST reset between cases. The engine keeps conversation state across
        # complete() calls, and prior unrelated queries change the answer to the
        # next one -- deterministically, not as noise. Scoring 30-odd utterances
        # in one process without this measures accuracy under accumulated
        # history, which is not what a one-shot operator command is.
        agent.reset()
        start = time.time()
        out = agent.complete(case["utterance"])
        elapsed = time.time() - start
        latencies.append(elapsed)
        calls = out.get("function_calls") or []
        ok = same(case["expect"], calls)
        results.append({
            "id": case["id"], "utterance": case["utterance"], "pass": ok,
            "expected": case["expect"], "got": calls,
            "confidence": out.get("confidence"), "seconds": round(elapsed, 3),
            "peak_ram_mb": out.get("peak_ram_mb"),
            "reasoning": out.get("reasoning"),
        })
        if args.verbose or not ok:
            mark = "PASS" if ok else "FAIL"
            print(f"[{mark}] {case['id']:9s} {case['utterance']}")
            if not ok:
                print(f"           want: {render(case['expect'])}")
                print(f"           got : {render(calls)}")
            print(f"           conf: {out.get('confidence')}  "
                  f"{elapsed*1000:.0f} ms")

    assert not mock.writes, f"complete() caused side effects: {mock.writes}"

    groups = {}
    for row, case in zip(results, cases):
        groups.setdefault(case["id"].split("-")[0], []).append(row["pass"])

    total = sum(r["pass"] for r in results)
    print("\n" + "=" * 62)
    print(f"model load        {load_seconds:.1f} s")
    print(f"latency           median {statistics.median(latencies)*1000:.0f} ms, "
          f"max {max(latencies)*1000:.0f} ms")
    peaks = [r["peak_ram_mb"] for r in results if r["peak_ram_mb"]]
    if peaks:
        print(f"peak RAM          {max(peaks):.0f} MB")
    print("-" * 62)
    for group in ("read", "tune", "act", "multi", "oos"):
        if group in groups:
            passes = groups[group]
            print(f"{group:9s} {sum(passes):2d}/{len(passes):<2d} "
                  f"{sum(passes)/len(passes):6.0%}")
    print("-" * 62)
    print(f"{'TOTAL':9s} {total:2d}/{len(results):<2d} {total/len(results):6.0%}")
    print("=" * 62)
    print("\nno side effects from complete(): confirmed")

    if args.json_out:
        with open(args.json_out, "w") as handle:
            json.dump({"results": results,
                       "load_seconds": load_seconds,
                       "total": total, "count": len(results)}, handle, indent=2)
        print(f"wrote {args.json_out}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
