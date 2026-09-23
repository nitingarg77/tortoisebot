"""Operator console: decode a request, gate it, then execute what is allowed.

The gate is the point of this file, not the model. Needle returns a call and a
calibrated confidence; this decides whether that call is allowed to happen:

  read  -- run it if confidence >= READ_MIN, else ask for a rephrase.
  tune  -- always confirm, showing old value -> new value.
  act   -- always confirm, with a warning line saying what it does physically.

Nothing is executed by the model. `complete()` decodes without running the tool
body (verified in run_eval.py), and this module calls the Python function itself
only after the gate passes. That inversion is what makes a 14%-accurate model
safe to point at a production cell: a wrong call becomes a declined prompt, not a
dropped part.

    python console.py                 # mock backend, no ROS needed
    python console.py --live          # real cell, needs a sourced Humble env
"""

import argparse
import sys

sys.path.insert(0, ".")

import backend as backend_mod
import tb_tools as cobot_tools

READ_MIN = 0.60

WARN = {
    "go_to_place": "drives the robot across the house",
    "go_home": "drives the robot back to the dock",
    "turn": "rotates the robot in place, open loop",
    "drive": "drives the robot straight, open loop and with no obstacle check",
    "stop": "cancels the active navigation goal",
}


def describe(call):
    arguments = call.get("arguments") or {}
    inner = ", ".join(f"{k}={v!r}" for k, v in arguments.items())
    return f"{call.get('name')}({inner})"


def confirm(prompt):
    try:
        return input(f"{prompt} [y/N] ").strip().lower() in ("y", "yes")
    except (EOFError, KeyboardInterrupt):
        print()
        return False


def handle(agent, backend, text):
    # Each utterance is an independent command, so the engine's conversation
    # state is cleared first. Without this, an earlier unrelated request changes
    # the answer to this one -- deterministically, not as noise. Measured at
    # roughly 30 points of accuracy on the eval; see RESULTS.md.
    agent.reset()
    out = agent.complete(text)
    calls = out.get("function_calls") or []
    confidence = out.get("confidence")

    if not calls:
        print(f"  no tool covers that (confidence {confidence}).")
        if out.get("reasoning"):
            print(f"  model: {out['reasoning']}")
        return

    print(f"  confidence {confidence}   {out.get('seconds', '')}")
    for call in calls:
        name = call.get("name")
        tier = cobot_tools.RISK.get(name)
        function = cobot_tools._functions_by_name().get(name)

        if tier is None or function is None:
            print(f"  ! {describe(call)} is not a tool of this console; refused")
            continue

        if tier == cobot_tools.READ:
            if confidence is not None and confidence < READ_MIN:
                print(f"  ? {describe(call)} -- confidence {confidence:.2f} below "
                      f"{READ_MIN}; rephrase rather than guess")
                continue
        else:
            note = WARN.get(name)
            print(f"  ! {tier.upper()}: {describe(call)}"
                  + (f"\n    {note}" if note else ""))
            if not confirm("    run it?"):
                print("    declined")
                continue

        try:
            result = function(**(call.get("arguments") or {}))
        except Exception as err:                      # noqa: BLE001
            print(f"  x {describe(call)} raised {err!r}")
            continue
        print(f"  -> {result}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--live", action="store_true",
                        help="drive the real robot instead of the mock")
    args = parser.parse_args()

    if args.live:
        backend = backend_mod.Nav2Backend()
        print("backend: Nav2 -- act tools MOVE THE REAL ROBOT")
    else:
        backend = backend_mod.MockBackend()
        print("backend: mock -- nothing leaves this process")

    agent = cobot_tools.agent(backend)
    print(f"{len(cobot_tools.ALL)} tools. Ctrl-D to quit.\n")

    while True:
        try:
            text = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not text:
            continue
        handle(agent, backend, text)

    if hasattr(backend, "close"):
        backend.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
