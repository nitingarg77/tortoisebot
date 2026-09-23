# spike_needle_nl — natural-language console for TortoiseBot

Can [Needle 3](https://github.com/cactus-compute/needle) (2-bit, 8–29 MB,
on-device) drive an offline voice/text console over TortoiseBot's Nav2 stack?

**21-22/32 (66-69%) on the base model.** Promising enough to fine-tune;
not deployable yet, because it refuses only 3 of 7 requests it has no tool for.
Numbers and method in [RESULTS.md](RESULTS.md).

The same spike scored 37-40% on the cobot cell (separate repo: `cobot_ws/spike_needle_nl`).
TortoiseBot is the fair test: Pi-class hardware where a tiny model is the point,
and house-navigation phrasing that sits inside Needle's training distribution.

Outside `src/`, so `colcon build` ignores it. Delete the directory to remove it.

## Layout

- `tb_tools.py` — 10 tools over Nav2, each tagged `read`/`tune`/`act`
- `backend.py` — `MockBackend` (no ROS) and `Nav2Backend` (the real bridge)
- `evalset.jsonl` — 32 spoken commands and the call each should produce
- `run_eval.py` — exact-match scorer; never executes a tool body
- `console.py` — REPL with the risk gate

## Run it

```sh
python3 -m venv nv && ./nv/bin/pip install cactus-needle
export NEEDLE_TELEMETRY=0 DO_NOT_TRACK=1
./nv/bin/python run_eval.py --verbose
./nv/bin/python console.py            # mock backend
./nv/bin/python console.py --live     # real robot; act tools MOVE IT
```

## Read this before using the API anywhere else

**`complete()` is not stateless.** The engine keeps conversation state across
calls and a previous unrelated query deterministically changes the next answer.
`Needle.reset()` clears it. Calling it per utterance was worth **32 points** here.
Nothing in the upstream README mentions this.

## Bridge status

`Nav2Backend` is validated against a live Ignition + Nav2 stack: all reads, the
parameter write, goal submission and cancellation work, and **all 5 named places
were reached with at most 0.15 m error**. Doing this found four bugs the mock
could not -- two QoS mismatches, a wrong navigation status, and three unnavigable
waypoint coordinates -- all fixed.

It also produced a retraction: an earlier version of this spike blamed a
localisation bug in this workspace. That was wrong. The cause was two sim stacks
running at once, so two `/clock` publishers made simulated time run backwards and
cleared every TF buffer. **Nothing in tb_ws needed fixing.** Check
`ros2 topic info /clock` shows `Publisher count: 1` before believing any TF
symptom here. Details in [RESULTS.md](RESULTS.md).

## The safety inversion

The model never executes anything. `complete()` decodes a call without running the
tool body; the console applies the risk tier and only then calls the function
itself. Every `act` tool needs confirmation regardless of confidence — verified:
three attempted moves with no confirmation produced zero backend writes.

That matters more than the score. Confidence does **not** separate right from
wrong here ("what's the weather in Lagos" → `get_nearest_obstacle()` at 1.00), so
the gate, not the model, is what makes this safe to point at a robot.
