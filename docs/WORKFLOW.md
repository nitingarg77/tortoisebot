# Workflow

One feature goes idea -> supported release through seven commands. Each phase has an owner-agent and hands over through a document; nothing is carried in chat memory. You approve at exactly three points.

| Phase | Command | Agent (model) | Produces | You approve |
|---|---|---|---|---|
| Scope | `/plan <idea>` | product-analyst (sonnet) | `docs/requirements/<feature>.md` | **Scope** |
| Design | `/design <feature>` | architect (opus), then tech-lead (sonnet) | `docs/architecture/<feature>.md`, ADRs, `docs/tasks/T-NNN-*.md`, `INDEX.md` | **Design** |
| Build | `/implement T-NNN` | developer + qa-engineer (sonnet) | code on `feature/T-NNN-<slug>`, `docs/test-reports/T-NNN.md` | - |
| Review | `/review T-NNN` | reviewer (sonnet, read-only, fresh context) | verdict written into the ticket | merge to `frontier-exploration` is yours |
| Release | `/release <version>` | devops (haiku) | `docs/releases/<version>.md` | **Deploy** |
| Support | `/incident <text>` | support-engineer (haiku), then architect for RCA | `docs/incidents/<id>.md`, bug ticket | - |
| Overview | `/status` | - | per-stage table, blocked-on-you / blocked-on-Claude | - |

## Ticket status line
`ready` -> `in progress` -> `awaiting review` -> `done`; or `changes requested` (back to `/implement`), `blocked: escalate` (3 failed attempts; rerun `/implement T-NNN opus`), `needs-architect`.

## Rules the hooks enforce (repo adopted via `.claude/kit.conf`)
- No commits or pushes on `frontier-exploration`; no force-push; no `rm -rf`; no Claude attribution in commits.
- Lint on every edited source file; the fast test suite runs before a task may finish and **zero tests counts as failure**.
- Each agent can edit only its own folders (the reviewer, nothing).
- Heavy jobs (simulation, SLAM or Nav2 runs, `colcon build`) are tickets of type `job`, never run inline; anything that drives the robot follows CLAUDE.md section 2.

## Where things are
Program layer: `MODULE.md`, `CLAUDE.md` (sections 1-8), `README.md`, `BUILD_LOG.md` (auto-generated). Feature layer: folders above. Templates: `docs/templates/`. Kit: `~/.claude/kit` (shared across repos and machines).
