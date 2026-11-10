# Prompts for Claude Code

Paste one prompt per session. Each milestone starts in a fresh session so the context
stays small. Pick the model with `/model` before pasting.

## First session (M0, then M1)

```
Read CLAUDE.md, STATUS.md and docs/MILESTONES.md. Do milestone M0, verify its acceptance
lines, commit, and update STATUS.md. Then start M1: read spec sections 3, 5, 6 and 8,
write the M1 tests first, and work until they pass or you are blocked. Do not start M2.
Finish by updating STATUS.md with what is verified and what is open.
```

## Every later milestone

```
Read CLAUDE.md, STATUS.md and docs/MILESTONES.md. Continue with milestone M<n>. Read only
the spec sections that milestone lists. Write its acceptance tests first, then make them
pass. Commit as you go. Do not start the next milestone. Finish by updating STATUS.md.
```

## Resuming an unfinished milestone

```
Read CLAUDE.md and STATUS.md. Milestone M<n> is unfinished. Run its tests, tell me in five
lines what passes and what fails, then continue with the first failing acceptance line.
```

## Asking for a review instead of code

```
Read STATUS.md and the diff since the last tag. In ten lines: what would you not trust yet
in milestone M<n>, and which single test would raise your confidence most? Do not change code.
```

## After a hardware measurement

```
I measured <part>: <values>. Update config/robot.yaml, add a decision entry, run make cad
and make test, and tell me what changed downstream.
```
