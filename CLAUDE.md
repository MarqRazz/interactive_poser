# CLAUDE.md

Guidance for Claude Code (claude.ai/code) in this repository.

Read the same docs a user does, in this order:

- [README.md](README.md): what the tool does, how to run it, how to calibrate a
  cell, build and test commands, troubleshooting.
- [docs/design.md](docs/design.md): why it is built this way. Read before
  changing behaviour that looks arbitrary; most of it is load-bearing.
- [docs/xacro-integration.md](docs/xacro-integration.md): the consumer side, for
  anything touching a description.

Source comments state a constraint and point at the design doc section that
explains it. Keep them that way rather than restoring the explanation inline.

Two repository rules that are not in those docs:

- This is a public standalone package. No cell-specific frame, topic or package
  names in committed files; examples use placeholders.
- Prose follows the workspace writing style: active voice, no em-dashes, no
  emojis, terse comments.
