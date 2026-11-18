---
name: folder-index
description: >-
  Maintains INDEX.md, a compact machine-readable brief of each code folder:
  format, scope, and update triggers. Use when creating a code folder, adding,
  removing, or renaming code files, changing public signatures, or reading or
  writing INDEX.md.
---

# Folder Index

`INDEX.md` lets an agent understand a folder without opening its files. Optimize for tokens and parsing: fixed keys, one line per file, no prose.

## Scope

- Index every folder that directly contains code files (any language).
- Exempt: `tests/` folders and generated, vendored, or build folders (`node_modules`, `.venv`, `dist`, `build`, `__pycache__`).
- One entry per file. List public API only: symbols other files can import or call.

## Update triggers

Update `INDEX.md` in the same turn as the code when:

- A code file is added, removed, or renamed.
- A public signature is added, removed, or changed.
- A file's role, `deps`, or `io` changes.
- A subfolder is added or removed.

Otherwise (body-only edits, private changes) do not touch it. If `INDEX.md` is missing in a folder you are editing, create it.

Edit only the affected entries. Copy signatures verbatim from source.

## Format

```text
folder: src/net
purpose: TCP client and message framing
files:
- client.py — connection pool, request send; deps: framing.py, ../config.py; io: network
  - class Client(host: str, port: int)
  - Client.send(msg: Message) -> Reply
- framing.py — length-prefixed encode/decode; deps: none
  - def encode(msg: Message) -> bytes
  - def decode(data: bytes) -> Message
subfolders:
- tls/ — certificate loading
```

Rules:

- Keys in order: `folder`, `purpose`, `files`, `subfolders`. Omit `subfolders` if none.
- File line: `- <name> — <role>; deps: <project-internal imports or none>; io: <network|filesystem|global state>`. Omit `io` when none.
- Signatures are indented bullets under their file, one per public symbol, in the source language's own syntax.
- Each subfolder owns its `INDEX.md`; do not describe its contents here.
- No examples, rationale, history, or TODOs. Rationale belongs in code comments or commit messages.
