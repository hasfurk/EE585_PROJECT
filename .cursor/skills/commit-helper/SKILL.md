---
name: commit-helper
description: >-
  Commits the staged changes with a Brief line plus bullet list: file-level
  bullets for critical changes, folder-level bullets for the rest. Use when the
  user asks to commit staged changes or write a commit message.
---

# Commit Helper

Commit only what is already staged. Do not push unless the user asks.

## Message format

```text
Brief: <1-2 sentence summary of the overall change>.
- <path/to/file.ext>: <what changed and why>.
- <path/to/folder/>: <what changed in that group and why>.
```

- **File bullet** (repo-relative path): a file whose logic or behavior changed, or a doc stating new behavior.
- **Folder bullet** (path ending in `/`, one sentence per group): tests, fixtures, generated data, lockfiles, config, `INDEX.md` files, and mechanical edits (renames, imports, formatting), including in source files.
- If a folder holds both a critical file and supporting files, the critical file keeps its own bullet; the folder bullet covers only the rest.
- Describe what changed and why. One sentence per bullet, no line wrapping.
- Omit a group with no meaningful delta (e.g. lockfile noise) unless it is the whole change.

Example:

```text
Brief: Retry failed uploads with backoff and surface the final error to the caller.
- src/upload/client.py: Retry up to 3 times with exponential backoff and raise UploadError on the last failure.
- docs/upload.md: Document the retry policy and the new error.
- tests/: Cover retry success, exhaustion, and backoff timing.
```

## Steps

1. Run `git status`, `git diff --staged`, and `git log -5 --oneline` (match recent style).
2. If nothing is staged, stop and tell the user. Never run `git add`.
3. If a staged file likely holds secrets (`.env`, `credentials.json`, key files), do not commit; warn the user.
4. Write the message to `.git/COMMIT_EDITMSG`, then run `git commit -F .git/COMMIT_EDITMSG`. Never use `-a` or `-am`.
5. Run `git status` to confirm. Reply with the commit hash, or the error.
6. Push only when asked: `git push`, or `git push -u origin HEAD` if the branch has no upstream.

## Safety

- No `--amend`, `--no-verify`, or force-push unless the user explicitly requests it; warn on force-push to `main` or `master`.
- If a pre-commit hook fails, fix the issue and make a new commit; do not amend.
