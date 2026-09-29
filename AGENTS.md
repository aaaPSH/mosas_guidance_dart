# AGENTS.md

## Testing

Only perform necessary and meaningful tests that directly verify the modified functionality or critical behavior.

Do not run redundant, low-value, repetitive, or unrelated tests merely for completeness.

Any temporary test program or test code created solely for functional verification must be deleted immediately after the test is completed. Do not leave temporary test files, scripts, debug code, or test-only artifacts in the project.

## Planning Documents

Temporary planning documents must not be added to Git or committed to the repository.

Planning documents may be created when needed during implementation, but they must be deleted immediately after the planned work is completed.

Do not leave temporary plans, implementation notes, task breakdown files, or similar planning artifacts in the project.

## Code Modification

When modifying code, make the change completely and keep the final logic clean and coherent.

Do not preserve obsolete interfaces, compatibility layers, deprecated logic, or unused code unless they are explicitly required.

If old interfaces or implementations are no longer needed, remove them together with their callers and related dead code.

Avoid redundancy, duplicate implementations, compatibility hacks, and zombie code.