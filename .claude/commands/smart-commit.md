---
allowed-tools: Bash(git status:*), Bash(git diff:*), Bash(git log:*), Bash(git add:*), Bash(git commit:*)
description: Intelligently organize and commit git changes with conventional commit messages
---

## Your task

Analyze the git repository state and create well-organized, atomic commits:

1. Review all changes, available tools are:
   - `git status` to see current changes
   - `git diff HEAD` to review unstaged changes
   - `git diff --cached` to review staged changes
   - `git log` to see commit history
2. Group related changes by logical purpose (features, fixes, docs, etc.)
3. Create atomic commits for each group:
   - Stage specific files with `git add <files>`
   - Commit with conventional format: `type(scope): description`
      - feat: new features
      - fix: bug fixes
      - docs: documentation
      - style: formatting/whitespace
      - refactor: code restructuring
      - test: adding tests
      - chore: maintenance tasks
4. Repeat for all groups of changes until all are committed

## Best Practices

- One logical change per commit
- Clear, descriptive commit messages
- Review staged changes before committing
- Use conventional commit format consistently
