#!/usr/bin/env bash

file=$(jq -r '.tool_response.filePath // .tool_input.file_path // empty')

[[ -z "$file" ]] && exit 0
[[ "$file" != *.md ]] && exit 0
[[ ! -f "$file" ]] && exit 0

# Step 1: Format (table alignment, spacing)
if ! command -v prettier &>/dev/null; then
  echo "prettier not found. Install with: npm install -g prettier" >&2
  exit 1
fi

prettier --write "$file" 2>/dev/null

# Step 2: Lint fix (heading spacing, list markers, etc.)
if ! command -v markdownlint-cli2 &>/dev/null; then
  echo "markdownlint-cli2 not found. Install with: npm install -g markdownlint-cli2" >&2
  exit 1
fi

markdownlint-cli2 --fix "$file" 2>/dev/null

exit 0
