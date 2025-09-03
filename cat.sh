#!/bin/bash

if [ $# -ne 1 ]; then
  echo "Usage: $0 <file>"
  exit 1
fi

output_file="$1"

if [ -f "${output_file}" ]; then
  echo "Error: File ${output_file} already exists."
  exit 1
fi

parts=$(ls "${output_file}"_* 2>/dev/null)
if [ -z "$parts" ]; then
  echo "Error: No parts found for ${output_file}."
  exit 1
fi

echo "🔧 Merging parts into ${output_file}..."
cat $(ls "${output_file}"_* | sort) > "$output_file"
echo "✅ Done. Output file: ${output_file}"

