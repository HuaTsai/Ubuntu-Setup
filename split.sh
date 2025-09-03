#!/bin/bash

if [ $# -ne 1 ]; then
  echo "Usage: $0 <file>"
  exit 1
fi

input_file="$1"

if [ ! -f "${input_file}" ]; then
  echo "Error: File ${input_file} not found."
  exit 1
fi

parts=$(ls "${input_file}"_* 2>/dev/null)
if [ -n "$parts" ]; then
  echo "Error: Parts found for prefix ${input_file}_."
  exit 1
fi

echo "🔧 Splitting ${input_file} into 2GB chunks..."
split -b 2G -d --suffix-length=3 "${input_file}" "${input_file}_"
echo "✅ Done. Output files:"
ls -1 "${input_file}"_*

