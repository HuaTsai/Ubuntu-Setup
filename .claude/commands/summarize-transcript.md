---
allowed-tools: Read(*), Write(*)
description: Summarize and organize transcript files with key content
---

## Your task

The user will provide a transcript file path as: `/summarize-transcript <file-path>`
The file path is available as `$ARGUMENTS`.

Organize and summarize transcript content by:

1. Read the transcript file at `$ARGUMENTS` using the Read tool
2. Analyze and extract key information:
   - Main topics discussed
   - Key points and takeaways
   - Important decisions or conclusions
   - Action items (if any)
3. Structure the content in a clear, organized format with:
   - A brief summary at the top
   - Organized sections by topic
   - Bullet points for key information
4. Overwrite the original file with the organized content using the Write tool

## Output Format

The organized transcript should follow this structure:

```markdown
# Transcript Summary

## Overview

[Brief 2-3 sentence summary of the main content]

## Key Topics

### [Topic 1]

- Key point 1
- Key point 2

### [Topic 2]

- Key point 1
- Key point 2

## Main Takeaways

- Takeaway 1
- Takeaway 2

## Action Items

- [ ] Action item 1
- [ ] Action item 2
```

## Best Practices

- **IMPORTANT: Output all content in Traditional Chinese (繁體中文)**
- Keep the summary concise but informative
- Group related content together
- Use clear headings and bullet points
- Preserve important details while removing filler
- Maintain the original meaning and context
