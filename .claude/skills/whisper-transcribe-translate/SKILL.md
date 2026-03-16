---
name: whisper-transcribe-translate
description: Transcribe audio files using local Whisper model, then translate the result to Traditional Chinese using Claude directly in the CLI. Use when the user has audio/video files and wants Japanese (or other language) transcription + Chinese translation written to output files with timestamps.
---

# Whisper Transcribe & Translate

Transcribes audio files with a local faster-whisper model, then Claude translates the segments directly — no external translation API needed.

## Step 0: Environment Setup (if needed)

Before running, check whether faster-whisper is available.

### Detect existing venv

Look for a `.venv` in the current project or any parent directory. Test it:

```bash
.venv/bin/python -c "import faster_whisper; print('ok')"
```

### Create environment with uv (if missing)

```bash
uv venv .venv --python 3.11
uv pip install faster-whisper --python .venv/bin/python
```

> Use Python 3.11 or 3.12. Avoid 3.10 — some dependencies (e.g. onnxruntime) only ship wheels for 3.11+.

### Download a Whisper model (if missing)

Models are cached automatically on first use via Hugging Face. To pre-download explicitly:

```bash
.venv/bin/python -c "
from faster_whisper import WhisperModel
WhisperModel('large-v3', device='cpu', compute_type='int8')
print('Model ready')
"
```

Available model sizes (trade-off speed vs. accuracy):

| Model     | Size  | Notes                        |
|-----------|-------|------------------------------|
| `tiny`    | 75 MB | Fast, lower accuracy         |
| `base`    | 145 MB| Good for quick tests         |
| `small`   | 466 MB| Balanced                     |
| `medium`  | 1.5 GB| Good accuracy                |
| `large-v3`| 3 GB  | Best accuracy, recommended   |

---

## Workflow

### Step 1: Transcribe with Whisper

Write a temporary Python script that transcribes all audio files and outputs JSON to stdout:

```python
from pathlib import Path
from faster_whisper import WhisperModel
import json, sys

def format_time(seconds):
    m = int(seconds // 60)
    s = seconds % 60
    return f"{m:02d}:{s:05.2f}"

model = WhisperModel("large-v3", device="cpu", compute_type="int8")
result = {}

for audio_path in sorted(Path(".").glob("*.mp3")):  # adjust glob as needed
    print(f"[Transcribing] {audio_path.name}", file=sys.stderr)
    segments, info = model.transcribe(str(audio_path), language="ja", beam_size=5, vad_filter=True)
    result[audio_path.name] = [
        {"start": format_time(s.start), "end": format_time(s.end), "text": s.text.strip()}
        for s in segments
    ]

print(json.dumps(result, ensure_ascii=False, indent=2))
```

Run it:

```bash
.venv/bin/python transcribe_only.py > /tmp/result.json 2>/tmp/progress.txt
```

Key parameters:
- `language="ja"` — set to target language; omit to auto-detect
- `vad_filter=True` — skip silence
- `beam_size=5` — balance speed/accuracy

**Important**: always use `glob` to discover files — never hardcode filenames with Japanese/special characters, as encoding issues will cause `FileNotFoundError`.

### Step 2: Claude Translates

Read the JSON output and translate each segment's `text` field into Traditional Chinese. Claude handles this directly in the CLI — no API key or external service required.

Tips:
- Translate segment-by-segment, preserving brevity for short fragments (「え？」→「欸？」)
- Keep character names consistent throughout (e.g. アマミヤ → 天宮)
- Context carries across segments — read a few lines ahead when a fragment is ambiguous

### Step 3: Write Output Files

For each audio file, write a `<stem>_zh.txt` with one line per segment:

```
[MM:SS.ss-MM:SS.ss] Chinese translation
```

Optionally also write `<stem>_ja.txt` with the original Japanese in the same format.

---

## Tips

- **Short segments**: Japanese voice content often produces single-word segments — preserve them as-is, don't merge
- **Repeated text**: faster-whisper occasionally repeats a phrase; check the progress log for obvious duplicates
- **Other audio formats**: change the glob pattern (`*.wav`, `*.m4a`, `*.mp4`, etc.) as needed
- **GPU**: if CUDA is available, switch `device="cpu"` → `device="cuda"` and `compute_type="int8"` → `"float16"` for significant speedup
