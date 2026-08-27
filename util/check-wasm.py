#!/usr/bin/env python3
"""Check that wasm/chime.{js,wasm} are up-to-date with core sources.

Compares mtimes: if any source in esphome/components/chime/ or util/bindings.cpp
is newer than wasm/chime.js or wasm/chime.wasm, exits 1 with instruction to rebuild.

Used as pre-commit hook (repo: local, id: wasm-stale).
"""

import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]

SOURCES = [
    ROOT / "esphome/components/chime/chime_engine.cpp",
    ROOT / "esphome/components/chime/chime_engine.h",
    ROOT / "esphome/components/chime/chime_pattern.cpp",
    ROOT / "esphome/components/chime/chime_pattern.h",
    ROOT / "esphome/components/chime/chime_types.h",
    ROOT / "util/bindings.cpp",
    ROOT / "util/Makefile",
]

ARTIFACTS = [
    ROOT / "wasm/chime.js",
    ROOT / "wasm/chime.wasm",
]


def main() -> int:
    missing = [str(p) for p in SOURCES + ARTIFACTS if not p.exists()]
    if missing:
        # Artifacts may not exist on fresh clone — warn but don't fail if sources also missing?
        if any(str(a) in missing for a in ARTIFACTS):
            print("wasm artifacts missing:", ", ".join(missing), file=sys.stderr)
            print("Run: make -C util", file=sys.stderr)
            return 1
        print("Missing sources (unexpected):", ", ".join(missing), file=sys.stderr)
        return 1

    src_mtime = max(p.stat().st_mtime for p in SOURCES)
    art_mtime = min(p.stat().st_mtime for p in ARTIFACTS)

    if src_mtime > art_mtime:
        print("WASM artifacts are stale.", file=sys.stderr)
        newest_src = max(SOURCES, key=lambda p: p.stat().st_mtime)
        print(
            f"  Newest source: {newest_src} ({newest_src.stat().st_mtime})",
            file=sys.stderr,
        )
        oldest_art = min(ARTIFACTS, key=lambda p: p.stat().st_mtime)
        print(
            f"  Oldest artifact: {oldest_art} ({oldest_art.stat().st_mtime})",
            file=sys.stderr,
        )
        print("  Rebuild with: make -C util", file=sys.stderr)
        print("  Or: emsdk install 3.1.6 && make -C util", file=sys.stderr)
        return 1

    print("WASM artifacts up-to-date.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
