#!/usr/bin/env python3
import filecmp
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
PAIRS = [
    (ROOT / "js/app.js", ROOT / "tone-tools/www/js/app.js"),
    (ROOT / "wasm/chime.js", ROOT / "tone-tools/www/wasm/chime.js"),
    (ROOT / "wasm/chime.wasm", ROOT / "tone-tools/www/wasm/chime.wasm"),
]


def main() -> int:
    missing = [str(a) for _, a in PAIRS if not a.exists()] + [str(b) for b, _ in PAIRS if not b.exists()]
    if missing:
        print("missing files:", ", ".join(missing), file=sys.stderr)
        return 1
    stale = []
    for src, dst in PAIRS:
        if not filecmp.cmp(src, dst, shallow=False):
            stale.append(f"{src.relative_to(ROOT)} != {dst.relative_to(ROOT)}")
    www_index = ROOT / "tone-tools/www/index.html"
    if not www_index.exists():
        stale.append("tone-tools/www/index.html missing")
    elif 'apiBase = "api"' not in www_index.read_text():
        stale.append("tone-tools/www/index.html missing HA ingress patch (apiBase)")
    if stale:
        print("www artifacts out of sync:", file=sys.stderr)
        for s in stale:
            print(f"  {s}", file=sys.stderr)
        print("Sync with: cp js/app.js tone-tools/www/js/app.js && cp wasm/chime.* tone-tools/www/wasm/", file=sys.stderr)
        return 1
    print("www artifacts in sync.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
