#!/usr/bin/env bash
# One-command bring-up per scope §11 (< 45 min target)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

if [[ ! -d .venv ]]; then
  python3 -m venv .venv
fi
# shellcheck disable=SC1091
source .venv/bin/activate
pip install -q -e ".[dev]"

python scripts/build_assets.py
python scripts/validate.py
python scripts/run_hover.py --vehicle AER8110-1 --seconds 3
python scripts/run_hover.py --vehicle AER8110-1 --seconds 3 --articulation-deg 15

echo ""
echo "Bring-up complete. USD assets in assets/ | report: validation_report.json"
