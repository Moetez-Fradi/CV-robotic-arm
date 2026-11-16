"""Shared helpers for the experiment scripts: output paths, plot style, tables."""
from __future__ import annotations

import json
from datetime import date
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
REPORTS = ROOT / "reports"
FIGS = REPORTS / "figures"
MEDIA = REPORTS / "media"
DATA = REPORTS / "data"
for _d in (FIGS, MEDIA, DATA):
    _d.mkdir(parents=True, exist_ok=True)

plt.rcParams.update({
    "figure.dpi": 110, "savefig.dpi": 150, "savefig.bbox": "tight",
    "axes.grid": True, "grid.alpha": 0.3, "axes.spines.top": False, "axes.spines.right": False,
    "font.size": 9, "axes.titlesize": 10, "legend.fontsize": 8,
})
JOINT_COLOURS = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e", "#8c564b"]

ASSUMPTION_NOTE = (
    "> **Status of the inputs.** Link lengths, part masses and the cell layout are "
    "placeholders from `arm/params.py`, set before the CAD existed. Motor data other than "
    "the spec-book figures are typical catalogue values. The method and the scripts are "
    "final; the numbers must be regenerated (`uv run python -m experiments.run_all`) once "
    "the CAD values are entered.\n"
)


def savefig(fig, name: str) -> str:
    fig.savefig(FIGS / name)
    plt.close(fig)
    return f"figures/{name}"


def table(headers, rows) -> str:
    def fmt(x):
        return f"{x:.3g}" if isinstance(x, float) else str(x)
    out = ["| " + " | ".join(headers) + " |", "|" + "---|" * len(headers)]
    out += ["| " + " | ".join(fmt(c) for c in r) + " |" for r in rows]
    return "\n".join(out) + "\n"


def verdict(ok: bool) -> str:
    return "PASS" if ok else "**FAIL**"


def write_report(name: str, text: str) -> Path:
    path = REPORTS / name
    path.write_text(text.rstrip() + f"\n\n---\n*Generated {date.today().isoformat()} by the script named in the header.*\n",
                    encoding="utf-8")
    print(f"wrote {path.relative_to(ROOT)}")
    return path


def save_json(name: str, obj) -> None:
    (DATA / name).write_text(json.dumps(obj, indent=2), encoding="utf-8")


def load_json(name: str):
    p = DATA / name
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else None
