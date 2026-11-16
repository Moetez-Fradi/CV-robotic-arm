"""Local dashboard: edit the design values, run the experiments, read the reports.

Run:  uv run python -m dashboard        then open http://127.0.0.1:8765

Standard library only. Listens on localhost only. Experiments run as a subprocess of
this server (`python -m experiments.run_all ...`), one job at a time.
"""
from __future__ import annotations

import html
import json
import mimetypes
import re
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from arm import config  # noqa: E402

HOST, PORT = "127.0.0.1", 8765
REPORTS = ROOT / "reports"
INDEX = Path(__file__).resolve().parent / "index.html"
EXPERIMENTS = {
    "e1": ("Kinematics and workspace", "E1_kinematics_workspace.md", 10),
    "e2": ("Torque and gearing", "E2_torque_gearing.md", 40),
    "e3": ("Motion timing", "E3_motion_timing.md", 5),
    "e4": ("Pick-and-sort simulation", "E4_pick_place_simulation.md", 30),
    "e5": ("Power budget", "E5_power_budget.md", 1),
}


# --------------------------------------------------------------------------
# job runner
# --------------------------------------------------------------------------
class Job:
    def __init__(self):
        self.lock = threading.Lock()
        self.proc = None
        self.log: list[str] = []
        self.names: list[str] = []
        self.returncode = None

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def start(self, names):
        with self.lock:
            if self.running():
                raise RuntimeError("A run is already in progress")
            self.log, self.names, self.returncode = [], list(names), None
            self.proc = subprocess.Popen(
                [sys.executable, "-u", "-m", "experiments.run_all", *names], cwd=ROOT,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace")
            threading.Thread(target=self._pump, args=(self.proc,), daemon=True).start()

    def _pump(self, proc):
        for line in proc.stdout:
            self.log.append(line.rstrip("\n"))
        self.returncode = proc.wait()

    def state(self):
        done = [m.group(1) for line in self.log if (m := re.match(r"\[(e\d)\] (ok|failed)", line))]
        current = next((m.group(1) for line in reversed(self.log) if (m := re.match(r"\[(e\d)\] started", line))), None)
        return {"running": self.running(), "names": self.names, "done": done,
                "current": current if self.running() else None,
                "returncode": self.returncode, "log": self.log[-400:]}


JOB = Job()


# --------------------------------------------------------------------------
# minimal Markdown -> HTML for the generated reports
# --------------------------------------------------------------------------
def _inline(text: str, bust: str) -> str:
    t = html.escape(text, quote=False)
    codes = []

    def stash(m):
        codes.append(m.group(1))
        return f"\x00{len(codes) - 1}\x00"

    t = re.sub(r"`([^`]+)`", stash, t)

    def image(m):
        src = m.group(2)
        if src.endswith((".mp4", ".webm")):
            return f'<video controls loop muted src="/reports/{src}?v={bust}"></video>'
        return f'<img alt="{html.escape(m.group(1))}" src="/reports/{src}?v={bust}" loading="lazy">'

    t = re.sub(r"!\[([^\]]*)\]\(([^)]+)\)", image, t)
    t = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", lambda m: f'<a href="/reports/{m.group(2)}" target="_blank" rel="noopener">{m.group(1)}</a>', t)
    t = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", t)
    t = re.sub(r"(?<![\w*])\*([^*\s][^*]*)\*(?![\w*])", r"<em>\1</em>", t)
    return re.sub(r"\x00(\d+)\x00", lambda m: f"<code>{codes[int(m.group(1))]}</code>", t)


def markdown_to_html(md: str, bust: str) -> str:
    out, lines, i = [], md.splitlines(), 0
    while i < len(lines):
        line = lines[i]
        if not line.strip():
            i += 1
        elif line.startswith("```"):
            i += 1
            code = []
            while i < len(lines) and not lines[i].startswith("```"):
                code.append(lines[i])
                i += 1
            i += 1
            out.append("<pre>" + html.escape("\n".join(code)) + "</pre>")
        elif m := re.match(r"(#{1,4}) (.*)", line):
            n = len(m.group(1))
            out.append(f"<h{n}>{_inline(m.group(2), bust)}</h{n}>")
            i += 1
        elif line.startswith("|"):
            rows = []
            while i < len(lines) and lines[i].startswith("|"):
                rows.append([c.strip() for c in lines[i].strip().strip("|").split("|")])
                i += 1
            body = [r for k, r in enumerate(rows) if k != 1]
            cells = ["<tr>" + "".join(f"<{'th' if k == 0 else 'td'}>{_inline(c, bust)}</{'th' if k == 0 else 'td'}>" for c in r) + "</tr>"
                     for k, r in enumerate(body)]
            out.append('<div class="tablewrap"><table>' + "".join(cells) + "</table></div>")
        elif line.startswith(">"):
            quote = []
            while i < len(lines) and lines[i].startswith(">"):
                quote.append(lines[i].lstrip("> "))
                i += 1
            out.append("<blockquote>" + _inline(" ".join(quote), bust) + "</blockquote>")
        elif re.match(r"-{3,}\s*$", line):
            out.append("<hr>")
            i += 1
        elif re.match(r"\s*(\*|\d+\.) ", line):
            ordered = bool(re.match(r"\s*\d+\. ", line))
            items = []
            while i < len(lines) and lines[i].strip() and (re.match(r"\s*(\*|\d+\.) ", lines[i]) or lines[i].startswith(" ")):
                if re.match(r"\s*(\*|\d+\.) ", lines[i]):
                    items.append(re.sub(r"^\s*(\*|\d+\.) ", "", lines[i]))
                else:
                    items[-1] += " " + lines[i].strip()
                i += 1
            tag = "ol" if ordered else "ul"
            out.append(f"<{tag}>" + "".join(f"<li>{_inline(it, bust)}</li>" for it in items) + f"</{tag}>")
        else:
            para = []
            while i < len(lines) and lines[i].strip() and not re.match(r"(#{1,4} |\||>|```|\s*(\*|\d+\.) )", lines[i]):
                para.append(lines[i].strip())
                i += 1
            out.append("<p>" + _inline(" ".join(para), bust) + "</p>")
    text = "\n".join(out)
    text = text.replace("<td>PASS</td>", '<td><span class="pass">PASS</span></td>')
    text = text.replace("<strong>FAIL</strong>", '<span class="fail">FAIL</span>')
    return re.sub(r"(Verdict:|verdict:) PASS", r'\1 <span class="pass">PASS</span>', text)


# --------------------------------------------------------------------------
# HTTP
# --------------------------------------------------------------------------
def read_json(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def app_state():
    cfg = config.load()
    return {
        "config": cfg, "defaults": config.DEFAULTS, "hash": config.config_hash(cfg),
        "saved_file": config.CONFIG_PATH.exists(),
        "labels": config.LABELS, "sections": config.SECTION_TITLES, "joint_names": config.JOINT_NAMES,
        "experiments": {k: {"title": v[0], "seconds": v[2], "has_report": (REPORTS / v[1]).exists()} for k, v in EXPERIMENTS.items()},
        "meta": read_json(REPORTS / "data" / "run_meta.json") or {},
        "results": {k: read_json(REPORTS / "data" / f"{k}.json") for k in EXPERIMENTS},
        "job": JOB.state(),
    }


class Handler(BaseHTTPRequestHandler):
    server_version = "ArmDashboard/1.0"

    def log_message(self, *args):  # keep the terminal quiet
        pass

    def _send(self, code, body, ctype="application/json"):
        data = body if isinstance(body, bytes) else (json.dumps(body) if ctype == "application/json" else body).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype + ("; charset=utf-8" if ctype.startswith(("text/", "application/json")) else ""))
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _local_only(self) -> bool:
        """Refuse requests that a web page on another site could have triggered."""
        host = (self.headers.get("Host") or "").split(":")[0]
        origin = self.headers.get("Origin")
        ok_host = host in ("127.0.0.1", "localhost")
        ok_origin = origin is None or urlparse(origin).hostname in ("127.0.0.1", "localhost")
        if not (ok_host and ok_origin):
            self._send(403, {"error": "local requests only"})
        return ok_host and ok_origin

    def do_GET(self):
        if not self._local_only():
            return
        url = urlparse(self.path)
        if url.path in ("/", "/index.html"):
            return self._send(200, INDEX.read_bytes(), "text/html")
        if url.path == "/model.js":
            return self._send(200, (Path(__file__).resolve().parent / "model.js").read_bytes(), "text/javascript")
        if url.path == "/api/state":
            return self._send(200, app_state())
        if url.path == "/api/model":
            r = subprocess.run([sys.executable, "-m", "arm.describe"], cwd=ROOT, capture_output=True, text=True,
                               encoding="utf-8", errors="replace", timeout=60)
            if r.returncode != 0:
                return self._send(500, {"error": (r.stderr.strip().splitlines() or ["model description failed"])[-1]})
            return self._send(200, r.stdout.encode("utf-8"))
        if url.path == "/api/job":
            return self._send(200, JOB.state())
        if url.path == "/api/report":
            name = parse_qs(url.query).get("name", [""])[0]
            if name not in EXPERIMENTS:
                return self._send(404, {"error": "unknown report"})
            path = REPORTS / EXPERIMENTS[name][1]
            if not path.exists():
                return self._send(200, {"html": "<p>This experiment has not been run yet.</p>"})
            return self._send(200, {"html": markdown_to_html(path.read_text(encoding="utf-8"), str(int(path.stat().st_mtime)))})
        if url.path.startswith("/reports/"):
            target = (REPORTS / url.path[len("/reports/"):]).resolve()
            if REPORTS.resolve() not in target.parents or not target.is_file():
                return self._send(404, {"error": "not found"})
            return self._send(200, target.read_bytes(), mimetypes.guess_type(target.name)[0] or "application/octet-stream")
        self._send(404, {"error": "not found"})

    def do_POST(self):
        if not self._local_only():
            return
        length = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(length) or b"{}")
        except ValueError:
            return self._send(400, {"error": "invalid JSON"})
        try:
            if self.path == "/api/config":
                config.save(body)
            elif self.path == "/api/reset":
                config.reset()
            elif self.path == "/api/run":
                names = [n for n in body.get("names", []) if n in EXPERIMENTS]
                if not names:
                    raise ValueError("Select at least one experiment")
                if "config" in body:
                    config.save(body["config"])
                JOB.start(names)
            else:
                return self._send(404, {"error": "not found"})
        except (ValueError, RuntimeError) as exc:
            return self._send(400, {"error": str(exc)})
        self._send(200, app_state())


def main():
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"Arm dashboard running at http://{HOST}:{PORT}  (Ctrl+C to stop)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
