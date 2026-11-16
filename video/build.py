"""Build the narrated explainer video.

Run:  uv run python -m video.build
Output: reports/media/project_explainer.mp4 (+ .srt captions)

Pipeline: simulation clips (MuJoCo) -> narration (Windows voices, offline) -> slides drawn with PIL ->
frames written at 24 fps, every sentence lasting exactly as long as its audio -> ffmpeg joins picture and sound.
"""
from __future__ import annotations

import hashlib
import json
import math
import re
import subprocess
import sys
import wave
from pathlib import Path

import imageio.v2 as imageio
import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from arm import params as P
from arm.sim import run_cycle
from arm.task import plan_cycle
from .draw import ACC, BG, FONT_DIR, INK, JC, Canvas, H, W, caption_overlay
from .slides import build_slides

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "video" / "build"
AUDIO = BUILD / "audio"
OUT = ROOT / "reports" / "media" / "project_explainer.mp4"
FPS = 24
FRAME_DT = 42 * 0.001     # the simulator renders one frame every 42 steps of 1 ms
PAUSE = 0.30            # seconds of silence after each sentence
SLIDE_END_PAUSE = 0.55  # extra at the end of a slide
VOICE, RATE = "Microsoft David Desktop", 0
COLOURS = ("red", "blue")


# ------------------------------------------------------------------ simulation clips
SORT_LABELS = {"to above pick": "Moving above the pick spot", "descend": "Straight down to the object",
               "grip": "Closing the gripper", "lift": "Lifting the object", "lower": "Lowering into the bin",
               "release": "Opening the gripper", "retreat": "Retreating upward", "return to ready": "Back to the ready pose",
               "settle": "Back to the ready pose"}


def segment_label(name: str):
    m = re.match(r"axes check J(\d) (low|high|back)", name)
    if m:
        j = int(m.group(1))
        what = {"low": "towards its lower limit", "high": "towards its upper limit", "back": "back to the ready pose"}[m.group(2)]
        return f"J{j} {P.JOINTS[j - 1].name.split(' ', 1)[1]}: {what}", JC[j - 1]
    m = re.match(r"\d: to (\w+) bin", name)
    if m:
        return f"Swinging to the {m.group(1)} bin, turning the wrist", ACC
    key = name.split(": ", 1)[-1]
    return SORT_LABELS.get(key, key), ACC


def overlay_label(frame, title, label, colour):
    im = Image.fromarray(frame).convert("RGBA")
    ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    ft, fl = ImageFont.truetype(FONT_DIR + "segoeui.ttf", 20), ImageFont.truetype(FONT_DIR + "segoeuib.ttf", 28)
    d.rounded_rectangle([24, 20, 24 + max(ft.getlength(title), fl.getlength(label)) + 56, 112], radius=14, fill=(8, 10, 14, 215))
    d.rectangle([24, 20, 32, 112], fill=colour + (255,))
    d.text((48, 28), title, font=ft, fill=(190, 198, 210, 255))
    d.text((48, 60), label, font=fl, fill=(245, 247, 250, 255))
    return np.asarray(Image.alpha_composite(im, ov).convert("RGB"))


def render_clips(slides) -> dict:
    paths = {"check": BUILD / "clip_check.mp4", "sort": BUILD / "clip_sort.mp4"}
    if all(p.exists() for p in paths.values()) and "--rerender" not in sys.argv:
        return paths
    traj = plan_cycle(COLOURS)
    segs = traj.segments
    t_split = next(t0 for name, t0, t1 in segs if not name.startswith("axes check"))
    titles = {s.clip: f"{s.kicker}: {s.title}" for s in slides if s.clip}
    writers = {k: imageio.get_writer(p, fps=FPS, quality=8, macro_block_size=None) for k, p in paths.items()}

    def cb(t, frame):
        label, col = next(((segment_label(n)) for n, t0, t1 in segs if t0 <= t < t1), ("Back to the ready pose", ACC))
        key = "check" if t < t_split else "sort"
        writers[key].append_data(overlay_label(frame, titles[key], label, col))

    print("rendering simulation clips ...", flush=True)
    run_cycle(traj, COLOURS, render=True, fps=FPS, size=(720, 1280), camera="iso", frame_cb=cb)
    for w in writers.values():
        w.close()
    return paths


def clip_marks() -> dict:
    """For each clip slide: the clip time (seconds) at which each sentence should end."""
    traj = plan_cycle(COLOURS)
    segs = traj.segments
    t_split = next(t0 for name, t0, t1 in segs if not name.startswith("axes check"))
    end_of = lambda name: next(t1 for n, t0, t1 in segs if n == name)
    check = [end_of("axes check J1 back"), end_of("axes check J3 back"), t_split, t_split]
    sort_start = t_split
    attach = next(t for t, kind, i in traj.events if kind == "attach" and i == 0)
    end_obj1 = end_of("1: retreat")
    mid2 = end_of("2: to blue bin")
    total = traj.duration
    sort = [attach - sort_start, end_obj1 - sort_start, total - sort_start - 3.0, total - sort_start]
    return {"check": check, "sort": sort, "_split": t_split}


def render_still():
    import mujoco
    from arm.mjcf import build_xml
    m = mujoco.MjModel.from_xml_string(build_xml(COLOURS))
    d = mujoco.MjData(m)
    d.qpos[:6] = np.radians(P.READY_Q_DEG)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 700, 700)
    r.update_scene(d, camera="iso")
    im = Image.fromarray(r.render().copy())
    r.close()
    return im


# ------------------------------------------------------------------ narration
def pretty(s: str) -> str:
    nums = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6}
    s = re.sub(r"\bJ (one|two|three|four|five|six)\b", lambda m: f"J{nums[m.group(1)]}", s)
    for a, b in (("I two C", "I²C"), ("T M C twenty two oh nine", "TMC2209"), ("P C A nine six eight five", "PCA9685"),
                 ("the P C", "the PC"), ("USB", "USB")):
        s = s.replace(a, b)
    return s


def chunks(text: str, limit: int = 150) -> list:
    """Split a sentence into caption pieces of at most ~2 lines, preferring breaks after commas."""
    if len(text) <= limit:
        return [text]
    parts = re.split(r"(?<=[,:;.]) ", text)
    out, cur = [], ""
    for p in parts:
        if cur and len(cur) + 1 + len(p) > limit:
            out.append(cur)
            cur = p
        else:
            cur = (cur + " " + p).strip()
    out.append(cur)
    return out


def synthesize(sentences: list) -> list:
    AUDIO.mkdir(parents=True, exist_ok=True)
    items = []
    for text in sentences:
        h = hashlib.md5(f"{VOICE}|{RATE}|{text}".encode()).hexdigest()[:12]
        items.append({"path": str(AUDIO / f"{h}.wav"), "text": text})
    todo = [it for it in items if not Path(it["path"]).exists()]
    if todo:
        jf = BUILD / "tts_items.json"
        jf.write_text(json.dumps(todo), encoding="utf-8")
        print(f"synthesizing {len(todo)} sentences ...", flush=True)
        r = subprocess.run(["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(ROOT / "video" / "tts.ps1"),
                            "-Json", str(jf), "-Voice", VOICE, "-Rate", str(RATE)], capture_output=True, text=True)
        if r.returncode != 0:
            raise RuntimeError("text-to-speech failed: " + (r.stderr or r.stdout)[-500:])
    return [it["path"] for it in items]


def read_wav(path):
    with wave.open(path, "rb") as w:
        return w.getparams(), w.readframes(w.getnframes())


# ------------------------------------------------------------------ assembly
def fmt_srt(t):
    ms = int(round(t * 1000))
    return f"{ms // 3600000:02d}:{ms // 60000 % 60:02d}:{ms // 1000 % 60:02d},{ms % 1000:03d}"


def main():
    BUILD.mkdir(parents=True, exist_ok=True)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    slides = build_slides()
    clips = render_clips(slides)
    ctx = {"still": render_still()}
    marks = clip_marks()

    sentences = [s for sl in slides for s, _ in sl.lines]
    wavs = synthesize(sentences)
    params0, _ = read_wav(wavs[0])
    rate, width, ch = params0.framerate, params0.sampwidth, params0.nchannels
    silent = lambda sec: b"\x00" * (int(round(sec * rate)) * width * ch)

    writer = imageio.get_writer(BUILD / "silent.mp4", fps=FPS, quality=7, macro_block_size=None)
    audio_out = wave.open(str(BUILD / "narration.wav"), "wb")
    audio_out.setparams(params0)
    srt, t_now, wi, total_frames = [], 0.0, 0, 0
    cache = {}

    for si, sl in enumerate(slides):
        # frames per sentence
        plan = []
        for li, (text, _) in enumerate(sl.lines):
            p, data = read_wav(wavs[wi])
            wi += 1
            assert (p.framerate, p.sampwidth, p.nchannels) == (rate, width, ch), "voice format changed"
            dur = len(data) / (rate * width * ch)
            pause = PAUSE + (SLIDE_END_PAUSE if li == len(sl.lines) - 1 else 0)
            n = int(math.ceil((dur + pause) * FPS))
            pad = n / FPS - dur
            plan.append((text, data, dur, n, pad))
        n_slide = sum(p[3] for p in plan)
        reader = imageio.get_reader(clips[sl.clip]) if sl.clip else None
        n_clip = reader.count_frames() if reader else 0
        cur_i, cur_frame = -1, None
        frame_idx = 0
        for li, (text, data, dur, n, pad) in enumerate(plan):
            cap = pretty(text)
            pieces = chunks(cap)
            tot = sum(len(p) for p in pieces)
            bounds = np.cumsum([len(p) / tot for p in pieces]) * (dur * FPS)
            for fi in range(n):
                piece = next((i for i, b in enumerate(bounds) if fi < b), len(pieces) - 1)
                if sl.clip:
                    lo = 0.0 if li == 0 else marks[sl.clip][li - 1]
                    hi = marks[sl.clip][li]
                    t_clip = lo + (hi - lo) * (fi / n)
                    want = min(n_clip - 1, int(t_clip / FRAME_DT))
                    while cur_i < want:
                        cur_frame = reader.get_next_data()
                        cur_i += 1
                    frame = caption_overlay(cur_frame, pieces[piece])
                else:
                    key = (si, sl.steps[li], piece)
                    if key not in cache:
                        bkey = (si, sl.steps[li])
                        if bkey not in cache:
                            c = Canvas()
                            c.title(sl.title, sl.kicker)
                            sl.draw(c, sl.steps[li], ctx)
                            cache[bkey] = c.frame()
                        cache[key] = caption_overlay(cache[bkey], pieces[piece])
                    frame = cache[key]
                writer.append_data(frame)
                frame_idx += 1
            audio_out.writeframes(data + silent(pad))
            srt.append((t_now, t_now + dur, cap))
            t_now += n / FPS
            total_frames += n
        if reader:
            reader.close()
        print(f"  slide {si + 1}/{len(slides)} {sl.key}: {n_slide / FPS:.1f} s", flush=True)
    writer.close()
    audio_out.close()

    with open(OUT.with_suffix(".srt"), "w", encoding="utf-8") as f:
        for i, (a, b, t) in enumerate(srt, 1):
            f.write(f"{i}\n{fmt_srt(a)} --> {fmt_srt(b)}\n{t}\n\n")
    ff = imageio_ffmpeg.get_ffmpeg_exe()
    r = subprocess.run([ff, "-y", "-loglevel", "error", "-i", str(BUILD / "silent.mp4"), "-i", str(BUILD / "narration.wav"),
                        "-c:v", "copy", "-c:a", "aac", "-b:a", "128k", "-movflags", "+faststart", "-shortest", str(OUT)],
                       capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(r.stderr)
    print(f"done: {OUT.relative_to(ROOT)}  {total_frames / FPS / 60:.1f} min, {OUT.stat().st_size / 1e6:.0f} MB", flush=True)


if __name__ == "__main__":
    main()
