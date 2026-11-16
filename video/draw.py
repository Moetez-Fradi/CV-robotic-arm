"""Small drawing toolkit for the explainer video: a 1280x720 canvas drawn at 2x and scaled down."""
from __future__ import annotations

import math
from functools import lru_cache

import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H, SS = 1280, 720, 2
BG = (18, 21, 27)
PANEL = (27, 32, 41)
PANEL2 = (36, 42, 54)
LINE = (62, 71, 89)
INK = (231, 234, 240)
MUTED = (154, 165, 181)
ACC = (240, 138, 60)
OK = (95, 208, 138)
BAD = (255, 120, 110)
BLUE = (122, 167, 255)
YELLOW = (240, 200, 80)
JC = [(31, 119, 180), (214, 39, 40), (44, 160, 44), (148, 103, 189), (255, 127, 14), (140, 86, 75)]
JC = [tuple(min(255, int(c * 0.75 + 70)) for c in col) for col in JC]   # lighter for the dark background
FONT_DIR = "C:/Windows/Fonts/"


@lru_cache(maxsize=None)
def font(size: int, bold: bool = False, mono: bool = False) -> ImageFont.FreeTypeFont:
    name = "consola.ttf" if mono else ("segoeuib.ttf" if bold else "segoeui.ttf")
    return ImageFont.truetype(FONT_DIR + name, int(size * SS))


class Canvas:
    def __init__(self, bg=BG):
        self.img = Image.new("RGB", (W * SS, H * SS), bg)
        self.d = ImageDraw.Draw(self.img)

    # ---- primitives (all coordinates in 1280x720 space)
    @staticmethod
    def s(v):
        return int(round(v * SS))

    def pts(self, pts):
        return [(self.s(x), self.s(y)) for x, y in pts]

    def rect(self, x, y, w, h, fill=None, outline=None, r=10, width=2):
        self.d.rounded_rectangle([self.s(x), self.s(y), self.s(x + w), self.s(y + h)], radius=self.s(r), fill=fill,
                                 outline=outline, width=self.s(width) if outline else 0)

    def line(self, p1, p2, color=LINE, width=3):
        self.d.line(self.pts([p1, p2]), fill=color, width=self.s(width))

    def poly(self, pts, color=LINE, width=3, closed=False, fill=None):
        if fill is not None:
            self.d.polygon(self.pts(pts), fill=fill)
        p = self.pts(pts + ([pts[0]] if closed else []))
        self.d.line(p, fill=color, width=self.s(width), joint="curve")

    def circle(self, cx, cy, r, fill=None, outline=None, width=2):
        self.d.ellipse([self.s(cx - r), self.s(cy - r), self.s(cx + r), self.s(cy + r)], fill=fill, outline=outline,
                       width=self.s(width) if outline else 0)

    def arrow(self, p1, p2, color=MUTED, width=3, head=12):
        self.line(p1, p2, color, width)
        a = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        for da in (2.6, -2.6):
            self.line(p2, (p2[0] + head * math.cos(a + da), p2[1] + head * math.sin(a + da)), color, width)

    def arc(self, cx, cy, r, a0, a1, color=ACC, width=3, head=True):
        """Arc in degrees (0 = right, counter-clockwise on screen), with an arrow head at the end."""
        pts = [(cx + r * math.cos(math.radians(a)), cy - r * math.sin(math.radians(a)))
               for a in np.linspace(a0, a1, 40)]
        self.poly(pts, color, width)
        if head:
            tip, prev = pts[-1], pts[-3]
            self.arrow(prev, tip, color, width, 11)

    def text(self, x, y, s, size=20, color=INK, bold=False, mono=False, anchor="la", maxw=None, spacing=1.25):
        f = font(size, bold, mono)
        if maxw:
            lines = self.wrap(s, f, maxw)
        else:
            lines = s.split("\n")
        lh = size * spacing
        for i, ln in enumerate(lines):
            self.d.text((self.s(x), self.s(y + i * lh)), ln, font=f, fill=color, anchor=anchor)
        return len(lines) * lh

    def wrap(self, s, f, maxw):
        out = []
        for para in s.split("\n"):
            cur = ""
            for word in para.split(" "):
                t = (cur + " " + word).strip()
                if f.getlength(t) <= maxw * SS or not cur:
                    cur = t
                else:
                    out.append(cur)
                    cur = word
            out.append(cur)
        return out

    def box(self, x, y, w, h, title, sub=None, fill=PANEL, outline=LINE, tcolor=INK, tsize=20, ssize=15, accent=None):
        self.rect(x, y, w, h, fill, outline, 12, 2)
        if accent:
            self.rect(x, y, 8, h, accent, None, 4)
        ty = y + (h - (tsize * 1.2 + (ssize * 1.3 * (sub.count("\n") + 1) if sub else 0))) / 2
        self.text(x + w / 2, ty, title, tsize, tcolor, True, anchor="ma")
        if sub:
            self.text(x + w / 2, ty + tsize * 1.3, sub, ssize, MUTED, anchor="ma", spacing=1.3)

    def paste(self, im: Image.Image, x, y, w=None, h=None):
        if w and not h:
            h = int(im.height * w / im.width)
        if h and not w:
            w = int(im.width * h / im.height)
        if w:
            im = im.resize((self.s(w), self.s(h)), Image.LANCZOS)
        self.img.paste(im, (self.s(x), self.s(y)))
        return w, h

    def title(self, text, kicker=None):
        if kicker:
            self.text(48, 28, kicker.upper(), 14, ACC, True)
        self.text(48, 44, text, 34, INK, True)
        self.line((48, 96), (W - 48, 96), LINE, 2)

    def frame(self) -> np.ndarray:
        return np.asarray(self.img.resize((W, H), Image.LANCZOS))

    def pil(self) -> Image.Image:
        return self.img.resize((W, H), Image.LANCZOS)


def caption_overlay(frame: np.ndarray, text: str, size: int = 22) -> np.ndarray:
    """Burn a subtitle into the bottom of an RGB frame (1280x720)."""
    im = Image.fromarray(frame).convert("RGBA")
    ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    f = ImageFont.truetype(FONT_DIR + "segoeui.ttf", size)
    lines, cur = [], ""
    for word in text.split(" "):
        t = (cur + " " + word).strip()
        if f.getlength(t) <= W - 200 or not cur:
            cur = t
        else:
            lines.append(cur)
            cur = word
    lines.append(cur)
    lh = int(size * 1.3)
    h = lh * len(lines) + 24
    y0 = H - h - 18
    d.rounded_rectangle([60, y0, W - 60, y0 + h], radius=14, fill=(8, 10, 14, 215))
    for i, ln in enumerate(lines):
        d.text((W / 2, y0 + 12 + i * lh), ln, font=f, fill=(245, 247, 250, 255), anchor="ma")
    return np.asarray(Image.alpha_composite(im, ov).convert("RGB"))
