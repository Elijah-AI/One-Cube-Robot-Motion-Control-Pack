#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 scenario_palletizing_rack_transfer.md 导出为 PDF（A4 纵向）。

依赖：
    - matplotlib（已安装 3.5.1）
    - 系统中文字体（微软雅黑 msyh.ttc / msyhbd.ttc）

用法：
    python3 task_manager/docs/export_scenario_palletizing_to_pdf.py

输出：
    task_manager/docs/scenario_palletizing_rack_transfer.pdf
"""

import glob
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Rectangle

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MD_FILE = os.path.join(BASE_DIR, "scenario_palletizing_rack_transfer.md")
PDF_FILE = os.path.join(BASE_DIR, "scenario_palletizing_rack_transfer.pdf")

# ---------------- 页面常量（A4，pt） ----------------
PAGE_W, PAGE_H = 595.0, 842.0
MARGIN_L, MARGIN_R = 50.0, 50.0
MARGIN_T, MARGIN_B = 56.0, 42.0
USABLE_W = PAGE_W - MARGIN_L - MARGIN_R


# ---------------- 字体 ----------------
def setup_fonts():
    files = []
    for root in ("/usr/share/fonts", os.path.expanduser("~/.local/share/fonts")):
        files += glob.glob(os.path.join(root, "**", "*"), recursive=True)
    font_files = [f for f in files if f.lower().endswith((".ttf", ".ttc", ".otf"))]
    for f in font_files:
        try:
            font_manager.fontManager.addfont(f)
        except Exception:
            pass
    regular = font_manager.FontProperties(family="Microsoft YaHei")
    bold_file = next((f for f in font_files
                      if "bd" in os.path.basename(f).lower()
                      and ("yahei" in os.path.basename(f).lower() or "msyh" in os.path.basename(f).lower())), None)
    return regular, bold_file


REGULAR, BOLD_FILE = setup_fonts()


def make_fp(size, bold=False):
    if bold and BOLD_FILE:
        p = font_manager.FontProperties(fname=BOLD_FILE, size=size)
        return p
    p = REGULAR.copy()
    p.set_size(size)
    if bold:
        p.set_weight("bold")
    return p


# ---------------- Markdown 解析 ----------------
def clean(text):
    text = re.sub(r"\[([^\]]*)\]\(([^)]*)\)", r"\1", text)
    text = text.replace("**", "").replace("`", "")
    return text.strip()


def split_row(line):
    parts = line.strip().split("|")
    return [p.strip() for p in parts if p.strip() != ""]


def parse_blocks(lines):
    blocks = []
    i, n = 0, len(lines)
    while i < n:
        s = lines[i].strip()
        if not s or re.match(r"^-{3,}$", s):
            i += 1
            continue
        m = re.match(r"^(#{1,6})\s+(.*)$", s)
        if m:
            blocks.append(("h", len(m.group(1)), clean(m.group(2))))
            i += 1
            continue
        if s.startswith("```"):
            code = []
            i += 1
            while i < n and not lines[i].strip().startswith("```"):
                code.append(lines[i])
                i += 1
            i += 1
            blocks.append(("code", "\n".join(code)))
            continue
        if s.startswith("|"):
            header = split_row(lines[i])
            j = i + 1
            while j < n and re.match(r"^\|[\s:|-]*\|?$", lines[j].strip()):
                j += 1
            rows = []
            while j < n and lines[j].strip().startswith("|"):
                rows.append(split_row(lines[j]))
                j += 1
            blocks.append(("table", header, rows))
            i = j
            continue
        if s.startswith(">"):
            blocks.append(("note", clean(s.lstrip(">").strip())))
            i += 1
            continue
        if s.startswith("- ") or re.match(r"^\d+\.\s", s):
            items = []
            while i < n:
                t = lines[i].strip()
                if t.startswith("- "):
                    items.append(clean(t[2:]))
                elif re.match(r"^\d+\.\s", t):
                    items.append(clean(re.sub(r"^\d+\.\s", "", t)))
                else:
                    break
                i += 1
            blocks.append(("list", items))
            continue
        paras = []
        while i < n:
            t = lines[i].strip()
            if (not t or t.startswith("|") or t.startswith("#") or t.startswith(">")
                    or t.startswith("- ") or t.startswith("```")
                    or re.match(r"^\d+\.\s", t) or re.match(r"^-{3,}$", t)):
                break
            paras.append(clean(t))
            i += 1
        if paras:
            blocks.append(("p", " ".join(paras)))
    return blocks


# ---------------- 换行 ----------------
def is_wide(ch):
    o = ord(ch)
    if o < 0x1100:
        return False
    if (0x1100 <= o <= 0x115F or 0x2E80 <= o <= 0x303E or 0x3041 <= o <= 0x33FF
            or 0x3400 <= o <= 0x4DBF or 0x4E00 <= o <= 0x9FFF or 0xA000 <= o <= 0xA4CF
            or 0xAC00 <= o <= 0xD7A3 or 0xF900 <= o <= 0xFAFF or 0xFE30 <= o <= 0xFE4F
            or 0xFF00 <= o <= 0xFF60 or 0xFFE0 <= o <= 0xFFE6):
        return True
    return False


def w_units(ch):
    return 2 if is_wide(ch) else 1


def wrap_text(text, max_units):
    if not text:
        return [""]
    lines, cur, curu = [], "", 0
    for ch in text:
        u = w_units(ch)
        if curu + u > max_units and cur:
            lines.append(cur)
            cur, curu = "", 0
        cur += ch
        curu += u
    if cur:
        lines.append(cur)
    return lines


def capacity_for(pt, size):
    return max(int(pt / (0.55 * size)), 4)


# ---------------- 渲染器 ----------------
class Renderer(object):
    def __init__(self, pdf):
        self.pdf = pdf
        self.fig = None
        self.ax = None
        self.y = PAGE_H - MARGIN_T
        self.page_no = 0
        self.new_page()

    def new_page(self):
        if self.fig is not None:
            self.pdf.savefig(self.fig)
            plt.close(self.fig)
        self.page_no += 1
        self.fig = plt.figure(figsize=(PAGE_W / 72.0, PAGE_H / 72.0), dpi=72)
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_xlim(0, PAGE_W)
        self.ax.set_ylim(0, PAGE_H)
        self.ax.axis("off")
        # matplotlib 的 y 轴向上增长，起始 y 取页面顶部，向下递减绘制
        self.y = PAGE_H - MARGIN_T
        # 页脚页码
        self.ax.text(PAGE_W / 2.0, 18, "第 %d 页" % self.page_no,
                     fontproperties=make_fp(8), color="#888888",
                     va="center", ha="center", clip_on=False)

    def finish(self):
        if self.fig is not None:
            self.pdf.savefig(self.fig)
            plt.close(self.fig)

    def ensure(self, h):
        if self.y - h < MARGIN_B:
            self.new_page()

    def put(self, x, y, s, fp, color="black"):
        self.ax.text(x, y, s, fontproperties=fp, color=color,
                     va="top", ha="left", clip_on=False)

    # ---------- 分派 ----------
    def draw(self, block):
        kind = block[0]
        if kind == "h":
            self.draw_h(block[1], block[2])
        elif kind == "p":
            self.draw_p(block[1])
        elif kind == "note":
            self.draw_note(block[1])
        elif kind == "list":
            self.draw_list(block[1])
        elif kind == "code":
            self.draw_code(block[1])
        elif kind == "table":
            self.draw_table(block[1], block[2])

    # ---------- 各块 ----------
    def draw_h(self, level, text):
        size = {1: 17, 2: 14, 3: 12, 4: 11, 5: 10.5, 6: 10.5}.get(level, 10.5)
        before = 16 if level <= 2 else (10 if level == 3 else 8)
        self.ensure(before + size + 8)
        self.y -= before
        fp = make_fp(size, bold=True)
        if level == 1:
            self.ax.text(PAGE_W / 2.0, self.y, text, fontproperties=fp,
                         va="top", ha="center", color="black", clip_on=False)
        else:
            self.put(MARGIN_L, self.y, text, fp)
        self.y -= size + 6

    def draw_p(self, text):
        size, line = 10, 15
        lines = wrap_text(text, capacity_for(USABLE_W, size))
        self.ensure(len(lines) * line + 4)
        fp = make_fp(size)
        for ln in lines:
            self.put(MARGIN_L, self.y, ln, fp)
            self.y -= line
        self.y -= 6

    def draw_note(self, text):
        size, line = 9.5, 14
        lines = wrap_text(text, capacity_for(USABLE_W - 22, size))
        self.ensure(len(lines) * line + 10)
        h = len(lines) * line + 8
        self.ax.plot([MARGIN_L + 4, MARGIN_L + 4], [self.y, self.y - h],
                     color="#888888", lw=1.5)
        fp = make_fp(size)
        for ln in lines:
            self.put(MARGIN_L + 12, self.y, ln, fp, color="#333333")
            self.y -= line
        self.y -= 8

    def draw_list(self, items):
        size, line = 10, 15
        fp = make_fp(size)
        for idx, it in enumerate(items):
            lines = wrap_text(it, capacity_for(USABLE_W - 22, size))
            self.ensure(len(lines) * line + 4)
            self.ax.text(MARGIN_L + 6, self.y, "\u2022",
                         fontproperties=fp, va="top", ha="center",
                         color="black", clip_on=False)
            for ln in lines:
                self.put(MARGIN_L + 18, self.y, ln, fp)
                self.y -= line
            self.y -= 3
        self.y -= 4

    def draw_code(self, code_text):
        size, line = 8.5, 12
        raw_lines = code_text.split("\n") if code_text else [""]
        lines = []
        for raw in raw_lines:
            lines.extend(wrap_text(raw, capacity_for(USABLE_W - 16, size)) or [""])
        fp = make_fp(size)
        i, n = 0, len(lines)
        while i < n:
            room = max(int((self.y - MARGIN_B - 6) // line), 1)
            take = min(room, n - i)
            chunk = lines[i:i + take]
            i += take
            h = len(chunk) * line + 10
            y0 = self.y
            self.ax.add_patch(Rectangle((MARGIN_L, y0 - h), USABLE_W, h,
                                        facecolor="#F5F5F5", edgecolor="#DDDDDD"))
            yy = y0 - 5
            for ln in chunk:
                self.put(MARGIN_L + 8, yy, ln, fp)
                yy -= line
            self.y = y0 - h - 6
            if i < n:
                self.new_page()

    def draw_table(self, header, rows):
        ncol = len(header)
        if not header and rows:
            ncol = max(len(r) for r in rows)
        ncol = max(ncol, 1)
        col_w = USABLE_W / ncol
        size, line, pad = 8.5, 11.5, 4
        usable_h = PAGE_H - MARGIN_T - MARGIN_B

        def wrap_cell(c):
            return wrap_text(c, capacity_for(col_w, size)) or [""]

        if header:
            hh = max((len(wrap_cell(c)) for c in header), default=1) * line + 2 * pad
        else:
            hh = 0

        def row_h(r):
            cells = (r + [""] * ncol)[:ncol]
            hmax = max((len(wrap_cell(c)) for c in cells), default=1)
            return hmax * line + 2 * pad

        # 按“剩余空间”将行切分为若干页
        chunks = []
        room = self.y - MARGIN_B - (hh + 8 if header else 0)
        cur = []
        for r in rows:
            rh = row_h(r)
            if rh > room and cur:
                chunks.append(cur)
                cur = []
                room = usable_h - (hh + 8 if header else 0)
            if rh > room and not cur:
                cur.append(r)
                room = 0
                continue
            cur.append(r)
            room -= rh
        if cur:
            chunks.append(cur)

        fp_small = make_fp(size)
        fp_bold = make_fp(size, bold=True)
        global_ri = 0
        for ci, group in enumerate(chunks):
            if ci > 0:
                self.new_page()
            y0 = self.y
            y = y0
            total = (hh if header else 0) + sum(row_h(r) for r in group)
            if header:
                self.ax.add_patch(Rectangle((MARGIN_L, y - hh), USABLE_W, hh,
                                            facecolor="#E8EEF7", edgecolor="#888888"))
                x = MARGIN_L
                for c in header:
                    for j, ln in enumerate(wrap_cell(c)):
                        self.put(x + 3, y - j * line - pad, ln, fp_bold)
                    x += col_w
                y -= hh
            for r in group:
                rh = row_h(r)
                if global_ri % 2 == 1:
                    self.ax.add_patch(Rectangle((MARGIN_L, y - rh), USABLE_W, rh,
                                                facecolor="#F7F7F7", edgecolor="#888888", lw=0.5))
                x = MARGIN_L
                cells = (r + [""] * ncol)[:ncol]
                for cix in range(ncol):
                    for j, ln in enumerate(wrap_cell(cells[cix])):
                        self.put(x + 3, y - j * line - pad, ln, fp_small)
                    x += col_w
                y -= rh
                global_ri += 1
            # 外框 + 网格线
            self.ax.add_patch(Rectangle((MARGIN_L, y0 - total), USABLE_W, total,
                                        fill=False, edgecolor="#888888"))
            for cix in range(1, ncol):
                x = MARGIN_L + cix * col_w
                self.ax.plot([x, x], [y0, y0 - total], color="#AAAAAA", lw=0.5)
            yy = y0 - hh
            rh_list = [row_h(r) for r in group]
            if len(rh_list) > 1:
                for rhh in rh_list[:-1]:
                    yy -= rhh
                    self.ax.plot([MARGIN_L, MARGIN_L + USABLE_W], [yy, yy],
                                 color="#AAAAAA", lw=0.5)
            self.y = y - 8


def main():
    with open(MD_FILE, encoding="utf-8") as f:
        lines = f.read().splitlines()
    blocks = parse_blocks(lines)
    with PdfPages(PDF_FILE) as pdf:
        r = Renderer(pdf)
        for b in blocks:
            r.draw(b)
        r.finish()
        n = r.page_no
    print("解析块数：%d，PDF 页数：%d" % (len(blocks), n))
    print("输出文件：%s" % PDF_FILE)


if __name__ == "__main__":
    main()
