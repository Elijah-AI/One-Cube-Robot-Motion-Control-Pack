#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把 scenario_palletizing_rack_transfer.md 导出为 Word (.docx)。

零外部依赖：仅使用 Python 标准库 zipfile，手写 OOXML（WordprocessingML）。
渲染规则与 export_scenario_palletizing_to_pdf.py 对齐：
  - # 标题 → Heading 样式（Word 导航窗格可用）
  - 普通段落 → 正文
  - > 引用块 → 左竖线 + 浅蓝底引用段落
  - 缩进列表 → 缩进段落（保留原文编号 / 圆点）
  - ``` 代码块 → 灰底等宽小字（保留原始换行）
  - | 表格  → 真实 Word 表格（表头浅蓝底加粗、带边框、按列数定宽）
  - --- 水平线 → 段落下边框
行内：**粗体** / `等宽代码` / [文字](链接)（链接只保留文字）。

页面：A4 横向，边距 2 cm，内容区宽约 25 cm（多数内容为宽表格）。
"""

import os
import re
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
MD_FILE = os.path.join(HERE, "scenario_palletizing_rack_transfer.md")
DOCX_FILE = os.path.join(HERE, "scenario_palletizing_rack_transfer.docx")

W = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"


def esc(t):
    """XML 转义（文本内容）。用 chr(38) 构造 '&'，避免源码出现字面实体。"""
    amp = chr(38)  # '&'
    return (t.replace(amp, amp + "amp;")
             .replace("<", amp + "lt;")
             .replace(">", amp + "gt;"))


# ---------------------------------------------------------------------------
# 行内 Markdown → run 列表（链接 / **粗体** / `等宽`）
# ---------------------------------------------------------------------------
_INLINE = re.compile(
    r"(\[[^\]\[]*\]\([^)]*\))"      # 1 链接 [...](...)
    r"|(\*\*([^*]+)\*\*)"            # 2/3 粗体
    r"|(`([^`]+)`)"                  # 4/5 等宽
)


def parse_inline(text):
    """返回 [(文本, 类型)]，类型 ∈ {'n','b','c'}。"""
    runs = []
    pos = 0
    for m in _INLINE.finditer(text):
        if m.start() > pos:
            runs.append((text[pos:m.start()], "n"))
        if m.group(1) is not None:          # 链接：仅保留 link 文字（可能内含 `code` / ** **）
            runs.extend(parse_inline(m.group(1)[1:-1].rsplit("](", 1)[0]))
        elif m.group(2) is not None:        # 粗体
            runs.append((m.group(3), "b"))
        else:                               # 等宽
            runs.append((m.group(5), "c"))
        pos = m.end()
    if pos < len(text):
        runs.append((text[pos:], "n"))
    return runs


def run_xml(text, kind="n", force_bold=False):
    rpr = ""
    if kind == "b" or force_bold:
        rpr = "<w:rPr><w:b/>"
        rpr += "</w:rPr>"
    elif kind == "c":
        rpr = ('<w:rPr><w:rFonts w:ascii="Consolas" w:hAnsi="Consolas" '
               'w:cs="Consolas"/><w:sz w:val="18"/></w:rPr>')
    return ('<w:r>%s<w:t xml:space="preserve">%s</w:t></w:r>'
            % (rpr, esc(text)))


def runs_xml(runs, force_bold=False):
    return "".join(run_xml(t, k, force_bold) for t, k in runs)


# ---------------------------------------------------------------------------
# 块 → OOXML
# ---------------------------------------------------------------------------
def heading_xml(level, text):
    pid = {1: "Heading1", 2: "Heading2", 3: "Heading3", 4: "Heading4"}.get(level, "Heading4")
    return ('<w:p><w:pPr><w:pStyle w:val="%s"/></w:pPr>%s</w:p>'
            % (pid, runs_xml(parse_inline(text), force_bold=False)))


def para_xml(text):
    return "<w:p>%s</w:p>" % runs_xml(parse_inline(text))


def quote_xml(multiline):
    """引用块：多行合并为一段，行间 <w:br/>。"""
    lines = []
    for ln in multiline.split("\n"):
        s = ln.strip()
        if s.startswith(">"):
            s = s[1:].strip()
        lines.append(s)
    text = "\n".join(lines)
    body = ""
    segs = text.split("\n")
    for i, seg in enumerate(segs):
        if i > 0:
            body += "<w:r><w:br/></w:r>"
        body += runs_xml(parse_inline(seg))
    ppr = (
        "<w:pPr>"
        '<w:shd w:val="clear" w:color="auto" w:fill="F2F6FC"/>'
        '<w:pBdr><w:left w:val="single" w:sz="18" w:space="4" w:color="8FAADC"/></w:pBdr>'
        '<w:ind w:left="284" w:right="142"/>'
        '<w:spacing w:before="80" w:after="120" w:line="276" w:lineRule="auto"/>'
        "</w:pPr>"
    )
    return "<w:p>%s%s</w:p>" % (ppr, body)


def code_xml(code_text):
    lines = code_text.split("\n")
    ppr = (
        "<w:pPr>"
        '<w:shd w:val="clear" w:color="auto" w:fill="F2F2F2"/>'
        '<w:ind w:left="142" w:right="142"/>'
        '<w:spacing w:before="40" w:after="80" w:line="240" w:lineRule="auto"/>'
        "<w:jc w:val=\"left\"/>"
        "</w:pPr>"
    )
    body = ""
    for i, ln in enumerate(lines):
        if i > 0:
            body += "<w:r><w:br/></w:r>"
        body += (
            '<w:r><w:rPr><w:rFonts w:ascii="Consolas" w:hAnsi="Consolas" '
            'w:eastAsia="宋体" w:cs="Consolas"/><w:sz w:val="15"/></w:rPr>'
            '<w:t xml:space="preserve">%s</w:t></w:r>' % esc(ln)
        )
    return "<w:p>%s%s</w:p>" % (ppr, body)


def hr_xml():
    return ('<w:p><w:pPr><w:pBdr>'
            '<w:bottom w:val="single" w:sz="6" w:space="1" w:color="BFBFBF"/>'
            "</w:pBdr></w:pPr></w:p>")


def list_xml(text, depth):
    ind = 284 + depth * 340
    ppr = ('<w:pPr><w:ind w:left="%d"/><w:spacing w:after="60"/>'
           "</w:pPr>" % ind)
    return "<w:p>%s%s</w:p>" % (ppr, runs_xml(parse_inline(text)))


# ---------------------------------------------------------------------------
# 表格（按列数 + 表头特征定宽，适配本文档）
# ---------------------------------------------------------------------------
def col_widths_cm(ncol, header):
    def _use():
        return [8.3, 16.7] if ncol == 2 else [5.0, 4.0, 16.0] \
            if ncol == 3 else None
    fixed = _use()
    if fixed:
        return fixed
    if ncol == 4:
        # 第 2 列含“语义”→ §6.1 type 表（语义/输出 两列宽）；否则 #|能力|说明|对应扩展
        if any("语义" in h for h in header):
            return [1.5, 7.2, 4.4, 11.9]
        return [1.4, 6.4, 11.6, 5.6]
    if ncol == 5:
        return [3.3, 2.6, 2.6, 5.8, 10.7]
    if ncol == 7:
        return [2.6, 1.6, 9.2, 3.4, 1.4, 1.6, 5.2]
    total = 25.0
    base = total / ncol
    return [base] * ncol


_SEP = re.compile(r":?-+:?")


def table_xml(row_lines):
    """row_lines: 该表所有物理行（含分隔行），返回 <w:tbl>。"""
    rows = []          # rows: list[list[str]]
    header = None
    for ln in row_lines:
        s = ln.strip()
        if not s.startswith("|"):
            continue
        cells = [c.strip() for c in s.strip("|").split("|")]
        if all(_SEP.fullmatch(c) for c in cells):
            continue                      # 表头分隔行
        rows.append(cells)
    if not rows:
        return ""
    header = rows[0]
    widths = col_widths_cm(len(header), header)
    # 统一列数（个别行多/少列时按 header 列数对齐）
    ncol = len(header)

    def cell_xml(text, wcm, is_header):
        wdxa = int(round(wcm * 567.0))
        fills = {"0": 1, "1": 1, "2": 1, "3": 1}
        ppr = "<w:pPr>"
        if is_header:
            ppr += '<w:shd w:val="clear" w:color="auto" w:fill="DEEAF6"/>'
        ppr += ('<w:jc w:val="left"/><w:spacing w:before="20" w:after="20" '
                'w:line="264" w:lineRule="auto"/></w:pPr>')
        inner = ppr + runs_xml(parse_inline(text), force_bold=is_header)
        return ('<w:tc><w:tcPr><w:tcW w:w="%d" w:type="dxa"/>'
                '<w:vAlign w:val="center"/></w:tcPr>'
                "<w:p>%s</w:p></w:tc>" % (wdxa, inner))

    tbl_borders = (
        '<w:tblBorders>'
        '<w:top w:val="single" w:sz="4" w:space="0" w:color="7F7F7F"/>'
        '<w:left w:val="single" w:sz="4" w:space="0" w:color="7F7F7F"/>'
        '<w:bottom w:val="single" w:sz="4" w:space="0" w:color="7F7F7F"/>'
        '<w:right w:val="single" w:sz="4" w:space="0" w:color="7F7F7F"/>'
        '<w:insideH w:val="single" w:sz="4" w:space="0" w:color="7F7F7F"/>'
        '<w:insideV w:val="single" w:sz="4" w:space="0" w:color="7F7F7F"/>'
        "</w:tblBorders>"
    )
    grid = "".join('<w:gridCol w:w="%d"/>' % int(round(w * 567.0)) for w in widths)

    body = ""
    for r_i, r in enumerate(rows):
        r = (r + [""] * ncol)[:ncol]
        is_header = (r_i == 0)
        cells = "".join(cell_xml(r[j], widths[j], is_header) for j in range(ncol))
        body += "<w:tr>%s</w:tr>" % cells

    tbl_pr = (
        '<w:tblPr><w:tblStyle w:val="TableGrid"/>'
        '<w:tblW w:w="%d" w:type="dxa"/>'
        '<w:tblLayout w:type="fixed"/><w:tblCellMar>'
        '<w:left w:w="57" w:type="dxa"/><w:right w:w="57" w:type="dxa"/>'
        "</w:tblCellMar>%s</w:tblPr>" % (int(round(25.0 * 567.0)), tbl_borders)
    )
    return ("<w:tbl>%s<w:tblGrid>%s</w:tblGrid>%s</w:tbl>" % (tbl_pr, grid, body))


# ---------------------------------------------------------------------------
# Markdown 分块
# ---------------------------------------------------------------------------
_HEAD = re.compile(r"^(#{1,4})\s+(.*)$")
_BULLET = re.compile(r"^(\s*)([-*])\s+(.*)$")
_NUM = re.compile(r"^(\s*)(\d+[.)])\s+(.*)$")


def split_blocks(text):
    lines = text.split("\n")
    blocks = []          # ('h',lv,text) ('p',t) ('quote',m) ('code',c)
                         # ('table', rows) ('hr',) ('li', depth, text)
    i, n = 0, len(lines)
    while i < n:
        line = lines[i]
        raw = line.rstrip("\n")
        if raw.strip().startswith("```"):
            buf = []
            i += 1
            while i < n and not lines[i].strip().startswith("```"):
                buf.append(lines[i])
                i += 1
            i += 1                       # 跳过结束 ``` 
            blocks.append(("code", "\n".join(buf)))
            continue

        s = raw.strip()
        if s == "":
            i += 1
            continue

        if s.startswith("|"):            # 连续表格行整体收集
            rows = []
            while i < n and lines[i].strip().startswith("|"):
                rows.append(lines[i])
                i += 1
            blocks.append(("table", rows))
            continue

        if s == "---" or s == "***" or s == "___":
            blocks.append(("hr",))
            i += 1
            continue

        m = _HEAD.match(s)
        if m:
            lv = len(m.group(1))
            blocks.append(("h", lv, m.group(2).strip()))
            i += 1
            continue

        if s.startswith(">"):            # 相邻引用行合并为一个块（保留空行）
            qlines = []
            while i < n and (lines[i].strip() == "" or lines[i].strip().startswith(">")):
                ql = lines[i].strip()
                if ql == "":
                    break
                qlines.append(ql)
                i += 1
            blocks.append(("quote", "\n".join(qlines)))
            continue

        m = _BULLET.match(raw)
        if m:
            depth = len(m.group(1)) // 2
            blocks.append(("li", depth, "• " + m.group(3).strip()))
            i += 1
            continue
        m = _NUM.match(raw)
        if m:
            depth = len(m.group(1)) // 2
            blocks.append(("li", depth, m.group(2) + " " + m.group(3).strip()))
            i += 1
            continue

        blocks.append(("p", s))
        i += 1
    return blocks


def render_blocks(blocks):
    out = []
    for b in blocks:
        if b[0] == "h":
            out.append(heading_xml(b[1], b[2]))
        elif b[0] == "p":
            out.append(para_xml(b[1]))
        elif b[0] == "quote":
            out.append(quote_xml(b[1]))
        elif b[0] == "code":
            out.append(code_xml(b[1]))
        elif b[0] == "table":
            out.append(table_xml(b[1]))
        elif b[0] == "hr":
            out.append(hr_xml())
        elif b[0] == "li":
            out.append(list_xml(b[2], b[1]))
    return "".join(out)


# ---------------------------------------------------------------------------
# OOXML 包裹文件
# ---------------------------------------------------------------------------
CONTENT_TYPES = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
<Default Extension="xml" ContentType="application/xml"/>
<Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>
<Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>
</Types>
"""

ROOT_RELS = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/>
</Relationships>
"""

DOC_RELS = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/>
</Relationships>
"""

STYLES = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:styles xmlns:w="%s">
<w:docDefaults>
<w:rPrDefault><w:rPr>
<w:rFonts w:ascii="Calibri" w:hAnsi="Calibri" w:eastAsia="宋体" w:cs="Calibri"/>
<w:sz w:val="21"/><w:szCs w:val="21"/>
<w:lang w:val="zh-CN" w:eastAsia="zh-CN"/>
</w:rPr></w:rPrDefault>
<w:pPrDefault><w:pPr><w:spacing w:after="120" w:line="276" w:lineRule="auto"/></w:pPr></w:pPrDefault>
</w:docDefaults>
<w:style w:type="paragraph" w:default="1" w:styleId="Normal">
<w:name w:val="Normal"/><w:qFormat/>
</w:style>
<w:style w:type="paragraph" w:styleId="Heading1">
<w:name w:val="heading 1"/><w:basedOn w:val="Normal"/><w:next w:val="Normal"/><w:qFormat/>
<w:pPr><w:keepNext/><w:keepLines/><w:spacing w:before="300" w:after="160"/>
<w:outlineLvl w:val="0"/><w:widowControl/></w:pPr>
<w:rPr><w:rFonts w:ascii="Arial" w:hAnsi="Arial" w:eastAsia="黑体"/><w:b/>
<w:color w:val="1F3864"/><w:sz w:val="32"/><w:szCs w:val="32"/></w:rPr>
</w:style>
<w:style w:type="paragraph" w:styleId="Heading2">
<w:name w:val="heading 2"/><w:basedOn w:val="Normal"/><w:next w:val="Normal"/><w:qFormat/>
<w:pPr><w:keepNext/><w:keepLines/><w:spacing w:before="260" w:after="120"/>
<w:outlineLvl w:val="1"/><w:widowControl/></w:pPr>
<w:rPr><w:rFonts w:ascii="Arial" w:hAnsi="Arial" w:eastAsia="黑体"/><w:b/>
<w:color w:val="2E5395"/><w:sz w:val="28"/><w:szCs w:val="28"/></w:rPr>
</w:style>
<w:style w:type="paragraph" w:styleId="Heading3">
<w:name w:val="heading 3"/><w:basedOn w:val="Normal"/><w:next w:val="Normal"/><w:qFormat/>
<w:pPr><w:keepNext/><w:keepLines/><w:spacing w:before="220" w:after="100"/>
<w:outlineLvl w:val="2"/><w:widowControl/></w:pPr>
<w:rPr><w:rFonts w:ascii="Arial" w:hAnsi="Arial" w:eastAsia="黑体"/><w:b/>
<w:color w:val="2E5395"/><w:sz w:val="24"/><w:szCs w:val="24"/></w:rPr>
</w:style>
<w:style w:type="paragraph" w:styleId="Heading4">
<w:name w:val="heading 4"/><w:basedOn w:val="Normal"/><w:next w:val="Normal"/><w:qFormat/>
<w:pPr><w:keepNext/><w:keepLines/><w:spacing w:before="180" w:after="80"/>
<w:outlineLvl w:val="3"/><w:widowControl/></w:pPr>
<w:rPr><w:rFonts w:ascii="Arial" w:hAnsi="Arial" w:eastAsia="黑体"/><w:b/>
<w:color w:val="404040"/><w:sz w:val="22"/><w:szCs w:val="22"/></w:rPr>
</w:style>
<w:style w:type="table" w:styleId="TableGrid">
<w:name w:val="Table Grid"/><w:basedOn w:val="TableNormal"/><w:uiPriority w:val="59"/>
<w:pPr><w:spacing w:after="0" w:line="240" w:lineRule="auto"/></w:pPr>
<w:tblPr><w:tblInd w:w="0" w:type="dxa"/><w:tblCellMar>
<w:top w:w="0" w:type="dxa"/><w:left w:w="57" w:type="dxa"/>
<w:bottom w:w="0" w:type="dxa"/><w:right w:w="57" w:type="dxa"/>
</w:tblCellMar></w:tblPr>
</w:style>
<w:style w:type="table" w:default="1" w:styleId="TableNormal">
<w:name w:val="Normal Table"/><w:uiPriority w:val="99"/><w:semiHidden/>
<w:unhideWhenUsed/><w:tblPr><w:tblInd w:w="0" w:type="dxa"/>
<w:tblCellMar><w:top w:w="0" w:type="dxa"/><w:left w:w="108" w:type="dxa"/>
<w:bottom w:w="0" w:type="dxa"/><w:right w:w="108" w:type="dxa"/></w:tblCellMar>
</w:tblPr></w:style>
</w:styles>
""" % W

SECTPR = (
    '<w:sectPr>'
    '<w:pgSz w:w="16838" w:h="11906" w:orient="landscape"/>'
    '<w:pgMar w:top="1134" w:right="1134" w:bottom="1134" w:left="1134" '
    'w:header="708" w:footer="708" w:gutter="0"/>'
    '<w:cols w:space="708"/><w:docGrid w:linePitch="360"/>'
    "</w:sectPr>"
)


def make_document_xml(body):
    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
        '<w:document xmlns:w="%s">'
        "<w:body>%s%s</w:body></w:document>"
        % (W, body, SECTPR)
    )


def build_docx(blocks):
    body = render_blocks(blocks)
    doc_xml = make_document_xml(body)
    with zipfile.ZipFile(DOCX_FILE, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", CONTENT_TYPES)
        z.writestr("_rels/.rels", ROOT_RELS)
        z.writestr("word/document.xml", doc_xml)
        z.writestr("word/_rels/document.xml.rels", DOC_RELS)
        z.writestr("word/styles.xml", STYLES)


def main():
    with open(MD_FILE, encoding="utf-8") as f:
        text = f.read()
    blocks = split_blocks(text)
    kinds = {}
    for b in blocks:
        kinds[b[0]] = kinds.get(b[0], 0) + 1
    build_docx(blocks)
    size = os.path.getsize(DOCX_FILE)
    print("DOCX 生成完毕：%s" % DOCX_FILE)
    print("大小：%d bytes；块统计：%s" % (size, kinds))


if __name__ == "__main__":
    main()
