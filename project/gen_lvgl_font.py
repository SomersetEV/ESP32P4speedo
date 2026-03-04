#!/usr/bin/env python3
"""
Generate LVGL 8.x 4-bpp font C files from Montserrat-Medium.ttf.

Key LVGL 8.x font-bitmap rules:
  - 4 bits per pixel (0 = transparent, 15 = opaque)
  - HIGH nibble = LEFT pixel (not low-first)
  - Each row padded to a full byte  (no row-to-row carry)
  - glyph_dsc.ofs_y = distance from baseline to BOTTOM of bbox (positive = above baseline)
  - glyph_dsc.adv_w  = advance width in 1/16-pixel units
"""

import freetype, os, sys

FONT = (
    r"c:/Users/benay/AppData/Local/Espressif/ComponentManager/Cache/"
    r"service_d92d8f1e/lvgl__lvgl_8.4.0_d7c1ac03/scripts/built_in_font/Montserrat-Medium.ttf"
)

OUT_DIR = (
    r"c:/Users/benay/OneDrive/Documents/github/Ultimate_Gauge_Board_PIO-main/"
    r"LT230 Speedo/project/main/tach_ui/fonts"
)

# ---------------------------------------------------------------------------

def to_4bpp(buf, pitch, w, h):
    """Pack 8-bpp greyscale FreeType bitmap → LVGL 4-bpp (hi-nibble = left pixel)."""
    out = bytearray()
    for row in range(h):
        base = row * abs(pitch)       # abs() in case pitch is ever negative
        for col in range(0, w, 2):
            hi = buf[base + col] >> 4
            lo = (buf[base + col + 1] >> 4) if (col + 1) < w else 0
            out.append((hi << 4) | lo)
    return bytes(out)


def render_glyphs(size_px, codepoints):
    """Return (glyphs_list, raw_bitmap_bytes, font_metrics_dict)."""
    face = freetype.Face(FONT)
    face.set_pixel_sizes(0, size_px)

    m = face.size
    # Divide 26.6 fixed-point by 64 to get pixels
    asc  =  m.ascender  >> 6
    dsc  =  m.descender >> 6          # negative
    lh   = asc - dsc                  # line height in pixels
    bl   = -dsc                       # base_line (pixels below baseline)

    glyphs = []
    bmap   = bytearray()

    for cp in codepoints:
        face.load_char(chr(cp), freetype.FT_LOAD_RENDER)
        g  = face.glyph
        bm = g.bitmap

        w, h   = bm.width, bm.rows
        pitch  = bm.pitch             # bytes per row in the FT buffer
        buf    = bytes(bm.buffer)

        ofs_x  = g.bitmap_left
        ofs_y  = g.bitmap_top - h     # bottom-of-bbox relative to baseline

        # advance.x is 26.6 fixed-point pixels; LVGL wants 1/16-px units
        adv_w  = (g.advance.x + 2) >> 2   # /64 * 16  = /4

        raw = to_4bpp(buf, pitch, w, h) if (w > 0 and h > 0) else b''

        glyphs.append(dict(
            cp           = cp,
            bitmap_index = len(bmap),
            adv_w        = adv_w,
            box_w        = w,
            box_h        = h,
            ofs_x        = ofs_x,
            ofs_y        = ofs_y,
        ))
        bmap.extend(raw)

    metrics = dict(line_height=lh, base_line=bl)
    return glyphs, bytes(bmap), metrics


def c_hex_block(data, indent=4):
    """Format a bytes object as C hex literals, 16 per line."""
    lines = []
    for i in range(0, len(data), 16):
        chunk = data[i:i+16]
        lines.append(" " * indent + ", ".join(f"0x{b:02x}" for b in chunk) + ",")
    return "\n".join(lines)


def generate_c(size_px, codepoints, symbol, guard, include):
    glyphs, bmap, metrics = render_glyphs(size_px, codepoints)

    lh = metrics["line_height"]
    bl = metrics["base_line"]

    # ---- cmap (SPARSE_TINY) ------------------------------------------------
    range_start  = min(codepoints)
    range_end    = max(codepoints)
    range_length = range_end - range_start + 1
    ulist        = [cp - range_start for cp in codepoints]

    L = []
    A = L.append

    A(f"/*{'*'*78}")
    A(f" * Size: {size_px} px  Bpp: 4")
    A(f" * Generated from Montserrat-Medium.ttf")
    A(f" * Glyphs: {', '.join(f'U+{cp:04X}({chr(cp)!r})' for cp in codepoints)}")
    A(f" {'*'*78}/")
    A(include)
    A("")
    A(f"#ifndef {guard}")
    A(f"#define {guard} 1")
    A(f"#endif")
    A("")
    A(f"#if {guard}")
    A("")
    A("static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {")

    for g in glyphs:
        bpr   = (g["box_w"] + 1) // 2
        total = bpr * g["box_h"]
        A(f"    /* U+{g['cp']:04X} {chr(g['cp'])!r} */")
        if total > 0:
            chunk = bmap[g["bitmap_index"]: g["bitmap_index"] + total]
            A(c_hex_block(chunk))

    A("};")
    A("")
    A("static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {")
    A("    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id=0 reserved */,")
    for g in glyphs:
        A(f"    {{.bitmap_index = {g['bitmap_index']}, "
          f".adv_w = {g['adv_w']}, "
          f".box_w = {g['box_w']}, "
          f".box_h = {g['box_h']}, "
          f".ofs_x = {g['ofs_x']}, "
          f".ofs_y = {g['ofs_y']}}},  /* U+{g['cp']:04X} {chr(g['cp'])!r} */")
    A("};")
    A("")
    A(f"static const uint16_t unicode_list_0[] = {{ {', '.join(str(x) for x in ulist)} }};")
    A("")
    A("static const lv_font_fmt_txt_cmap_t cmaps[] = {")
    A("    {")
    A(f"        .range_start = {range_start},")
    A(f"        .range_length = {range_length},")
    A( "        .glyph_id_start = 1,")
    A( "        .unicode_list = unicode_list_0,")
    A( "        .glyph_id_ofs_list = NULL,")
    A(f"        .list_length = {len(codepoints)},")
    A( "        .type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY,")
    A("    }")
    A("};")
    A("")
    A("#if LV_VERSION_CHECK(8, 0, 0)")
    A("static lv_font_fmt_txt_glyph_cache_t cache;")
    A("static const lv_font_fmt_txt_dsc_t font_dsc = {")
    A("#else")
    A("static lv_font_fmt_txt_dsc_t font_dsc = {")
    A("#endif")
    A("    .glyph_bitmap = glyph_bitmap, .glyph_dsc = glyph_dsc,")
    A("    .cmaps = cmaps, .kern_dsc = NULL, .kern_scale = 0,")
    A("    .cmap_num = 1, .bpp = 4, .kern_classes = 0, .bitmap_format = 0,")
    A("#if LV_VERSION_CHECK(8, 0, 0)")
    A("    .cache = &cache")
    A("#endif")
    A("};")
    A("")
    A("#if LV_VERSION_CHECK(8, 0, 0)")
    A(f"const lv_font_t {symbol} = {{")
    A("#else")
    A(f"lv_font_t {symbol} = {{")
    A("#endif")
    A("    .get_glyph_dsc    = lv_font_get_glyph_dsc_fmt_txt,")
    A("    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,")
    A(f"    .line_height = {lh}, .base_line = {bl},")
    A("#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)")
    A("    .subpx = LV_FONT_SUBPX_NONE,")
    A("#endif")
    A("#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8")
    A("    .underline_position = -2, .underline_thickness = 1,")
    A("#endif")
    A("    .dsc = &font_dsc")
    A("};")
    A("")
    A(f"#endif /*#if {guard}*/")

    return "\n".join(L) + "\n"


# ---------------------------------------------------------------------------
# Generate ui_font_Montserrat_48 — digits + gear letters
# ---------------------------------------------------------------------------
CP_48 = (
    [0x20, 0x2E]               # space, '.'
    + list(range(0x30, 0x3A))  # 0-9
    + [0x44, 0x4E, 0x50, 0x52, 0x53]  # D N P R S
)

src48 = generate_c(
    size_px   = 48,
    codepoints= CP_48,
    symbol    = "ui_font_Montserrat_48",
    guard     = "UI_FONT_MONTSERRAT_48",
    include   = '#include "../ui.h"',
)
out48 = os.path.join(OUT_DIR, "ui_font_Montserrat_48.c")
with open(out48, "w", newline="\n") as f:
    f.write(src48)
print(f"Wrote {out48}  ({len(src48)} chars)")

# ---------------------------------------------------------------------------
# Generate ui_font_Montserrat_120 — dash + digits for speed display
# ---------------------------------------------------------------------------
CP_120 = (
    [0x20, 0x2D]               # space, '-'
    + list(range(0x30, 0x3A))  # 0-9
)

src120 = generate_c(
    size_px   = 120,
    codepoints= CP_120,
    symbol    = "ui_font_Montserrat_120",
    guard     = "UI_FONT_MONTSERRAT_120",
    include   = '#include "../ui.h"',
)
out120 = os.path.join(OUT_DIR, "ui_font_Montserrat_120.c")
with open(out120, "w", newline="\n") as f:
    f.write(src120)
print(f"Wrote {out120}  ({len(src120)} chars)")

print("Done.")
