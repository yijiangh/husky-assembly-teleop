"""
The generated reports' look: an engineering drawing sheet, one offline page with everything inline.

- `page` wraps a body into a whole document; `titleblock`, `section`, `table`, `cards` are its parts.
- ! Text from data goes through `esc`; `table` does not escape its cells, as they may hold marks.
"""

from __future__ import annotations

from html import escape
from pathlib import Path
from typing import Iterable, Sequence

CSS = """
:root {
  --paper: #f4f5f3; --sheet: #fcfcfb; --ink: #14181b; --ink-2: #485158; --muted: #747c81; --rule: #dde0dd;
  --rule-strong: #c2c7c3; --accent: #1c5d8a; --accent-wash: #e4ecf2; --s1: #2a78d6; --s2: #eb6834; --s3: #1baf7a;
  --down: #2a78d6; --up: #e34948; --band: #ebeae5; --rh: #6250d6; --mc: #eda100; --used: #1a7f37;
  --font-display: "Barlow Semi Condensed", "Arial Narrow", "Segoe UI", system-ui, sans-serif;
  --font-body: "Atkinson Hyperlegible", system-ui, -apple-system, "Segoe UI", sans-serif;
  --font-data: "JetBrains Mono", ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
  color-scheme: light;
}
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) {
    --paper: #0e1012; --sheet: #17191a; --ink: #eceeed; --ink-2: #b6bdc1; --muted: #8b9297; --rule: #2a2e30;
    --rule-strong: #3d4245; --accent: #82b6de; --accent-wash: #1a2733; --s1: #3987e5; --s2: #d95926; --s3: #199e70;
    --down: #3987e5; --up: #e66767; --band: #24272a; --rh: #9085e9; --mc: #c98500; --used: #3fb950;
    color-scheme: dark;
  }
}
:root[data-theme="dark"] {
  --paper: #0e1012; --sheet: #17191a; --ink: #eceeed; --ink-2: #b6bdc1; --muted: #8b9297; --rule: #2a2e30;
  --rule-strong: #3d4245; --accent: #82b6de; --accent-wash: #1a2733; --s1: #3987e5; --s2: #d95926; --s3: #199e70;
  --down: #3987e5; --up: #e66767; --band: #24272a; --rh: #9085e9; --mc: #c98500; --used: #3fb950;
  color-scheme: dark;
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--paper); color: var(--ink); font-family: var(--font-body); font-size: 1rem;
  line-height: 1.55; }
.page { max-width: 74rem; margin: 0 auto; padding-left: max(16px, env(safe-area-inset-left, 0px));
  padding-right: max(16px, env(safe-area-inset-right, 0px)); padding-block: 28px 72px; display: grid; gap: 56px; }
.page > * { min-width: 0; }
h1, h2, h3, h4 { font-family: var(--font-display); font-weight: 600; margin: 0; line-height: 1.15; }
h1 { font-size: clamp(2rem, 4.6vw, 3.1rem); letter-spacing: -0.01em; }
h2 { font-size: 1.75rem; }
h3 { font-size: 1.2rem; }
p { margin: 0; }
a { color: var(--accent); }
.num, td.n, .mono { font-family: var(--font-data); font-variant-numeric: tabular-nums; }
.eyebrow { font-family: var(--font-display); font-weight: 600; font-size: 0.8rem; text-transform: uppercase;
  letter-spacing: 0.09em; color: var(--accent); }
.muted { color: var(--muted); }
section { display: grid; gap: 18px; min-width: 0; }
section > * { min-width: 0; }
.section-head { display: grid; gap: 8px; max-width: 46rem; }
.section-head p { color: var(--ink-2); }
.section-head.sub { margin-top: 18px; }
.titleblock { display: grid; grid-template-columns: minmax(0, 1.35fr) minmax(0, 1fr); border: 1.5px solid var(--ink);
  background: var(--sheet); }
.tb-main { padding: 26px 28px; display: grid; gap: 12px; align-content: start; border-right: 1.5px solid var(--ink); }
.tb-main .lede { color: var(--ink-2); max-width: 38rem; }
.tb-fields { margin: 0; display: grid; }
.tb-fields > div { display: grid; grid-template-columns: 8.4rem minmax(0, 1fr);
  border-bottom: 1px solid var(--rule-strong); }
.tb-fields > div:last-child { border-bottom: 0; }
.tb-fields dt { font-family: var(--font-display); font-weight: 600; font-size: 0.74rem; text-transform: uppercase;
  letter-spacing: 0.07em; color: var(--muted); padding: 10px 12px; border-right: 1px solid var(--rule-strong); }
.tb-fields dd { margin: 0; padding: 9px 12px; font-size: 0.92rem; min-width: 0; overflow-wrap: anywhere; }
@media (max-width: 760px) {
  .titleblock { grid-template-columns: 1fr; }
  .tb-main { border-right: 0; border-bottom: 1.5px solid var(--ink); padding: 20px 18px; }
  .tb-fields > div { grid-template-columns: 6.8rem minmax(0, 1fr); }
}
.table-wrap { overflow-x: auto; background: var(--sheet); border: 1px solid var(--rule); }
table { border-collapse: collapse; width: 100%; font-size: 0.86rem; }
th, td { text-align: left; vertical-align: top; padding: 9px 12px; border-bottom: 1px solid var(--rule); }
thead th { font-family: var(--font-display); font-weight: 600; font-size: 0.76rem; text-transform: uppercase;
  letter-spacing: 0.06em; color: var(--muted); background: var(--sheet); }
tbody tr:last-child td, tbody tr:last-child th { border-bottom: 0; }
td.n, th.n { text-align: right; white-space: nowrap; }
td.time { white-space: nowrap; font-family: var(--font-data); font-size: 0.78rem; }
td.z { color: var(--muted); }
small { color: var(--muted); }
.chart { margin: 0; background: var(--sheet); border: 1px solid var(--rule); padding: 16px 16px 12px; display: grid;
  gap: 10px; min-width: 0; }
.chart figcaption { display: grid; gap: 4px; }
.chart figcaption b { font-family: var(--font-display); font-weight: 600; font-size: 1.12rem; }
.chart figcaption span { color: var(--ink-2); font-size: 0.9rem; max-width: 52rem; }
.chart details { border-top: 1px solid var(--rule); padding-top: 8px; }
.chart details .table-wrap { margin-top: 10px; border: 0; }
.chart details table { font-size: 0.82rem; }
summary { cursor: pointer; color: var(--accent); font-size: 0.88rem; width: fit-content; }
.chart-legend { display: flex; flex-wrap: wrap; gap: 6px 16px; font-size: 0.84rem; color: var(--ink-2); }
.chart-legend span { display: inline-flex; align-items: center; gap: 6px; }
.chart-legend i { display: inline-block; width: 18px; height: 0; border-top: 2px solid; }
.chart-legend i.box { width: 12px; height: 12px; border: 0; border-radius: 2px; }
.multiples { display: grid; grid-template-columns: repeat(auto-fill, minmax(min(100%, 15.5rem), 1fr));
  gap: 14px 18px; }
.panel { display: grid; gap: 4px; min-width: 0; align-content: start; }
.panel h4 { font-size: 1rem; }
.panel p { font-size: 0.78rem; color: var(--muted); line-height: 1.35; overflow-wrap: anywhere; }
.panel svg { display: block; width: 100%; height: auto; overflow: visible; }
svg text { font-family: var(--font-data); font-size: 10px; fill: var(--muted); }
.cards { display: grid; grid-template-columns: repeat(auto-fit, minmax(min(100%, 20rem), 1fr)); gap: 14px; }
.note { background: var(--sheet); border: 1px solid var(--rule); padding: 14px 16px; display: grid; gap: 6px;
  align-content: start; }
.note h3 { font-size: 1.05rem; }
.note p { color: var(--ink-2); font-size: 0.93rem; }
.note p.big { font-family: var(--font-display); font-weight: 700; font-size: 2.4rem; line-height: 1;
  color: var(--ink); }
.method { display: grid; gap: 10px; max-width: 52rem; color: var(--ink-2); }
.method dl { margin: 0; display: grid; gap: 10px; }
.method dt { font-weight: 700; color: var(--ink); }
.method dd { margin: 0; }
footer { color: var(--muted); font-size: 0.84rem; border-top: 1px solid var(--rule); padding-top: 14px; }
.flag { display: inline-block; font-family: var(--font-display); font-weight: 600; font-size: 0.74rem;
  padding: 1px 7px; border-radius: 3px; border: 1px solid color-mix(in oklab, var(--mc) 60%, var(--sheet));
  background: color-mix(in oklab, var(--mc) 20%, var(--sheet)); white-space: nowrap; margin-right: 4px; }
.tick { display: inline-flex; align-items: baseline; font-family: var(--font-display); font-weight: 700;
  font-size: 0.8rem; white-space: nowrap; }
.tick::before { font-family: var(--font-body); font-weight: 700; margin-right: 4px; }
.tick.used { color: var(--used); }
.tick.used::before { content: "\\2713"; }
.tick.unused { color: var(--muted); }
.tick.unused::before { content: "\\2717"; }
.boxes { display: grid; gap: 3px; }
.boxes small { display: block; font-size: 0.74rem; }
/* * Horizontal bars of a change, with zero in the track. */
.bars { display: grid; gap: 2px; }
.bar-row { display: grid; grid-template-columns: minmax(8rem, 13rem) minmax(6rem, 1fr) 11rem; gap: 10px;
  align-items: center; font-size: 0.82rem; padding: 2px 0; }
.bar-row:hover { background: var(--band); }
.bar-label small { display: block; line-height: 1.2; font-size: 0.72rem; overflow-wrap: anywhere; }
.bar-track { position: relative; height: 16px; }
.bar-track .zero { position: absolute; top: -2px; bottom: -2px; width: 1px; background: var(--ink-2); }
.bar-track .mark { position: absolute; top: 2px; height: 12px; }
.bar-track .mark.down { background: var(--down); border-radius: 4px 0 0 4px; }
.bar-track .mark.up { background: var(--up); border-radius: 0 4px 4px 0; }
.bar-vals { text-align: right; white-space: nowrap; font-family: var(--font-data); font-size: 0.76rem; }
.bar-vals b { font-weight: 500; display: inline-block; min-width: 3.4rem; }
.bar-group { display: grid; gap: 4px; }
.bar-group + .bar-group { margin-top: 10px; }
.bar-group h3 { font-size: 1.02rem; }
.bar-group h3 small { font-family: var(--font-body); font-weight: 400; font-size: 0.8rem; }
.bar-axis { display: flex; justify-content: space-between; font-family: var(--font-data); font-size: 0.72rem;
  color: var(--muted); }
@media (max-width: 640px) {
  .bar-row { grid-template-columns: minmax(0, 1fr) minmax(0, 1fr); }
  .bar-track { grid-column: 1 / -1; grid-row: 2; }
}
/* * The incident page: kind chips, the count matrix and the per-day tables. */
.chip { display: inline-block; font-family: var(--font-display); font-weight: 600; font-size: 0.74rem;
  letter-spacing: 0.02em; padding: 1px 7px; border-radius: 3px; border: 1px solid var(--rule-strong);
  white-space: nowrap; color: var(--ink); }
.chip.k-red { background: color-mix(in oklab, var(--up) 16%, var(--sheet));
  border-color: color-mix(in oklab, var(--up) 55%, var(--sheet)); }
.chip.k-amber { background: color-mix(in oklab, var(--mc) 20%, var(--sheet));
  border-color: color-mix(in oklab, var(--mc) 60%, var(--sheet)); }
.chip.k-grey { background: var(--band); }
.chip.k-blue { background: color-mix(in oklab, var(--s1) 12%, var(--sheet));
  border-color: color-mix(in oklab, var(--s1) 45%, var(--sheet)); }
.chip.k-violet { background: color-mix(in oklab, var(--rh) 13%, var(--sheet));
  border-color: color-mix(in oklab, var(--rh) 50%, var(--sheet)); }
table.matrix { min-width: 52rem; font-family: var(--font-data); font-size: 0.8rem;
  font-variant-numeric: tabular-nums; }
table.matrix th[scope="row"], table.matrix td:nth-child(2) { font-family: var(--font-body); white-space: nowrap; }
table.matrix thead th { white-space: nowrap; }
table.matrix td.tot { font-weight: 500; color: var(--ink); }
table.matrix tr.sum th, table.matrix tr.sum td { border-top: 1.5px solid var(--ink); font-weight: 600;
  color: var(--ink); }
table.inc { min-width: 56rem; }
table.inc td { color: var(--ink-2); font-size: 0.84rem; }
table.inc td.time { color: var(--ink); }
table.inc td.what { min-width: 26rem; color: var(--ink); }
table.inc details { margin-top: 4px; }
table.inc .ev { display: grid; gap: 3px; margin-top: 4px; }
table.inc code { font-family: var(--font-data); font-size: 0.72rem; color: var(--ink-2); background: var(--band);
  padding: 2px 5px; border-radius: 2px; overflow-wrap: anywhere; }
.inf { font-family: var(--font-display); font-size: 0.72rem; font-weight: 600; text-transform: uppercase;
  letter-spacing: 0.06em; color: var(--muted); margin-left: 4px; }
details.day { background: var(--sheet); border: 1px solid var(--rule); }
details.day > summary { padding: 12px 16px; font-family: var(--font-display); font-weight: 600; font-size: 1.1rem;
  color: var(--ink); }
details.day > summary span { font-family: var(--font-body); font-weight: 400; font-size: 0.88rem;
  color: var(--muted); margin-left: 8px; }
details.day .table-wrap { border: 0; border-top: 1px solid var(--rule); }
#tip { position: fixed; z-index: 20; pointer-events: none; background: var(--sheet); color: var(--ink);
  border: 1px solid var(--rule-strong); box-shadow: 0 4px 14px rgba(0, 0, 0, 0.12); padding: 8px 10px;
  font-size: 0.8rem; line-height: 1.4; max-width: 20rem; border-radius: 4px; white-space: pre-line; }
"""
#: Any element with `data-tip` shows its text beside the pointer.
TIP_SCRIPT = """<div id="tip" hidden></div><script>
const tip = document.getElementById("tip");
document.addEventListener("pointermove", (event) => {
  const on = event.target.closest && event.target.closest("[data-tip]");
  if (!on) { tip.hidden = true; return; }
  tip.textContent = on.dataset.tip;
  tip.hidden = false;
  tip.style.left = Math.max(8, Math.min(event.clientX + 12, innerWidth - tip.offsetWidth - 8)) + "px";
  tip.style.top = Math.max(8, Math.min(event.clientY + 14, innerHeight - tip.offsetHeight - 8)) + "px";
});
</script>"""


def esc(value: object) -> str:
    """Text from data as safe HTML, quotes included."""
    return escape(str(value), quote=True)


def number(value: float | None, digits: int = 2) -> str:
    """A number for a table: a dash when there is none."""
    return "–" if value is None else f"{value:.{digits}f}"


def page(title: str, body: str) -> str:
    """A whole offline document around a body."""
    return (f'<!doctype html><html lang="en"><head><meta charset="utf-8">'
            f'<meta name="viewport" content="width=device-width,initial-scale=1"><title>{esc(title)}</title>'
            f'<style>{CSS}</style></head><body><div class="page">{body}</div>{TIP_SCRIPT}</body></html>')


def write(html: str, out: Path) -> Path:
    """Write a page, making its folder when needed, and return its path."""
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(html, encoding="utf-8")
    return out


def titleblock(eyebrow: str, title: str, lede: str, fields: Iterable[tuple[str, str]]) -> str:
    """The drawing's title block: the title and a lede on the left, the fields on the right; fields may hold marks."""
    rows = "".join(f"<div><dt>{esc(name)}</dt><dd>{value}</dd></div>" for name, value in fields)
    return (f'<header class="titleblock"><div class="tb-main"><p class="eyebrow">{esc(eyebrow)}</p>'
            f'<h1>{esc(title)}</h1><p class="lede">{lede}</p></div><dl class="tb-fields">{rows}</dl></header>')


def section(eyebrow: str, title: str, intro: str, body: str) -> str:
    """One section of the sheet; the intro and the body may hold marks."""
    words = f"<p>{intro}</p>" if intro else ""
    return (f'<section><div class="section-head"><p class="eyebrow">{esc(eyebrow)}</p><h2>{esc(title)}</h2>{words}'
            f"</div>{body}</section>")


def table(headers: Sequence[tuple[str, str]], rows: Iterable[Iterable[tuple[str, str]]], css: str = "") -> str:
    """A table in its scrolling frame; a header and a cell are (content, class), cells may hold marks."""
    head = "".join(f'<th scope="col" class="{kind}">{esc(name)}</th>' for name, kind in headers)
    lines = "".join("<tr>" + "".join(f'<td class="{kind}">{cell}</td>' for cell, kind in row) + "</tr>"
                    for row in rows)
    return (f'<div class="table-wrap"><table class="{css}"><thead><tr>{head}</tr></thead>'
            f"<tbody>{lines}</tbody></table></div>")


def method(entries: Iterable[tuple[str, str]]) -> str:
    """The method list: (term, what it means); the meaning may hold marks."""
    rows = "".join(f"<dt>{esc(term)}</dt><dd>{meaning}</dd>" for term, meaning in entries)
    return f'<div class="method"><dl>{rows}</dl></div>'
