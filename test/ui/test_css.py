"""CSS for viser widgets on the laptop's page: buttons side by side, button groups found, coloured, on one row."""

from husky_assembly_teleop.ui.css import (button_group_color_css, css_string, group_buttons, group_tooltips_css,
                                          one_row_css, side_by_side_css)


def test_side_by_side_buttons_share_a_row_by_their_ids():
    """Each button's box, found by the button's id, takes its share of the row, less viser's side margins."""
    css = side_by_side_css(["u1", "u2"])
    assert css.count("display: inline-block; width: calc(50% - 2 * var(--mantine-spacing-xs))") == 2
    assert 'div:has(> button[id="u1"])' in css and 'div:has(> button[id="u2"])' in css


def test_a_button_group_is_found_by_its_label_coloured_and_kept_on_one_row():
    """Its buttons by the label's `for`, one of them by its place from 0; a colour overrides viser's blue."""
    group = 'div:has(> div > p > label[for="g1"])'
    assert group_buttons("g1") == f"{group} button" and group_buttons("g1", 2) == f"{group} button:nth-child(3)"
    assert button_group_color_css("g1", "#f08c00") == (f"{group} button {{ color: #f08c00 !important; "
                                                       "border-color: #f08c00 !important; }")
    css = one_row_css("g1")
    assert f"{group} > div:last-child {{ min-width: 0; }}" in css and "flex-wrap: nowrap !important" in css


def test_a_button_groups_tooltips_hang_off_its_row_one_per_hovered_button():
    """Each tip is the row's `::after` while its button is hovered, as a CSS string with its line breaks; None: none."""
    group = 'div:has(> div > p > label[for="g1"])'
    css = group_tooltips_css("g1", ["J_M3\nTravel to the bar", None, 'say "hi"'])
    assert css.startswith(f"{group} {{ position: relative; }}{group}::after {{ display: none; position: absolute;")
    assert (f'{group}:has(button:nth-child(1):hover)::after {{ display: block; content: "J_M3\\A Travel to the bar"; }}'
            in css)
    assert "nth-child(2)" not in css
    assert css.endswith('nth-child(3):hover)::after { display: block; content: "say \\"hi\\""; }')
    assert css_string("a\\b") == '"a\\\\b"'
