from pathlib import Path

import pytest

from steelhead_localization.symbols import load_symbols, parse_symbols, unique_by_label

# Folder holding the symbol YAML files
CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"
# A correct symbol entry
VALID_ENTRY = {"label": "sos", "x": 3.0, "y": -1.0, "z": -0.6, "height": 0.168}


# The gate file loads with all 4 gate symbols
def test_gate_config_loads():
    symbols = load_symbols(CONFIG_DIR / "fiducial_symbols_gate.yaml")
    labels = {symbol.label for symbol in symbols}
    assert labels == {"sos", "buoy", "compass", "hammer_and_wrench"}


# The torpedo file keeps fire and blood, and drops the repeated circles
def test_torpedo_config_skips_repeated_circles():
    symbols = load_symbols(CONFIG_DIR / "fiducial_symbols_torpedo.yaml")
    assert set(unique_by_label(symbols)) == {"fire", "blood"}


# A correct entry is read with the right values
def test_valid_entry_is_parsed():
    (symbol,) = parse_symbols({"symbols": {"gate_sos": {**VALID_ENTRY, "x": 3}}})
    assert symbol.name == "gate_sos"
    assert symbol.label == "sos"
    assert symbol.position == (3.0, -1.0, -0.6)
    assert symbol.height == 0.168


# Broken entries are rejected with a clear error
@pytest.mark.parametrize(
    "entry, message",
    [
        ({**VALID_ENTRY, "label": ""}, "label"),
        ({**VALID_ENTRY, "x": "far"}, "'x'"),
        ({**VALID_ENTRY, "z": True}, "'z'"),
        ({k: v for k, v in VALID_ENTRY.items() if k != "height"}, "'height'"),
        ({**VALID_ENTRY, "height": -0.1}, "positive"),
        ("not a mapping", "mapping"),
    ],
)
def test_bad_entry_is_rejected(entry, message):
    with pytest.raises(ValueError, match=message):
        parse_symbols({"symbols": {"bad": entry}})


# A file with no symbols section is rejected
def test_missing_symbols_section_is_rejected():
    with pytest.raises(ValueError, match="'symbols' mapping"):
        parse_symbols({"stuff": 1})


# Labels used more than once are dropped from the lookup
def test_unique_by_label_drops_shared_labels():
    symbols = parse_symbols(
        {
            "symbols": {
                "circle_a": {**VALID_ENTRY, "label": "circle"},
                "circle_b": {**VALID_ENTRY, "label": "circle"},
                "gate_sos": VALID_ENTRY,
            }
        }
    )
    assert set(unique_by_label(symbols)) == {"sos"}
