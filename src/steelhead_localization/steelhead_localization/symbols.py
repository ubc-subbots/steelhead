from collections import Counter
from dataclasses import dataclass

import yaml

# Fields in each symbol entry that must be numbers
NUMBER_FIELDS = ("x", "y", "z", "height")


# One symbol: its YOLO label, where it is in the pool, and its real height
@dataclass(frozen=True)
class Symbol:
    name: str
    label: str
    position: tuple
    height: float


# Reads a symbol YAML file into a list of symbols
def load_symbols(path):
    with open(path) as file:
        return parse_symbols(yaml.safe_load(file))


# Turns YAML data into a list of symbols, rejecting bad entries
def parse_symbols(data):
    if not isinstance(data, dict) or not isinstance(data.get("symbols"), dict):
        raise ValueError("symbol file must contain a 'symbols' mapping")

    symbols = []
    for name, entry in data["symbols"].items():
        if not isinstance(entry, dict):
            raise ValueError(f"symbol '{name}' must be a mapping of fields")
        label = entry.get("label")
        if not isinstance(label, str) or not label:
            raise ValueError(f"symbol '{name}' needs a non-empty 'label'")
        for field in NUMBER_FIELDS:
            if not _is_number(entry.get(field)):
                raise ValueError(f"symbol '{name}' needs a number for '{field}'")
        if entry["height"] <= 0:
            raise ValueError(f"symbol '{name}' must have a positive 'height'")
        symbols.append(
            Symbol(
                name=str(name),
                label=label,
                position=(float(entry["x"]), float(entry["y"]), float(entry["z"])),
                height=float(entry["height"]),
            )
        )
    return symbols


# Makes a label -> symbol lookup, leaving out labels used more than once
def unique_by_label(symbols):
    counts = Counter(symbol.label for symbol in symbols)
    return {symbol.label: symbol for symbol in symbols if counts[symbol.label] == 1}


# Checks whether a value is a real number (not True/False)
def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)
