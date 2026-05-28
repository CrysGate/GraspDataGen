import argparse

import numpy as np
from rich import box
from rich.console import Console
from rich.panel import Panel
from rich.pretty import Pretty
from rich.table import Table
from rich.text import Text

def _array_preview(arr):
    if arr.size <= 10 or (arr.ndim == 1 and arr.size <= 50):
        return Pretty(arr.tolist(), max_length=12, max_depth=2)
    return Text("-", style="dim")


def inspect_npz(path):
    console = Console()
    with np.load(path, allow_pickle=True) as data:
        console.print(
            Panel.fit(
                Text(str(path), style="bold"),
                title="NPZ",
                border_style="cyan",
            )
        )
        console.print(Text(f"包含 {len(data.files)} 个数组", style="bold green"))

        table = Table(
            title="内容",
            box=box.MINIMAL_DOUBLE_HEAD,
            show_lines=False,
        )
        table.add_column("Key", style="bold", no_wrap=True)
        table.add_column("Kind", style="magenta", no_wrap=True)
        table.add_column("Shape", style="cyan", no_wrap=True)
        table.add_column("Dtype", style="green", no_wrap=True)
        table.add_column("Preview", overflow="fold")

        counts = {"array": 0, "scalar": 0, "dict": 0, "object": 0}

        for key in data.files:
            arr = data[key]
            if arr.shape == ():
                shape = "-"
                dtype = str(arr.dtype)
                try:
                    val = arr.item()
                    if isinstance(val, dict):
                        kind = "dict"
                        counts["dict"] += 1
                        preview = Pretty(val, max_length=12, max_depth=2)
                    else:
                        kind = "scalar"
                        counts["scalar"] += 1
                        preview = Pretty(val, max_length=12, max_depth=2)
                except Exception:
                    kind = "object"
                    counts["object"] += 1
                    preview = Text("<object>", style="dim")
            else:
                kind = "array"
                counts["array"] += 1
                shape = str(arr.shape)
                dtype = str(arr.dtype)
                preview = _array_preview(arr)

            table.add_row(key, kind, shape, dtype, preview)

        console.print(table)
        summary = Text.assemble(
            ("数组 ", "bold"),
            (str(counts["array"]), "bold cyan"),
            (" | 标量 ", "bold"),
            (str(counts["scalar"]), "bold cyan"),
            (" | 字典 ", "bold"),
            (str(counts["dict"]), "bold cyan"),
            (" | 对象 ", "bold"),
            (str(counts["object"]), "bold cyan"),
        )
        console.print(summary)


def main():
    parser = argparse.ArgumentParser(description="Inspect contents of a .npz file")
    parser.add_argument("npz_file", type=str, help="Path to the .npz file to inspect")
    args = parser.parse_args()
    path = args.npz_file
    inspect_npz(path)

if __name__ == "__main__":
    main()