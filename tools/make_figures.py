"""Regenerate figures/ by executing the solution notebooks with BOOTCAMP_SAVE_FIGURES=1.

python tools/make_figures.py            # all sessions
python tools/make_figures.py s04 s06    # only these
"""

from __future__ import annotations

import importlib.util
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main(argv=None) -> int:
    import nbclient
    import nbformat

    spec = importlib.util.spec_from_file_location(
        "build_notebooks", ROOT / "tools" / "build_notebooks.py"
    )
    bn = importlib.util.module_from_spec(spec)
    sys.modules["build_notebooks"] = bn
    spec.loader.exec_module(bn)
    only = argv if argv is not None else sys.argv[1:]
    os.environ["BOOTCAMP_SAVE_FIGURES"] = "1"
    os.environ["MPLBACKEND"] = "Agg"
    for src, _, _ in bn.targets():
        if only and not any(src.stem.startswith(o) for o in only):
            continue
        t0 = time.time()
        nb = nbformat.reads(bn.dumps(bn.build(src, student=False)), as_version=4)
        client = nbclient.NotebookClient(
            nb,
            timeout=1800,
            kernel_name="python3",
            resources={"metadata": {"path": str(ROOT / "notebooks")}},
        )
        client.execute()
        print(f"{src.stem}: {time.time() - t0:.0f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
