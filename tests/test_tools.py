"""Small tools: the serial logger (over pyserial's loopback URL) and the log reader."""

from __future__ import annotations

import importlib.util

import pytest

from bootcamp.logs import read_log, write_log
from conftest import ROOT


def _load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "tools" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_serial_logger_records_what_the_port_returns(tmp_path):
    pytest.importorskip("serial")
    out = tmp_path / "log.csv"
    logger = _load("serial_logger")
    # loop:// echoes everything written: the "sketch output" is the commands we send
    rc = logger.main(
        [
            "--port",
            "loop://",
            "--seconds",
            "1.0",
            "--send-after",
            "0",
            "--send",
            "# banner",
            "--send",
            "t_ms,y",
            "--send",
            "0,1.5",
            "--send",
            "10,2.5",
            str(out),
        ]
    )
    assert rc == 0
    log = read_log(out)
    assert log["y"].tolist() == [1.5, 2.5]


def test_write_and_read_log_round_trip(tmp_path):
    p = tmp_path / "x.csv"
    write_log(p, {"t": [0, 1, 2], "v": [0.5, 1.5, -2.0]}, comment="SYNTHETIC\nsecond line")
    d = read_log(p)
    assert d["v"].tolist() == [0.5, 1.5, -2.0]
    assert p.read_text().startswith("# SYNTHETIC")
