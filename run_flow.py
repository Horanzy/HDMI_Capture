import argparse
import json
import shutil
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
GW_SH = Path(r"E:\Gowin\Gowin_V1.9.11.03_Education_x64\IDE\bin\gw_sh.exe")


def kill_tree(pid):
    return subprocess.run(
        ["taskkill", "/PID", str(pid), "/T", "/F"],
        capture_output=True,
        text=True,
        timeout=15,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", required=True)
    parser.add_argument("--tcl", default="run_pnr.tcl")
    parser.add_argument("--sdc")
    parser.add_argument("--timeout", type=int, default=100)
    parser.add_argument("--stall", type=int, default=30)
    args = parser.parse_args()

    if args.sdc:
        shutil.copy2(Path(args.sdc), ROOT / "src" / "timing.sdc")

    run_dir = ROOT / "runs" / args.name
    run_dir.mkdir(parents=True, exist_ok=True)
    stdout_path = run_dir / "console.txt"
    pnr_log = ROOT / "impl" / "pnr" / "test.log"
    if args.tcl == "run_pnr.tcl" and pnr_log.exists():
        pnr_log.unlink()

    started = time.monotonic()
    with stdout_path.open("w", encoding="utf-8", errors="replace") as stdout:
        proc = subprocess.Popen(
            [str(GW_SH), str(ROOT / args.tcl)],
            cwd=ROOT,
            stdout=stdout,
            stderr=subprocess.STDOUT,
        )
        result = "running"
        detail = ""
        last_size = -1
        last_change = started
        while proc.poll() is None:
            time.sleep(1)
            now = time.monotonic()
            if pnr_log.exists():
                size = pnr_log.stat().st_size
                if size != last_size:
                    last_size = size
                    last_change = now
                text = pnr_log.read_text(encoding="utf-8", errors="replace")
                if (
                    args.tcl == "run_pnr.tcl"
                    and "Physical Constraint parsed completed" in text
                    and "Placement" not in text
                    and now - last_change >= args.stall
                ):
                    result = "stalled"
                    detail = f"No P&R log progress for {args.stall}s after CST parsing"
                    kill_tree(proc.pid)
                    break
            if now - started >= args.timeout:
                result = "timeout"
                detail = f"Hard timeout {args.timeout}s"
                kill_tree(proc.pid)
                break
        if result == "running":
            result = "passed" if proc.returncode == 0 else "failed"
            detail = f"exit={proc.returncode}"
        else:
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                kill_tree(proc.pid)

    elapsed = time.monotonic() - started
    for name in (
        "test.log",
        "cmd.do",
        "test.tr.html",
        "test_tr_content.html",
        "test.rpt.txt",
        "test.timing_paths",
    ):
        src = ROOT / "impl" / "pnr" / name
        if src.exists():
            shutil.copy2(src, run_dir / name)
    if pnr_log.exists():
        shutil.copy2(pnr_log, run_dir / "test.log")
    summary = {
        "name": args.name,
        "result": result,
        "detail": detail,
        "elapsed_seconds": round(elapsed, 2),
        "pid": proc.pid,
        "sdc": args.sdc,
    }
    (run_dir / "result.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
