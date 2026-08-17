"""
Unified E2E CLI Test Runner & Structured JSON Reporter (test_runner.py)

Executes Tier 1-5 end-to-end tests via pytest programmatically, collects quantitative
metrics, outputs structured JSON report to logs/e2e_results.json, and exits with standard code:
  0 = All tests passed
  1 = One or more test assertions failed
  2 = Infrastructure or command line argument error
"""

import os
import sys
import argparse
import json
import time
import datetime
import pytest

# Ensure project root is on sys.path
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)


class E2EJSONReportPlugin:
    """Pytest plugin collecting test execution state and quantitative metrics."""

    def __init__(self):
        self.results = []
        self.start_time = time.time()

    @pytest.hookimpl(hookwrapper=True)
    def pytest_runtest_makereport(self, item, call):
        outcome = yield
        report = outcome.get_result()
        if report.when == "call":
            test_name = item.name
            filepath = str(item.fspath)
            
            tier = 0
            if "test_tier1" in filepath:
                tier = 1
            elif "test_tier2" in filepath:
                tier = 2
            elif "test_tier3" in filepath:
                tier = 3
            elif "test_tier4" in filepath:
                tier = 4
            elif "test_tier5" in filepath:
                tier = 5

            passed = report.passed
            failure_reason = str(report.longrepr) if report.failed else None

            self.results.append({
                "test_id": test_name,
                "name": test_name,
                "tier": tier,
                "passed": passed,
                "duration_sec": round(report.duration, 3),
                "metrics": getattr(item, "user_metrics", {}),
                "failure_reason": failure_reason
            })

    def generate_summary(self) -> dict:
        total = len(self.results)
        passed = sum(1 for r in self.results if r["passed"])
        failed = total - passed
        duration = round(time.time() - self.start_time, 3)
        status = "PASSED" if failed == 0 and total > 0 else "FAILED"

        return {
            "timestamp": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "summary": {
                "total": total,
                "passed": passed,
                "failed": failed,
                "duration_sec": duration,
                "status": status
            },
            "results": self.results
        }



def main():
    parser = argparse.ArgumentParser(description="Unified E2E Test Runner for Decentralized Swarm Recon (Tiers 1-5)")
    parser.add_argument(
        "--tier",
        type=str,
        default="all",
        choices=["all", "1", "2", "3", "4", "5"],
        help="Specify test tier to execute (default: all)"
    )
    parser.add_argument(
        "--output",
        type=str,
        default=os.path.join(PROJECT_ROOT, "logs", "e2e_results.json"),
        help="Output path for structured JSON results report (default: logs/e2e_results.json)"
    )

    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose pytest output"
    )

    args = parser.parse_args()

    # Determine target test files
    test_dir = os.path.dirname(os.path.abspath(__file__))
    pytest_args = []
    if args.verbose:
        pytest_args.append("-v")

    if args.tier == "all":
        pytest_args.append(test_dir)
    else:
        target_file = os.path.join(test_dir, f"test_tier{args.tier}.py")
        if not os.path.exists(target_file):
            print(f"Error: Target test file '{target_file}' does not exist.", file=sys.stderr)
            sys.exit(2)
        pytest_args.append(target_file)

    # Initialize plugin
    plugin = E2EJSONReportPlugin()

    try:
        pytest_code = pytest.main(pytest_args, plugins=[plugin])
    except SystemExit as se:
        pytest_code = int(se.code) if isinstance(se.code, int) else 0
    except Exception as e:
        print(f"Infra Error executing pytest: {e}", file=sys.stderr)
        sys.exit(2)


    # Generate JSON summary
    summary_report = plugin.generate_summary()

    # Save to output file
    if not os.path.isabs(args.output):
        output_path = os.path.abspath(os.path.join(PROJECT_ROOT, args.output))
    else:
        output_path = os.path.abspath(args.output)
        
    try:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(summary_report, f, indent=2)
            f.flush()
            os.fsync(f.fileno())
        print(f"[TEST RUNNER DEBUG] CWD: {os.getcwd()} | Written to: {output_path} | File Exists: {os.path.exists(output_path)}")
    except Exception as ex:
        print(f"[TEST RUNNER ERROR] Failed to write JSON report to {output_path}: {ex}", file=sys.stderr)


    print(f"\n=======================================================")
    print(f"E2E Test Execution Summary:")
    print(f"Status: {summary_report['summary']['status']}")
    print(f"Total: {summary_report['summary']['total']} | Passed: {summary_report['summary']['passed']} | Failed: {summary_report['summary']['failed']}")
    print(f"Duration: {summary_report['summary']['duration_sec']}s")
    print(f"Results report written to: {output_path}")
    print(f"=======================================================\n")


    if summary_report["summary"]["failed"] > 0 or summary_report["summary"]["total"] == 0:
        sys.exit(1)
    else:
        sys.exit(0)


if __name__ == "__main__":
    main()
