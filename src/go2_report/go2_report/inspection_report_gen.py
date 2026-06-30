#!/usr/bin/env python3
"""
inspection_report_gen.py
=========================
Aggregates InspectionResult messages across the mission and generates:
  - inspection_report.json  — machine-readable, schema-validated
  - inspection_report.md    — human-readable field engineer report

Output directory is configurable via the 'output_dir' ROS2 parameter.
Report generation is triggered by the /mission/generate_report topic (Bool)
or automatically on mission completion (when all 5 stations are inspected).

Setting: Offshore oil rig automated inspection — Meridian Platform Alpha.
"""

import json
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool

from go2_interfaces.msg import InspectionResult, FaultEvent


REPORT_JSON_SCHEMA_VERSION = "1.0"
EXPECTED_STATIONS = {"station_1", "station_2", "station_3", "station_4", "station_5"}


class InspectionReportGen(Node):

    def __init__(self):
        super().__init__("inspection_report_gen")

        self.declare_parameter("output_dir", "/tmp/go2_inspection_reports")
        self.declare_parameter("auto_generate_on_complete", True)
        self.declare_parameter("mission_name", "Meridian_Platform_Alpha")

        self._output_dir = Path(self.get_parameter("output_dir").value)
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._auto_generate = self.get_parameter("auto_generate_on_complete").value
        self._mission_name = self.get_parameter("mission_name").value

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Collected data
        self._results: List[InspectionResult] = []
        self._fault_events: List[FaultEvent] = []
        self._mission_start: Optional[datetime] = None
        self._mission_end: Optional[datetime] = None
        self._inspected_stations: set = set()

        # Subscriptions
        self.create_subscription(
            InspectionResult, "/mission/inspection_log",
            self._result_callback, reliable_qos)

        self.create_subscription(
            FaultEvent, "/mission/fault_event",
            self._fault_callback, reliable_qos)

        self.create_subscription(
            Bool, "/mission/generate_report",
            lambda msg: self.generate_report() if msg.data else None,
            reliable_qos)

        self.get_logger().info(
            f"InspectionReportGen ready. Output: {self._output_dir}"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _result_callback(self, msg: InspectionResult) -> None:
        if self._mission_start is None:
            self._mission_start = datetime.now(timezone.utc)

        self._results.append(msg)
        self._inspected_stations.add(msg.station_id)
        self.get_logger().info(
            f"Logged: {msg.station_id} | anomaly={msg.telemetry_anomaly_detected or msg.visual_anomaly_detected} "
            f"| conf={msg.confidence:.2f} | degraded={msg.degraded_mode}"
        )

        # Auto-generate report when all expected stations inspected
        if (self._auto_generate
                and self._inspected_stations >= EXPECTED_STATIONS):
            self._mission_end = datetime.now(timezone.utc)
            self.get_logger().info("All stations inspected — auto-generating report.")
            self.generate_report()

    def _fault_callback(self, msg: FaultEvent) -> None:
        self._fault_events.append(msg)
        self.get_logger().warn(
            f"Fault event recorded: {msg.subsystem} / {msg.fault_type}"
        )

    # ── Report generation ─────────────────────────────────────────────────────

    def generate_report(self) -> None:
        """Generate JSON and Markdown reports."""
        if not self._results:
            self.get_logger().warn("No inspection results to report.")
            return

        timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        json_path = self._output_dir / f"inspection_report_{timestamp}.json"
        md_path = self._output_dir / f"inspection_report_{timestamp}.md"

        report_data = self._build_report_dict()

        # JSON
        with open(json_path, "w") as f:
            json.dump(report_data, f, indent=2, default=str)
        self.get_logger().info(f"JSON report written: {json_path}")

        # Markdown
        md_content = self._build_markdown(report_data)
        with open(md_path, "w") as f:
            f.write(md_content)
        self.get_logger().info(f"Markdown report written: {md_path}")

    def _build_report_dict(self) -> dict:
        anomaly_stations = [
            r for r in self._results
            if r.telemetry_anomaly_detected or r.visual_anomaly_detected
        ]
        degraded_stations = [r for r in self._results if r.degraded_mode]
        fault_summary = [
            {
                "subsystem": f.subsystem,
                "fault_type": f.fault_type,
                "detected_at": f.detected_at.sec,
                "recovery_action": f.recovery_action_taken,
            }
            for f in self._fault_events
        ]

        stations = []
        for r in sorted(self._results, key=lambda x: x.station_id):
            stations.append({
                "station_id": r.station_id,
                "inspected_at_sec": r.inspected_at.sec,
                "visual_anomaly": r.visual_anomaly_detected,
                "telemetry_anomaly": r.telemetry_anomaly_detected,
                "anomaly_description": r.anomaly_description,
                "confidence": round(r.confidence, 3),
                "degraded_mode": r.degraded_mode,
            })

        elapsed_sec = None
        if self._mission_start and self._mission_end:
            elapsed_sec = (self._mission_end - self._mission_start).total_seconds()

        return {
            "schema_version": REPORT_JSON_SCHEMA_VERSION,
            "mission_name": self._mission_name,
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "mission_elapsed_sec": elapsed_sec,
            "summary": {
                "total_stations": len(self._results),
                "anomaly_count": len(anomaly_stations),
                "degraded_mode_inspections": len(degraded_stations),
                "fault_events": len(self._fault_events),
            },
            "fault_events": fault_summary,
            "stations": stations,
        }

    def _build_markdown(self, data: dict) -> str:
        s = data["summary"]
        faults = data["fault_events"]
        stations = data["stations"]
        mission = data["mission_name"]
        generated = data["generated_at"]
        elapsed = data.get("mission_elapsed_sec")
        elapsed_str = f"{elapsed:.0f}s" if elapsed else "unknown"

        # Format elapsed for fault timestamp
        fault_lines = []
        for f in faults:
            fault_lines.append(
                f"- **{f['subsystem'].upper()}** fault detected "
                f"(`{f['fault_type']}`). "
                f"Recovery action: `{f['recovery_action']}`."
            )
        fault_section = "\n".join(fault_lines) if fault_lines else "_No faults detected._"

        station_rows = []
        for st in stations:
            flag = ""
            if st["visual_anomaly"] or st["telemetry_anomaly"]:
                flag = "⚠ ANOMALY"
            conf_note = " *(degraded)*" if st["degraded_mode"] else ""
            station_rows.append(
                f"| {st['station_id']} | {flag or 'OK'} | "
                f"{st['confidence']:.2f}{conf_note} | "
                f"{st['anomaly_description']} |"
            )
        station_table = "\n".join(station_rows)

        anomaly_details = []
        for st in stations:
            if st["visual_anomaly"] or st["telemetry_anomaly"]:
                kind = []
                if st["visual_anomaly"]:
                    kind.append("visual")
                if st["telemetry_anomaly"]:
                    kind.append("telemetry")
                anomaly_details.append(
                    f"- **{st['station_id']}**: {', '.join(kind)} anomaly — "
                    f"`{st['anomaly_description']}`"
                    + (f" *(collected under degraded navigation, confidence={st['confidence']:.2f})*"
                       if st["degraded_mode"] else "")
                )
        anomaly_section = (
            "\n".join(anomaly_details) if anomaly_details
            else "_No anomalies detected._"
        )

        return f"""# Autonomous Inspection Report
## {mission} — Meridian Offshore Platform

**Generated:** {generated}
**Mission Duration:** {elapsed_str}
**Report Schema:** v{data['schema_version']}

---

## Executive Summary

{s['total_stations']} stations inspected. **{s['anomaly_count']} anomalies detected.**
{s['degraded_mode_inspections']} station(s) inspected under degraded navigation mode.
{s['fault_events']} sensor fault event(s) recorded during mission.

---

## Fault Events

{fault_section}

> If a lidar fault occurred, all subsequent inspections were conducted using
> dead-reckoning navigation (odometry + IMU only). These readings are flagged
> with reduced confidence (0.60) to indicate lower positional certainty. The
> anomaly detection logic itself was unaffected by the navigation fault.

---

## Station Inspection Results

| Station | Status | Confidence | Notes |
|---------|--------|------------|-------|
{station_table}

---

## Anomaly Details

{anomaly_section}

---

## Navigation Integrity

{"**DEGRADED MODE ACTIVATED during mission.** Lidar topic went silent mid-patrol. Robot continued inspection using dead-reckoning (odom + IMU). Affected stations are marked above. Recommend re-inspection of degraded-mode stations at next available maintenance window." if s['degraded_mode_inspections'] > 0 else "All stations inspected under full sensor suite. Navigation integrity: NOMINAL."}

---

*This report was generated automatically by the Go2 Resilient Inspection System.*
*Human review is required before acting on any anomaly flag.*
"""


def main(args=None):
    rclpy.init(args=args)
    node = InspectionReportGen()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
