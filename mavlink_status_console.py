#!/usr/bin/env python3
"""
Standalone MAVLink telemetry status console for debugging QGC-style message loss.

Edit the constants below, then run:

    python3 tools/mavlink_status_console.py

The loss counters intentionally mirror QGroundControl's Telemetry settings page:
successfully decoded MAVLink messages are counted, and loss is inferred from
MAVLink sequence gaps per (sysid, compid).
"""

from __future__ import annotations

import csv
import os
import sys
import time
from collections import defaultdict, deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Deque, Dict, Iterable, Optional, Tuple

# ---------------------------------------------------------------------------
# Hardcoded settings to tweak while debugging
# ---------------------------------------------------------------------------

# Options: "udp_listen", "udp_out", "tcp_client", "tcp_server"
LINK_MODE = "udp_listen"

UDP_BIND_HOST = "0.0.0.0"
UDP_PORT = 14552
UDP_TARGET_HOST = "127.0.0.1"

TCP_HOST = "127.0.0.1"
TCP_PORT = 5760

MAVLINK_DIALECT = "ardupilotmega"
GCS_SYSTEM_ID = 255
GCS_COMPONENT_ID = 190  # MAV_COMP_ID_MISSIONPLANNER, same component used by QGC

SEND_GCS_HEARTBEAT = True
GCS_HEARTBEAT_HZ = 1.0

# Leave these disabled for passive observation. Enable only when you want this
# tool to actively change stream rates for comparison testing.
REQUEST_DATA_STREAM = False
REQUEST_DATA_STREAM_RATE_HZ = 10
SET_MESSAGE_INTERVALS_HZ = {
    # message_id: hz,
    # 0: 1,    # HEARTBEAT
    # 30: 20,  # ATTITUDE
}

DISPLAY_PERIOD_S = 1.0
RATE_WINDOW_S = 10.0
HEARTBEAT_STALE_S = 3.5  # QGC VehicleLinkManager comm-loss threshold
TOP_MESSAGE_COUNT = 14
RECENT_GAP_COUNT = 10
WARN_BACKWARD_OR_RESET_GAP = 128

# Set to None to disable. Relative paths are created from the current directory.
EVENT_LOG_PATH = "mavlink_status_events.log"
CSV_STATUS_PATH = "mavlink_status_samples.csv"

# ---------------------------------------------------------------------------

os.environ.setdefault("MAVLINK20", "1")

try:
    from pymavlink import mavutil
except ImportError as exc:
    raise SystemExit(
        "pymavlink is required. Install it with: python3 -m pip install pymavlink"
    ) from exc

SourceKey = Tuple[int, int]
MsgKey = Tuple[int, str]


@dataclass
class SourceStats:
    received: int = 0
    loss: int = 0
    last_seq: Optional[int] = None
    last_msg_time: float = 0.0
    last_msg_name: str = ""
    gap_events: int = 0
    max_gap: int = 0
    suspicious_gaps: int = 0
    interarrival_max_s: float = 0.0


@dataclass
class MessageStats:
    count: int = 0
    last_time: float = 0.0
    times: Deque[float] = field(default_factory=deque)


@dataclass
class GapEvent:
    when: float
    source: SourceKey
    msg_name: str
    prev_seq: Optional[int]
    expected_seq: int
    actual_seq: int
    lost: int
    dt_s: Optional[float]


class QgcLossTracker:
    def __init__(self) -> None:
        self.total_received = 0
        self.total_loss = 0
        self.running_loss_percent = 0.0
        self.sources: Dict[SourceKey, SourceStats] = defaultdict(SourceStats)
        self.messages: Dict[MsgKey, MessageStats] = defaultdict(MessageStats)
        self.msg_times: Deque[float] = deque()
        self.loss_times: Deque[Tuple[float, int]] = deque()
        self.recent_gaps: Deque[GapEvent] = deque(maxlen=RECENT_GAP_COUNT)
        self.heartbeats: Dict[SourceKey, float] = {}
        self.first_vehicle: Optional[SourceKey] = None
        self.first_msg_time: Optional[float] = None
        self.last_msg_time: Optional[float] = None

    def update(self, message, now: float) -> Optional[GapEvent]:
        sysid = int(message.get_srcSystem())
        compid = int(message.get_srcComponent())
        seq = int(message.get_seq())
        msg_name = message.get_type()
        source = (sysid, compid)
        stats = self.sources[source]
        previous_msg_time = stats.last_msg_time

        if self.first_msg_time is None:
            self.first_msg_time = now
        if previous_msg_time:
            stats.interarrival_max_s = max(stats.interarrival_max_s, now - previous_msg_time)

        self.total_received += 1
        stats.received += 1
        stats.last_msg_time = now
        stats.last_msg_name = msg_name
        self.last_msg_time = now
        self.msg_times.append(now)

        expected_seq = seq if stats.last_seq is None else (stats.last_seq + 1) & 0xFF
        lost = seq - expected_seq if seq >= expected_seq else seq + 256 - expected_seq

        gap_event = None
        if lost:
            self.total_loss += lost
            stats.loss += lost
            stats.gap_events += 1
            stats.max_gap = max(stats.max_gap, lost)
            self.loss_times.append((now, lost))
            if lost >= WARN_BACKWARD_OR_RESET_GAP:
                stats.suspicious_gaps += 1
            gap_event = GapEvent(
                when=now,
                source=source,
                msg_name=msg_name,
                prev_seq=stats.last_seq,
                expected_seq=expected_seq,
                actual_seq=seq,
                lost=lost,
                dt_s=(None if not previous_msg_time else now - previous_msg_time),
            )
            self.recent_gaps.append(gap_event)

        stats.last_seq = seq

        total_sent = self.total_received + self.total_loss
        current_loss_percent = (self.total_loss / total_sent) * 100.0 if total_sent else 0.0
        self.running_loss_percent = (current_loss_percent + self.running_loss_percent) * 0.5

        msg_id = int(message.get_msgId())
        msg_stats = self.messages[(msg_id, msg_name)]
        msg_stats.count += 1
        msg_stats.last_time = now
        msg_stats.times.append(now)

        if msg_name == "HEARTBEAT":
            self.heartbeats[source] = now
            if self.first_vehicle is None and sysid != GCS_SYSTEM_ID:
                self.first_vehicle = source

        self._trim_windows(now)
        return gap_event

    @property
    def total_sent_computed(self) -> int:
        return self.total_received + self.total_loss

    @property
    def cumulative_loss_percent(self) -> float:
        total_sent = self.total_sent_computed
        return (self.total_loss / total_sent) * 100.0 if total_sent else 0.0

    def recent_msg_rate(self, now: float) -> float:
        self._trim_windows(now)
        return len(self.msg_times) / RATE_WINDOW_S

    def recent_loss_rate(self, now: float) -> float:
        self._trim_windows(now)
        return sum(lost for _, lost in self.loss_times) / RATE_WINDOW_S

    def top_message_rates(self, now: float) -> Iterable[Tuple[float, int, str, int, float]]:
        self._trim_windows(now)
        rows = []
        for (msg_id, name), stats in self.messages.items():
            rows.append((len(stats.times) / RATE_WINDOW_S, msg_id, name, stats.count, now - stats.last_time))
        return sorted(rows, reverse=True)[:TOP_MESSAGE_COUNT]

    def _trim_windows(self, now: float) -> None:
        cutoff = now - RATE_WINDOW_S
        while self.msg_times and self.msg_times[0] < cutoff:
            self.msg_times.popleft()
        while self.loss_times and self.loss_times[0][0] < cutoff:
            self.loss_times.popleft()
        for stats in self.messages.values():
            while stats.times and stats.times[0] < cutoff:
                stats.times.popleft()


def connection_string() -> str:
    if LINK_MODE == "udp_listen":
        return f"udpin:{UDP_BIND_HOST}:{UDP_PORT}"
    if LINK_MODE == "udp_out":
        return f"udpout:{UDP_TARGET_HOST}:{UDP_PORT}"
    if LINK_MODE == "tcp_client":
        return f"tcp:{TCP_HOST}:{TCP_PORT}"
    if LINK_MODE == "tcp_server":
        return f"tcpin:{TCP_HOST}:{TCP_PORT}"
    raise ValueError(f"Unsupported LINK_MODE: {LINK_MODE}")


def open_event_log() -> Optional[object]:
    if not EVENT_LOG_PATH:
        return None
    path = Path(EVENT_LOG_PATH)
    log_file = path.open("a", encoding="utf-8", buffering=1)
    log_file.write(f"\n# start {time.strftime('%Y-%m-%d %H:%M:%S')} connection={connection_string()}\n")
    return log_file


def open_csv_log() -> Tuple[Optional[object], Optional[csv.DictWriter]]:
    if not CSV_STATUS_PATH:
        return None, None
    file_obj = Path(CSV_STATUS_PATH).open("a", encoding="utf-8", newline="", buffering=1)
    writer = csv.DictWriter(
        file_obj,
        fieldnames=[
            "unix_time",
            "total_sent_computed",
            "total_received",
            "total_loss",
            "qgc_running_loss_percent",
            "cumulative_loss_percent",
            "recent_msg_rate_hz",
            "recent_loss_rate_hz",
            "parser_receive_errors",
            "bytes_received",
            "active_heartbeat_age_s",
        ],
    )
    if file_obj.tell() == 0:
        writer.writeheader()
    return file_obj, writer


def send_heartbeat(master) -> None:
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        mavutil.mavlink.MAV_MODE_MANUAL_ARMED,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def send_stream_requests(master, target_system: int, target_component: int) -> None:
    if REQUEST_DATA_STREAM:
        master.mav.request_data_stream_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            REQUEST_DATA_STREAM_RATE_HZ,
            1,
        )

    for msg_id, hz in SET_MESSAGE_INTERVALS_HZ.items():
        interval_us = int(1_000_000 / hz) if hz > 0 else -1
        master.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )


def clear_screen() -> None:
    sys.stdout.write("\033[2J\033[H")


def fmt_age(age: Optional[float]) -> str:
    if age is None:
        return "n/a"
    return f"{age:5.1f}s"


def write_gap_event(log_file, event: GapEvent) -> None:
    if not log_file:
        return
    stamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(event.when))
    sysid, compid = event.source
    marker = " suspicious" if event.lost >= WARN_BACKWARD_OR_RESET_GAP else ""
    log_file.write(
        f"{stamp} gap sys={sysid} comp={compid} msg={event.msg_name} "
        f"prev={event.prev_seq} expected={event.expected_seq} got={event.actual_seq} "
        f"lost={event.lost}{marker}\n"
    )


def render(master, tracker: QgcLossTracker, start_time: float, last_bytes: Tuple[int, float]) -> Tuple[int, float]:
    now = time.monotonic()
    mav = master.mav
    bytes_received = int(getattr(mav, "total_bytes_received", 0))
    previous_bytes, previous_time = last_bytes
    dt = max(now - previous_time, 0.001)
    byte_rate = (bytes_received - previous_bytes) / dt
    parser_errors = int(getattr(mav, "total_receive_errors", 0))
    parser_packets = int(getattr(mav, "total_packets_received", 0))

    active = tracker.first_vehicle
    active_hb_age = None
    if active is not None and active in tracker.heartbeats:
        active_hb_age = now - tracker.heartbeats[active]

    clear_screen()
    print("MAVLink Telemetry Status Console")
    print("=" * 79)
    print(f"Connection: {connection_string()}  dialect={MAVLINK_DIALECT}  uptime={now - start_time:0.1f}s")
    print(f"GCS heartbeat: {'on' if SEND_GCS_HEARTBEAT else 'off'}  source={GCS_SYSTEM_ID}.{GCS_COMPONENT_ID}")
    print()
    print("QGC-style Link Status")
    print(f"  Total messages sent (computed): {tracker.total_sent_computed}")
    print(f"  Total messages received:        {tracker.total_received}")
    print(f"  Total message loss:             {tracker.total_loss}")
    print(f"  Loss rate shown by QGC:         {tracker.running_loss_percent:0.1f}%")
    print(f"  Cumulative loss rate:           {tracker.cumulative_loss_percent:0.1f}%")
    print()
    print("Transport / Parser")
    print(f"  MAVLink parser packets:         {parser_packets}")
    print(f"  MAVLink parser receive errors:  {parser_errors}")
    print(f"  Bytes received:                 {bytes_received} ({byte_rate:0.0f} B/s)")
    print(f"  Recent window:                  {tracker.recent_msg_rate(now):0.1f} msg/s, {tracker.recent_loss_rate(now):0.1f} lost-msg/s over {RATE_WINDOW_S:0.0f}s")
    print()

    hb_state = "no vehicle heartbeat yet"
    if active is not None:
        stale = active_hb_age is not None and active_hb_age > HEARTBEAT_STALE_S
        hb_state = f"active heartbeat {active[0]}.{active[1]} age={fmt_age(active_hb_age)}"
        if stale:
            hb_state += "  STALE"
    print(f"Heartbeat: {hb_state}")
    if tracker.last_msg_time is None:
        print("Last message: none")
    else:
        print(f"Last message age: {now - tracker.last_msg_time:0.2f}s")

    print()
    print("Per Source")
    print(" sys comp  recv    lost  loss% gaps max suspicious last_seq age    last_msg")
    for (sysid, compid), stats in sorted(tracker.sources.items()):
        sent = stats.received + stats.loss
        loss_pct = (stats.loss / sent) * 100.0 if sent else 0.0
        age = now - stats.last_msg_time if stats.last_msg_time else None
        print(
            f"{sysid:4d} {compid:4d} {stats.received:6d} {stats.loss:7d} "
            f"{loss_pct:5.1f} {stats.gap_events:4d} {stats.max_gap:3d} "
            f"{stats.suspicious_gaps:10d} {str(stats.last_seq):>7} {fmt_age(age)} {stats.last_msg_name}"
        )

    print()
    print(f"Top Message Rates ({RATE_WINDOW_S:0.0f}s window)")
    print("  rate     msgid name                         total age")
    for rate, msg_id, name, count, age in tracker.top_message_rates(now):
        print(f"{rate:6.1f}/s {msg_id:5d} {name[:28]:28s} {count:6d} {age:4.1f}s")

    print()
    print("Recent Sequence Gaps")
    if not tracker.recent_gaps:
        print("  none")
    else:
        for event in tracker.recent_gaps:
            age = now - event.when
            sysid, compid = event.source
            marker = " suspicious" if event.lost >= WARN_BACKWARD_OR_RESET_GAP else ""
            print(
                f"  {age:5.1f}s ago sys={sysid}.{compid} msg={event.msg_name} "
                f"prev={event.prev_seq} expected={event.expected_seq} got={event.actual_seq} "
                f"lost={event.lost}{marker}"
            )

    print()
    print("Debug hints")
    print(f"  Gap >= {WARN_BACKWARD_OR_RESET_GAP} is often component reboot, seq reset, or out-of-order traffic.")
    print("  Parser errors mean bad MAVLink frames; QGC's settings loss counter only sees decoded frames.")
    print("  Ctrl-C to stop. Edit constants at the top of this file to change links or debug behavior.")
    sys.stdout.flush()

    return bytes_received, now


def main() -> int:
    device = connection_string()
    print(f"Opening {device} ...")
    master = mavutil.mavlink_connection(
        device,
        source_system=GCS_SYSTEM_ID,
        source_component=GCS_COMPONENT_ID,
        dialect=MAVLINK_DIALECT,
        robust_parsing=True,
        autoreconnect=True,
        force_connected=True,
    )

    tracker = QgcLossTracker()
    event_log = open_event_log()
    csv_file, csv_writer = open_csv_log()
    start_time = time.monotonic()
    last_render = 0.0
    last_heartbeat = 0.0
    last_bytes = (0, start_time)
    stream_requests_sent: set[SourceKey] = set()

    try:
        while True:
            now = time.monotonic()

            if SEND_GCS_HEARTBEAT and now - last_heartbeat >= 1.0 / GCS_HEARTBEAT_HZ:
                try:
                    send_heartbeat(master)
                except Exception as exc:
                    if event_log:
                        event_log.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} heartbeat_send_error {exc}\n")
                last_heartbeat = now

            message = master.recv_match(blocking=False)
            if message is not None:
                if message.get_type() == "BAD_DATA":
                    continue

                gap_event = tracker.update(message, now)
                if gap_event is not None:
                    write_gap_event(event_log, gap_event)

                if message.get_type() == "HEARTBEAT":
                    source = (int(message.get_srcSystem()), int(message.get_srcComponent()))
                    if source not in stream_requests_sent and source[0] != GCS_SYSTEM_ID:
                        send_stream_requests(master, source[0], source[1])
                        stream_requests_sent.add(source)
                continue

            if now - last_render >= DISPLAY_PERIOD_S:
                last_bytes = render(master, tracker, start_time, last_bytes)
                if csv_writer:
                    active = tracker.first_vehicle
                    active_hb_age = ""
                    if active is not None and active in tracker.heartbeats:
                        active_hb_age = f"{now - tracker.heartbeats[active]:0.3f}"
                    csv_writer.writerow(
                        {
                            "unix_time": f"{time.time():0.3f}",
                            "total_sent_computed": tracker.total_sent_computed,
                            "total_received": tracker.total_received,
                            "total_loss": tracker.total_loss,
                            "qgc_running_loss_percent": f"{tracker.running_loss_percent:0.3f}",
                            "cumulative_loss_percent": f"{tracker.cumulative_loss_percent:0.3f}",
                            "recent_msg_rate_hz": f"{tracker.recent_msg_rate(now):0.3f}",
                            "recent_loss_rate_hz": f"{tracker.recent_loss_rate(now):0.3f}",
                            "parser_receive_errors": int(getattr(master.mav, "total_receive_errors", 0)),
                            "bytes_received": int(getattr(master.mav, "total_bytes_received", 0)),
                            "active_heartbeat_age_s": active_hb_age,
                        }
                    )
                last_render = now

            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if event_log:
            event_log.close()
        if csv_file:
            csv_file.close()
        try:
            master.close()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
