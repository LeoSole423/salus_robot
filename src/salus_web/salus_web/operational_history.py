"""Persistent operational history. ROS callbacks submit bounded, asynchronous writes."""
from concurrent.futures import ThreadPoolExecutor
from contextlib import closing
from datetime import datetime, timezone
from pathlib import Path
from threading import BoundedSemaphore, Lock
from time import monotonic

import json
import math
import re
import sqlite3
import uuid


START = {"PATROL_MISSION_STARTED", "ROUTE_LOOP_STARTED"}
FINISH = {"PATROL_AT_HOME": "home", "PATROL_MISSION_CANCELLED": "cancelled",
          "ROUTE_LOOP_CANCELLED": "cancelled", "ROUTE_LOOP_REPLACED": "replaced",
          "ROUTE_LOOP_COMPLETED": "completed"}
CHECKPOINT = "ROUTE_CHECKPOINT_REACHED"
STOP_CODES = {"BRAKE_REQUESTED", "SAFETY_SCAN_STALE", "PATH_HEALTH_STOP",
              "MANUAL_WATCHDOG_STOP", "COLLISION_STOP", "ESTOP_ACTIVE"}
PUBLIC_DETAILS = {"enabled", "phase", "route_status", "loop_index", "exit_index",
                  "brake_pct", "duration_s", "reason_code", "failure_code", "ros_stamp_ns",
                  "stop_signal", "input_index", "loop_iteration", "route_mission_id",
                  "gps_lat", "gps_lon", "gps_received_at", "gps_age_s", "gps_stamp_ns"}
STATE_EVENTS = {"SAFETY_SCAN_STALE", "PATH_STOP_AND_WAIT", "PATH_HEALTH_STOP"}
ROUTE_BLOCK_REASONS = {"PLANNER_U_TURN", "PLANNER_LOOP", "MANEUVER_GEOMETRY_INVALID",
                       "NO_VALID_PATH", "NAV_ABORTED"}


class OperationalHistory:
    def __init__(self, path, robot, log_error):
        self._path = Path(path)
        self._robot = robot
        self._log_error = log_error
        self._lock = Lock()
        self._slots = BoundedSemaphore(256)
        self._fault = None
        self._observations = {}
        self._sampled_stop = set()
        self._path_block = None
        self._drive_stop = None
        self._gps_sample = None
        self._last_events = {}
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="operational-history")
        self._worker.submit(self._initialize)

    def _connect(self):
        db = sqlite3.connect(self._path, timeout=2)
        db.row_factory = sqlite3.Row
        return db

    def _fail(self):
        with self._lock:
            if self._fault is None:
                self._fault = "history_incomplete"
                self._log_error("Operational history incomplete. Check storage and restart the gateway.")

    def _initialize(self):
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            with closing(self._connect()) as db, db:
                db.execute("PRAGMA journal_mode=WAL")
                db.executescript("""
                    CREATE TABLE IF NOT EXISTS patrols (
                        id TEXT PRIMARY KEY, robot TEXT NOT NULL, mission_id TEXT NOT NULL,
                        started_at TEXT NOT NULL, ended_at TEXT, status TEXT NOT NULL,
                        duration_s REAL NOT NULL DEFAULT 0, circuit_s REAL NOT NULL DEFAULT 0,
                        phase TEXT, last_tick REAL NOT NULL
                    );
                    CREATE TABLE IF NOT EXISTS history_events (
                        sequence INTEGER PRIMARY KEY, robot TEXT NOT NULL,
                        timestamp TEXT NOT NULL, patrol_id TEXT, category TEXT NOT NULL,
                        severity INTEGER NOT NULL, component TEXT NOT NULL,
                        code TEXT NOT NULL, details TEXT NOT NULL
                    );
                    CREATE INDEX IF NOT EXISTS history_event_robot_date
                        ON history_events(robot, timestamp);
                    CREATE INDEX IF NOT EXISTS history_event_patrol
                        ON history_events(robot, patrol_id, sequence);
                    CREATE INDEX IF NOT EXISTS history_patrol_robot_date
                        ON patrols(robot, started_at);
                """)
                db.execute("UPDATE patrols SET status='interrupted' "
                           "WHERE robot=? AND ended_at IS NULL AND status!='interrupted'", (self._robot,))
        except (OSError, sqlite3.Error):
            self._fail()

    def close(self):
        self._worker.shutdown(wait=True)

    def observe_gps(self, *, valid, lat, lon, stamp_ns):
        self._gps_sample = None
        if valid and math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180:
            received = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
            self._gps_sample = lat, lon, received, monotonic(), stamp_ns

    def _checkpoint_gps(self, event_stamp):
        sample = self._gps_sample
        if sample is None:
            return {}
        lat, lon, received, tick, stamp = sample
        age = monotonic() - tick
        if stamp > 0 and event_stamp > 0:
            source_age = (event_stamp - stamp) / 1e9
            if source_age < 0:
                return {}
            age = max(age, source_age)
        if not 0 <= age <= 2:
            return {}
        return {"gps_lat": lat, "gps_lon": lon, "gps_received_at": received,
                "gps_age_s": round(age, 3), "gps_stamp_ns": stamp}

    def nav_event(self, component, code, severity, details):
        if component == "incident_recorder" and code == "NAVIGATION_INCIDENT":
            self.record(component, code, "navigation", severity, details,
                        mission_id=details.get("mission_id") or None)
            return
        if code == CHECKPOINT and component == "route_executor":
            route_id = details.get("mission_id")
            if route_id and "input_index" in details and "loop_iteration" in details:
                gps = self._checkpoint_gps(int(details.get("ros_stamp_ns", 0)))
                self.record(component, code, "waypoint", severity, {
                    "input_index": details["input_index"],
                    "loop_iteration": details["loop_iteration"],
                    "ros_stamp_ns": details.get("ros_stamp_ns", 0),
                    "route_mission_id": route_id, **gps,
                }, mission_id=details.get("parent_patrol_id") or route_id)
            return
        if code in STATE_EVENTS:
            return  # Captured as a transition from authoritative navigation telemetry.
        if not (severity >= 1 or code.startswith(("PATROL_", "ROUTE_LOOP_")) or code in
                {"MANUAL_MODE_CHANGED", "RETURN_HOME_REQUESTED", "GOAL_RESULT_SUCCEEDED",
                 "ROUTE_BLOCKED_CLEARED"}):
            return
        category = ("mode" if code == "MANUAL_MODE_CHANGED" else
                    "stop" if code in STOP_CODES else
                    "fault" if severity >= 2 else
                    "patrol" if code.startswith(("PATROL_", "BATTERY_", "ROUTE_LOOP_")) or code == "RETURN_HOME_REQUESTED"
                    else "navigation")
        if code.startswith("ROUTE_BLOCKED_") and details.get("reason") in ROUTE_BLOCK_REASONS:
            details = {**details, "reason_code": details["reason"]}
        mission_id = details.get("mission_id") if component == "patrol_mission_coordinator" or code.startswith("ROUTE_LOOP_") else None
        self.record(component, code, category, severity, details, mission_id=mission_id)

    def observe(self, key, value, component, changes, *, initial=False, details=None):
        """Publish only observed state edges, optionally recording the initial state."""
        previous = self._observations.get(key)
        self._observations[key] = value
        if previous == value or (previous is None and not initial) or value not in changes:
            return
        code, category, severity = changes[value]
        self.record(component, code, category, severity, details)

    def observe_path_health(self, *, blocked, healthy, reason, ros_stamp_ns):
        if not blocked:
            if healthy and self._path_block is not None:
                self.record("path_health", "PATH_HEALTH_RECOVERED", "stop", details={
                    "reason_code": self._path_block, "ros_stamp_ns": ros_stamp_ns,
                })
                self._path_block = None
            return
        safe_reason = reason if re.fullmatch(r"[a-zA-Z0-9_:]{1,100}", reason) else ""
        if safe_reason == self._path_block:
            return
        self._path_block = safe_reason
        pending = safe_reason == "new_goal_pending_path"
        self.record("path_health", "PATH_STOP_AND_WAIT", "stop", 0 if pending else 1, {
            "reason_code": safe_reason, "ros_stamp_ns": ros_stamp_ns,
        })

    def observe_drive_stop(self, *, brake_pct, stop_signal):
        previous = self._drive_stop
        braking = brake_pct > 0
        standalone_stop = stop_signal and not braking
        self._drive_stop = (braking, stop_signal, standalone_stop)
        details = {"brake_pct": brake_pct, "stop_signal": stop_signal}
        if braking and (previous is None or not previous[0]):
            self.record("controller", "BRAKE_APPLIED", "stop", details=details)
        elif not braking and previous and previous[0]:
            self.record("controller", "BRAKE_RELEASED", "stop", details=details)
        if standalone_stop and (previous is None or not previous[2]):
            self.record("controller", "SAFETY_STOP_SIGNAL_ACTIVE", "stop", 2, details)
        elif previous and previous[1] != stop_signal:
            brake_just_applied = braking and not previous[0]
            brake_just_released = not braking and previous[0] and not previous[2]
            if not brake_just_applied and not brake_just_released:
                self.record("controller", "SAFETY_STOP_SIGNAL_CHANGED", "stop", details=details)

    def checkpoint(self):
        if self._slots.acquire(blocking=False):
            with self._lock:
                self._worker.submit(self._checkpoint, monotonic())

    def _checkpoint(self, tick):
        try:
            with closing(self._connect()) as db, db:
                db.execute(
                    "UPDATE patrols SET duration_s=duration_s+MAX(0,?-last_tick), "
                    "circuit_s=circuit_s+CASE WHEN phase='PATROL' THEN MAX(0,?-last_tick) ELSE 0 END, "
                    "last_tick=? WHERE robot=? AND ended_at IS NULL AND status IN ('active','paused')",
                    (tick, tick, tick, self._robot),
                )
        except (OSError, sqlite3.Error):
            self._fail()
        finally:
            self._slots.release()

    def observe_navigation_stop(self, causes):
        for code in sorted(causes - self._sampled_stop):
            self.record("navigation", code, "stop", 2)
        for code in sorted(self._sampled_stop - causes):
            self.record("navigation", "NAVIGATION_BLOCK_CLEARED", "stop", 0, {"reason_code": code})
        self._sampled_stop = causes

    def record(self, component, code, category, severity=0, details=None, *, mission_id=None):
        if not self._slots.acquire(blocking=False):
            self._fail()
            return
        public_fields = PUBLIC_DETAILS
        if component == "incident_recorder" and code == "NAVIGATION_INCIDENT":
            public_fields = public_fields | {"id", "status", "trigger_ns"}
        safe_details = {key: str(value)[:100] for key, value in (details or {}).items()
                        if key in public_fields}
        with self._lock:
            timestamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
            self._worker.submit(self._write, timestamp, monotonic(), component[:100], code[:100],
                                category, severity, safe_details, mission_id)

    def _write(self, timestamp, tick, component, code, category, severity, details, mission_id):
        try:
            with closing(self._connect()) as db, db:
                recent = self._last_events.get(component)
                fingerprint = (code, severity, {k: v for k, v in details.items() if k != "ros_stamp_ns"})
                if ((category in {"fault", "navigation"} or code == "PATH_HEALTH_STOP") and not code.startswith(("PATROL_", "ROUTE_LOOP_"))
                        and recent and recent[0] == fingerprint and tick - recent[1] < 2):
                    return
                active = db.execute(
                    "SELECT * FROM patrols WHERE robot=? AND ended_at IS NULL "
                    "AND status!='interrupted' ORDER BY rowid DESC LIMIT 1", (self._robot,)
                ).fetchone()
                patrol_id = active["id"] if active and code != "PATROL_MISSION_REJECTED" else None
                if active:
                    elapsed = max(0, tick - active["last_tick"])
                    phase = active["phase"]
                    circuit = elapsed if phase == "PATROL" else 0
                    status, ended_at = active["status"], None
                    if code in START:
                        status, ended_at = "replaced", timestamp
                    elif mission_id is None or mission_id == active["mission_id"]:
                        if code in FINISH:
                            status, ended_at = FINISH[code], timestamp
                        elif code in {"PATROL_PHASE_FAILED", "PATROL_ROUTE_PAUSED", "ROUTE_LOOP_PAUSED"}:
                            status, phase = "paused", "PAUSED"
                        elif code == "BATTERY_RETURN_LATCHED":
                            phase = details.get("phase", phase)
                            if phase == "AT_HOME":
                                status, ended_at = "held", timestamp
                        elif code in {"RETURN_HOME_REQUESTED", "BATTERY_RETURN_HOME_REQUESTED"}:
                            phase = "EXIT_LOOP"
                        elif code in {"BATTERY_RETURN_CANCEL_UNAVAILABLE", "BATTERY_RETURN_CANCEL_FAILED"}:
                            status, phase, ended_at = "failed", "ABORTED", timestamp
                        elif code in {"PATROL_PHASE_DISPATCHED", "PATROL_LOOP_JOINED"}:
                            phase = details.get("phase", phase)
                            status = "active"
                    else:
                        patrol_id = None
                    db.execute(
                        "UPDATE patrols SET duration_s=duration_s+?, circuit_s=circuit_s+?, "
                        "last_tick=?, phase=?, status=?, ended_at=? WHERE id=?",
                        (elapsed, circuit, tick, phase, status, ended_at, active["id"]),
                    )
                if code in START:
                    patrol_id = uuid.uuid4().hex
                    phase = details.get("phase")
                    held = phase == "AT_HOME"
                    db.execute(
                        "INSERT INTO patrols(id,robot,mission_id,started_at,ended_at,status,phase,last_tick) "
                        "VALUES(?,?,?,?,?,?,?,?)",
                        (patrol_id, self._robot, mission_id or "", timestamp, timestamp if held else None,
                         "held" if held else "active", phase, tick),
                    )
                elif code in {CHECKPOINT, "PATROL_HELD_AT_HOME_LOW_BATTERY", "NAVIGATION_INCIDENT"} and mission_id and patrol_id is None:
                    row = db.execute(
                        "SELECT id FROM patrols WHERE robot=? AND mission_id=? ORDER BY rowid DESC LIMIT 1",
                        (self._robot, mission_id),
                    ).fetchone()
                    patrol_id = row["id"] if row else None
                db.execute(
                    "INSERT INTO history_events(robot,timestamp,patrol_id,category,severity,component,code,details) "
                    "VALUES(?,?,?,?,?,?,?,?)",
                    (self._robot, timestamp, patrol_id, category, severity, component, code,
                     json.dumps(details, ensure_ascii=False)),
                )
            self._last_events[component] = (fingerprint, tick)
        except (OSError, sqlite3.Error):
            self._fail()
        finally:
            self._slots.release()

    def _event(self, row):
        event = dict(row)
        event["details"] = json.loads(event["details"])
        return event

    def _summary(self, row):
        summary = dict(row)
        tick = summary.pop("last_tick")
        if summary["status"] in {"active", "paused"} and not self._fault:
            elapsed = max(0, monotonic() - tick)
            summary["duration_s"] += elapsed
            if summary["phase"] == "PATROL":
                summary["circuit_s"] += elapsed
        summary["incomplete"] = bool(self._fault) or summary["status"] == "interrupted"
        return summary

    def query(self, op, fields):
        try:
            with closing(self._connect()) as db, db:
                db.execute("BEGIN")
                if op == "history_list":
                    return self._list(db, fields)
                return self._detail(db, fields)
        except sqlite3.Error as exc:
            raise OSError("Historial no disponible en el robot") from exc

    def _list(self, db, fields):
        view = fields.get("view", "activity")
        if view not in ("activity", "patrols"):
            raise ValueError("Vista de historial inválida")
        patrols = view == "patrols"
        table, date, order = ("patrols", "started_at", "rowid") if patrols else (
            "history_events", "timestamp", "sequence")
        conditions, params = ["robot=?"], [self._robot]
        for field, operator in (("start", ">="), ("end", "<")):
            if field in fields:
                raw = fields[field]
                if not isinstance(raw, str) or len(raw) > 40:
                    raise ValueError("Fecha inválida")
                parsed = datetime.fromisoformat(raw.replace("Z", "+00:00"))
                if parsed.tzinfo is None:
                    raise ValueError("La fecha debe incluir zona horaria")
                conditions.append(f"{date}{operator}?")
                params.append(parsed.astimezone(timezone.utc).isoformat(timespec="milliseconds"))
        if not patrols:
            if fields.get("category"):
                conditions.append("category=?")
                params.append(str(fields["category"])[:30])
            else:
                conditions.append("category!='waypoint'")
        if fields.get("before") is not None:
            conditions.append(f"{order}<?")
            params.append(self._cursor(fields["before"]))
        where = " AND ".join(conditions)
        rows = db.execute(
            f"SELECT {order} AS cursor, * FROM {table} WHERE {where} ORDER BY {order} DESC LIMIT 51",
            params,
        ).fetchall()
        items = []
        for row in rows[:50]:
            item = self._summary(row) if patrols else self._event(row)
            item.pop("cursor")
            items.append(item)
        return {"items": items, "next": rows[49]["cursor"] if len(rows) > 50 else None,
                "degraded": bool(self._fault), "robot": self._robot}

    @staticmethod
    def _cursor(value):
        if type(value) is not int or not 0 <= value < 2**63:
            raise ValueError("Cursor inválido")
        return value

    def _detail(self, db, fields):
        patrol_id = fields.get("patrol_id")
        if not isinstance(patrol_id, str) or len(patrol_id) != 32:
            raise ValueError("Identificador de patrulla inválido")
        row = db.execute("SELECT * FROM patrols WHERE robot=? AND id=?", (self._robot, patrol_id)).fetchone()
        if row is None:
            raise ValueError("Patrulla no encontrada")
        stream = fields.get("stream", "all")
        if stream not in ("all", "operations", "visits"):
            raise ValueError("Tipo de historial inválido")
        after = self._cursor(fields.get("after", 0))
        if "through" in fields:
            through = self._cursor(fields["through"])
        else:
            through = db.execute(
                "SELECT COALESCE(MAX(sequence),0) FROM history_events WHERE robot=? AND patrol_id=?",
                (self._robot, patrol_id),
            ).fetchone()[0]
        filters, extra = "", ()
        if stream == "operations":
            filters = "AND category!='waypoint'"
        elif stream == "visits":
            route, index = fields.get("route_mission_id"), fields.get("input_index")
            if not isinstance(route, str) or not isinstance(index, str):
                raise ValueError("Punto inválido")
            filters = ("AND category='waypoint' AND json_extract(details,'$.route_mission_id')=? "
                       "AND json_extract(details,'$.input_index')=?")
            extra = (route, index)
        rows = db.execute(
            "SELECT * FROM history_events WHERE robot=? AND patrol_id=? AND sequence>? AND sequence<=? "
            f"{filters} ORDER BY sequence LIMIT 101", (self._robot, patrol_id, after, through, *extra),
        ).fetchall()
        result = {"schema_version": 1, "patrol": self._summary(row),
                  "items": [self._event(event) for event in rows[:100]], "through": through,
                  "next": rows[99]["sequence"] if len(rows) > 100 else None}
        if stream == "operations" and after == 0:
            result["waypoints"] = self._waypoint_summary(db, patrol_id, through)
        return result

    def _waypoint_summary(self, db, patrol_id, through):
        rows = db.execute("""
            WITH arrivals AS (
                SELECT sequence, timestamp, details,
                    json_extract(details, '$.route_mission_id') AS route_id,
                    json_extract(details, '$.input_index') AS point_index
                FROM history_events
                WHERE robot=? AND patrol_id=? AND sequence<=? AND category='waypoint'
            ), ranked AS (
                SELECT *,
                    COUNT(*) OVER point AS visit_count,
                    MAX(timestamp) OVER point AS last_at,
                    ROW_NUMBER() OVER (
                        PARTITION BY route_id, point_index
                        ORDER BY (json_extract(details, '$.gps_lat') IS NOT NULL) DESC, sequence DESC
                    ) AS position_rank
                FROM arrivals
                WINDOW point AS (PARTITION BY route_id, point_index)
            )
            SELECT sequence, timestamp, details, visit_count, last_at
            FROM ranked WHERE position_rank=1 ORDER BY route_id, CAST(point_index AS INTEGER)
            """, (self._robot, patrol_id, through))
        return [self._event(row) for row in rows]
