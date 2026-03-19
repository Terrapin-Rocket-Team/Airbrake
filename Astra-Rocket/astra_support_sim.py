from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np


DEFAULT_AIRBRAKE_CSV = "HITL-Airbrake.csv"

ACTUAL_ANGLE_KEYS = (
    "AirbrakeCtrl - Actual Angle (deg)",
    "State - Actual Angle (deg)",
    "MotorDriver - Motor Angle",
)

COMMANDED_ANGLE_KEYS = (
    "AirbrakeCtrl - Actuation Angle (deg)",
    "State - Actuation Angle (deg)",
    "MotorDriver - Motor Target Angle",
)


class _TimeZeroSim:
    """Shift packet timestamps so the first emitted sample starts at t=0."""

    def __init__(self, source):
        self._source = source
        self._t0 = None

    def is_finished(self):
        return self._source.is_finished()

    def get_next_packet(self):
        packet = self._source.get_next_packet()
        if self._t0 is None:
            self._t0 = packet.timestamp
        packet.timestamp = packet.timestamp - self._t0
        return packet


class FlightCodePropagatorSim:
    """DataSource adapter for Airbrake Flight_Code propagator-based simulation."""

    def __init__(self, project_root: Path, astra_sim_module):
        self._project_root = project_root
        self._PacketData = astra_sim_module.PacketData
        self._prop = _load_local_propagator(project_root)
        self._launch_lat_deg = 45.0
        self._launch_lon_deg = -122.0
        self._flap_angle_deg = 0.0
        self._finished = False
        self._landed_time: float | None = None
        self._last_packet = None
        self._reset_propagator()

    def _reset_propagator(self) -> None:
        p = self._prop
        p.t = 0.0
        p.a = np.array([0.0, 0.0, -9.8], dtype=float)
        p.v = np.array([0.0, 0.0, 0.0], dtype=float)
        p.r = np.array([0.0, 0.0, 0.0], dtype=float)
        p.w = np.array([0.0, 0.0, 0.0], dtype=float)
        p.lat = 0.0
        p.long = 0.0
        p.m = float(p.wetMass)
        p.main_deployed = False
        p.settling_timer = 0.0
        p.atmosphere = p.Atmosphere(p.ground_altitude)
        self._finished = False
        self._landed_time = None
        self._last_packet = None

    def on_fc_telemetry(self, fields: dict[str, str]) -> None:
        # Use physical flap angle when available so motor dynamics (speed/lag)
        # actually influence propagation. Commanded angle is only a fallback.
        for key in ACTUAL_ANGLE_KEYS:
            raw_value = fields.get(key)
            if raw_value is None:
                continue
            try:
                value = float(raw_value)
            except (TypeError, ValueError):
                continue
            if math.isfinite(value):
                self._flap_angle_deg = min(105.0, max(0.0, value))
                return

        for key in COMMANDED_ANGLE_KEYS:
            raw_value = fields.get(key)
            if raw_value is None:
                continue
            try:
                value = float(raw_value)
            except (TypeError, ValueError):
                continue
            if math.isfinite(value):
                self._flap_angle_deg = min(105.0, max(0.0, value))
                return

    def is_finished(self) -> bool:
        return self._finished

    def _is_pad_idle_time(self) -> bool:
        launch_time = float(getattr(self._prop, "launchTime", 0.0))
        sim_time = float(getattr(self._prop, "t", 0.0))
        return sim_time < launch_time

    def _truth_inertial_accel_from_propagator(self, inertial_accel: np.ndarray) -> np.ndarray:
        # The legacy propagator uses -g as a prelaunch placeholder even while the
        # vehicle is clamped on the pad. Export a physically meaningful inertial
        # acceleration for diagnostics: zero on the pad, propagator output in flight.
        if self._is_pad_idle_time():
            return np.array([0.0, 0.0, 0.0], dtype=float)

        accel = np.asarray(inertial_accel, dtype=float).copy()
        if not np.all(np.isfinite(accel)):
            return np.array([0.0, 0.0, 0.0], dtype=float)
        return accel

    def _sensor_accel_from_inertial(self, inertial_accel: np.ndarray) -> np.ndarray:
        # FC expects accelerometer specific force in m/s^2.
        # At rest on pad this should be close to (0, 0, +9.81), i.e. (0, 0, +1g).
        gravity = 9.81

        if self._is_pad_idle_time():
            return np.array([0.0, 0.0, gravity], dtype=float)

        try:
            accel = np.asarray(inertial_accel, dtype=float).copy()
            # Propagator acceleration includes gravity on +Z/-Z depending phase.
            # Convert to specific force for IMU by adding back +g in Z.
            accel[2] += gravity
            if not np.all(np.isfinite(accel)):
                return np.array([0.0, 0.0, gravity], dtype=float)
            return accel
        except Exception:
            return np.array([0.0, 0.0, gravity], dtype=float)

    def get_next_packet(self):
        if self._finished and self._last_packet is not None:
            return self._last_packet

        self._prop.Propagate(self._flap_angle_deg)

        t = float(self._prop.t)
        pos = np.asarray(self._prop.r, dtype=float)
        vel = np.asarray(self._prop.v, dtype=float)
        inertial_accel = self._truth_inertial_accel_from_propagator(self._prop.a)
        sensor_accel = self._sensor_accel_from_inertial(inertial_accel)

        atmosphere = self._prop.atmosphere
        fallback_pressure_hpa = float(np.asarray(atmosphere.pressure, dtype=float).reshape(-1)[0] / 100.0)
        pressure_hpa = float(getattr(self._prop, "reported_pressure_hpa", fallback_pressure_hpa))
        temp_c = float(np.asarray(atmosphere.temperature, dtype=float).reshape(-1)[0] - 273.15)
        alt_agl = max(0.0, float(pos[2]))
        ground_alt = float(getattr(self._prop, "ground_altitude", 0.0))
        gps_alt_asl = ground_alt + alt_agl
        launch_time = float(getattr(self._prop, "launchTime", 0.0))

        # Generate GPS geodetic position from local ENU displacement.
        # This avoids legacy propagator lat/long values that are not earth-referenced.
        east_m = float(pos[0])
        north_m = float(pos[1])
        meters_per_deg_lat = 111_320.0
        lat = self._launch_lat_deg + (north_m / meters_per_deg_lat)
        cos_lat = math.cos(math.radians(self._launch_lat_deg))
        meters_per_deg_lon = meters_per_deg_lat * cos_lat if abs(cos_lat) > 1e-6 else meters_per_deg_lat
        lon = self._launch_lon_deg + (east_m / meters_per_deg_lon)

        packet = self._PacketData(
            timestamp=t,
            accel=sensor_accel,
            gyro=np.zeros(3, dtype=float),
            mag=np.zeros(3, dtype=float),
            pressure=pressure_hpa,
            temp=temp_c,
            lat=lat,
            lon=lon,
            alt=gps_alt_asl,
            fix=3 if t >= launch_time else 1,
            sats=10 if t >= launch_time else 6,
            heading=0.0,
            truth_alt=alt_agl,
            truth_accel=float(inertial_accel[2]),
            sensor_alt_agl=float(getattr(self._prop, "reported_height_agl", alt_agl)),
        )
        self._last_packet = packet

        if t > launch_time + 2.0 and alt_agl <= 0.5 and float(vel[2]) <= 0.0:
            if self._landed_time is None:
                self._landed_time = t
            elif t - self._landed_time > 2.0:
                self._finished = True
        else:
            self._landed_time = None

        return packet


def _airbrake_data_roots(project_root: Path) -> list[Path]:
    return [
        project_root / "flight-data" / "airbrake" / "raw",
        project_root / "HITL" / "data",
        project_root.parent / "HITL" / "data",
    ]


def _flight_code_copy_root(project_root: Path) -> Path:
    return project_root / "sim" / "flight_code_src"


def _load_local_propagator(project_root: Path):
    source_dir = _flight_code_copy_root(project_root)
    module_path = source_dir / "propagator.py"
    if not module_path.is_file():
        raise FileNotFoundError(
            f"Missing propagator copy: {module_path}. "
            "Copy Flight_Code/src files into Astra-Rocket/sim/flight_code_src."
        )

    module_name = f"airbrake_flight_code_propagator_{abs(hash(str(module_path.resolve())))}"
    sys.path.insert(0, str(source_dir))
    try:
        spec = importlib.util.spec_from_file_location(module_name, module_path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"Could not load module spec from {module_path}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        if sys.path and sys.path[0] == str(source_dir):
            sys.path.pop(0)


def _find_airbrake_csv(project_root: Path, requested: str | None) -> Path | None:
    query = (requested or DEFAULT_AIRBRAKE_CSV).strip().strip('"').strip("'")
    if not query:
        query = DEFAULT_AIRBRAKE_CSV

    direct = Path(query)
    direct_candidates = []
    if direct.is_absolute():
        direct_candidates.append(direct)
    else:
        direct_candidates.append(Path.cwd() / direct)
        direct_candidates.append(project_root / direct)

    if direct.suffix.lower() != ".csv":
        csv_guess = direct.with_suffix(".csv")
        if csv_guess.is_absolute():
            direct_candidates.append(csv_guess)
        else:
            direct_candidates.append(Path.cwd() / csv_guess)
            direct_candidates.append(project_root / csv_guess)

    for candidate in direct_candidates:
        if candidate.is_file():
            return candidate.resolve()

    roots = _airbrake_data_roots(project_root)
    search_names = [query]
    query_path = Path(query)
    if query_path.suffix.lower() != ".csv":
        search_names.append(f"{query}.csv")

    direct_matches: list[Path] = []
    for root in roots:
        if not root.is_dir():
            continue
        for name in search_names:
            path = root / name
            if path.is_file():
                direct_matches.append(path.resolve())

    if len(direct_matches) == 1:
        return direct_matches[0]
    if len(direct_matches) > 1:
        options = "\n  - ".join(str(path) for path in direct_matches[:8])
        raise ValueError(f"Ambiguous Airbrake source '{query}'. Matches:\n  - {options}")

    matches: list[Path] = []
    for root in roots:
        if not root.is_dir():
            continue
        for csv_path in root.rglob("*.csv"):
            stem_match = csv_path.stem.lower() == query_path.stem.lower()
            name_match = csv_path.name.lower() == query_path.name.lower()
            if stem_match or name_match:
                matches.append(csv_path.resolve())

    unique: list[Path] = []
    seen = set()
    for match in matches:
        key = str(match).lower()
        if key in seen:
            continue
        seen.add(key)
        unique.append(match)

    if len(unique) == 1:
        return unique[0]
    if len(unique) > 1:
        options = "\n  - ".join(str(path) for path in unique[:8])
        raise ValueError(f"Ambiguous Airbrake source '{query}'. Matches:\n  - {options}")
    return None


def _parse_airbrake_source(source: str):
    token = source.strip()
    lower = token.lower()

    if lower in {"airbrake", "airbrake:propagator", "airbrake:prop", "airbrake:flight_code"}:
        return "propagator", None

    if lower.startswith("airbrake:csv:"):
        value = token.split(":", 2)[2].strip()
        return "csv", value or DEFAULT_AIRBRAKE_CSV

    if lower.startswith("airbrake-csv:"):
        value = token.split(":", 1)[1].strip()
        return "csv", value or DEFAULT_AIRBRAKE_CSV

    if lower.startswith("airbrake:"):
        value = token.split(":", 1)[1].strip()
        value_lower = value.lower()
        if value_lower in {"propagator", "prop", "flight_code"}:
            return "propagator", None
        return "csv", value or DEFAULT_AIRBRAKE_CSV

    return None, None


def create_data_source(source: str, project_root: Path, astra_sim_module, args=None):
    kind, requested = _parse_airbrake_source(source)
    if kind is None:
        return None

    if kind == "propagator":
        return FlightCodePropagatorSim(project_root, astra_sim_module)

    csv_path = _find_airbrake_csv(project_root, requested)
    if csv_path is None:
        raise ValueError(f"Could not resolve Airbrake CSV source '{source}'")
    base = astra_sim_module.CSVSim(str(csv_path))
    return astra_sim_module.PadDelaySim(_TimeZeroSim(base))


def list_sim_sources(project_root: Path, args=None):
    names = {"airbrake", "airbrake:propagator"}
    for root in _airbrake_data_roots(project_root):
        if not root.is_dir():
            continue
        for csv_path in root.rglob("*.csv"):
            names.add(f"airbrake:csv:{csv_path.stem}")
    return sorted(names)
