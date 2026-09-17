"""Minimal WPILib data log (.wpilog) reader.

Enough of the format to pull AdvantageKit outputs back out of a log: control
records (Start/Finish/SetMetadata), the scalar types, and the WPILib struct
encodings for Pose3d / Transform3d.

Format reference: WPILib "WPILib Data Log File Format, Version 1.0".
No third-party dependencies.
"""

import mmap
import struct
from typing import Iterator, NamedTuple, Optional

_HEADER_MAGIC = b"WPILOG"

# WPILib struct encodings. Pose3d is Translation3d{x,y,z} + Rotation3d{Quaternion{w,x,y,z}}.
_POSE3D = struct.Struct("<7d")
_TRANSLATION3D = struct.Struct("<3d")


class StartRecord(NamedTuple):
    entry: int
    name: str
    type: str
    metadata: str


class Record(NamedTuple):
    entry: int
    timestamp: int  # microseconds
    data: bytes


class DataLogReader:
    """Iterates the raw records of a .wpilog file."""

    def __init__(self, path: str):
        with open(path, "rb") as f:
            self._buf = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        if self._buf[:6] != _HEADER_MAGIC:
            raise ValueError(f"{path} is not a WPILOG file (bad magic)")
        version = int.from_bytes(self._buf[6:8], "little")
        if version >> 8 != 1:
            raise ValueError(f"unsupported WPILOG major version {version >> 8}")
        extra_len = int.from_bytes(self._buf[8:12], "little")
        self._start = 12 + extra_len

    def __iter__(self) -> Iterator[Record]:
        buf = self._buf
        pos = self._start
        size = len(buf)
        while pos + 1 < size:
            header = buf[pos]
            entry_len = (header & 0x3) + 1
            size_len = ((header >> 2) & 0x3) + 1
            ts_len = ((header >> 4) & 0x7) + 1
            head_len = 1 + entry_len + size_len + ts_len
            if pos + head_len > size:
                break

            p = pos + 1
            entry = int.from_bytes(buf[p : p + entry_len], "little")
            p += entry_len
            payload_size = int.from_bytes(buf[p : p + size_len], "little")
            p += size_len
            timestamp = int.from_bytes(buf[p : p + ts_len], "little")
            p += ts_len

            if p + payload_size > size:
                break  # truncated log (robot powered off mid-write); stop cleanly
            yield Record(entry, timestamp, buf[p : p + payload_size])
            pos = p + payload_size


def _read_str(data: bytes, pos: int):
    length = int.from_bytes(data[pos : pos + 4], "little")
    pos += 4
    return data[pos : pos + length].decode("utf-8", "replace"), pos + length


def parse_start(data: bytes) -> Optional[StartRecord]:
    """Parse a control record, returning the Start payload or None for other kinds."""
    if not data or data[0] != 0:  # 0 = Start, 1 = Finish, 2 = SetMetadata
        return None
    entry = int.from_bytes(data[1:5], "little")
    name, pos = _read_str(data, 5)
    typ, pos = _read_str(data, pos)
    metadata, _ = _read_str(data, pos)
    return StartRecord(entry, name, typ, metadata)


def decode(typ: str, data: bytes):
    """Decode a payload according to its declared entry type.

    Returns None for types this reader does not handle, so callers can skip them.
    """
    if typ == "double":
        return struct.unpack("<d", data)[0]
    if typ == "int64":
        return int.from_bytes(data, "little", signed=True)
    if typ == "boolean":
        return bool(data[0])
    if typ in ("string", "json"):
        return data.decode("utf-8", "replace")
    if typ == "int64[]":
        return list(struct.unpack(f"<{len(data) // 8}q", data))
    if typ == "double[]":
        return list(struct.unpack(f"<{len(data) // 8}d", data))
    if typ == "boolean[]":
        return [bool(b) for b in data]
    if typ in ("struct:Pose3d", "struct:Transform3d"):
        return _POSE3D.unpack(data)  # (x, y, z, qw, qx, qy, qz)
    if typ == "struct:Translation3d":
        return _TRANSLATION3D.unpack(data)
    return None


def read_entries(path: str, name_filter=None):
    """Read a log into {entry name: (type, [(timestamp_seconds, value), ...])}.

    ``name_filter`` is an optional predicate on the entry name; entries it rejects
    are never decoded, which keeps memory sane on full match logs.
    """
    reader = DataLogReader(path)
    types: dict[int, str] = {}
    names: dict[int, str] = {}
    out: dict[str, tuple] = {}

    for record in reader:
        if record.entry == 0:
            start = parse_start(record.data)
            if start is None:
                continue
            if name_filter is not None and not name_filter(start.name):
                continue
            types[start.entry] = start.type
            names[start.entry] = start.name
            out.setdefault(start.name, (start.type, []))
            continue

        typ = types.get(record.entry)
        if typ is None:
            continue
        value = decode(typ, record.data)
        if value is None:
            continue
        out[names[record.entry]][1].append((record.timestamp / 1e6, value))

    return out
