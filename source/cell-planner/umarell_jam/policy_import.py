"""Dependency-free reader for an immutable umarell-cache-export-v1 data bundle.

This exposes candidate data and its certificates. It does not authorise execution
in another scene; the consumer must match dependencies and run release checks.
"""
import hashlib
import json
from pathlib import Path


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


class ImmutableCache:
    def __init__(self, root):
        self.root = Path(root)
        files = json.loads((self.root / "FILES.json").read_text())
        self.files = files["files"]
        self.manifest = self.read("cache-manifest.json")
        if self.manifest["schema_version"] != "umarell-cache-export-v1":
            raise ValueError("unsupported export schema")

    def read(self, relative):
        if relative not in self.files:
            raise ValueError("file is outside the frozen inventory")
        content = (self.root / relative).read_bytes()
        if hashlib.sha256(content).hexdigest() != self.files[relative]["sha256"]:
            raise ValueError("immutable file checksum mismatch")
        return json.loads(content)

    def candidates(self, cell_id, program_id, weld_id, enabled_robot_ids):
        cell = next(c for c in self.manifest["cells"] if c["cell_id"] == cell_id)
        for row in cell["coverage"]:
            if (row["program"] != program_id or row["weld"] != weld_id
                    or row["robot"] not in enabled_robot_ids or row["status"] != "certified"):
                continue
            doc = self.read(f"components/primitive_index/{row['key']}.json")
            if (doc["schema_version"] != "umarell-components-v1" or doc["key"] != row["key"]
                    or digest(doc["dependencies"]) != doc["key"]
                    or digest(doc["payload"]) != doc["payload_sha256"]):
                raise ValueError("component dependency/payload mismatch")
            for variant in doc["payload"]["variants"]:
                yield {"robot_id": row["robot"], "weld_id": weld_id,
                       "geometry_hash": cell["geometry_hash"], "component_key": row["key"],
                       "dependencies": doc["dependencies"], **variant}
