"""Group A identity mapping loader: task-layer (A01..A15) to execution-layer (UAV1..UAV15)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

import yaml


@dataclass(frozen=True)
class UavIdentity:
    uav_id: str
    exec_target: str
    hostname: str = ""
    ip: str = ""
    enabled: bool = True


class IdentityMap:
    """Loads and resolves the versioned uav_identity_map.yaml file."""

    def __init__(self, identities: List[UavIdentity]) -> None:
        self.uavs: List[UavIdentity] = list(identities)
        self._by_uav_id: Dict[str, UavIdentity] = {
            item.uav_id: item for item in identities
        }

    @classmethod
    def load(cls, path: str) -> "IdentityMap":
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        if not isinstance(data, dict):
            raise ValueError("identity map must be a YAML mapping")
        identities: List[UavIdentity] = []
        for item in data.get("uavs", []):
            if not isinstance(item, dict) or not item.get("uav_id"):
                raise ValueError("identity map entry requires uav_id")
            identities.append(
                UavIdentity(
                    uav_id=str(item["uav_id"]),
                    exec_target=str(item.get("exec_target", "")),
                    hostname=str(item.get("hostname", "")),
                    ip=str(item.get("ip", "")),
                    enabled=bool(item.get("enabled", True)),
                )
            )
        return cls(identities)

    def identity(self, uav_id: str) -> UavIdentity:
        try:
            return self._by_uav_id[uav_id]
        except KeyError:
            raise KeyError(f"uav_id {uav_id} not found in identity map") from None

    def resolve(self, uav_ids: List[str]) -> List[Tuple[str, UavIdentity]]:
        resolved: List[Tuple[str, UavIdentity]] = []
        for uav_id in uav_ids:
            resolved.append((uav_id, self.identity(uav_id)))
        return resolved