from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Iterable, List, Tuple
from Arm_Control.SCARA import *

def grand_tour(obj: List[Tuple[float, float, float]]):
    coordinate_mode()
    for pos in obj:
        x, y, _ = pos
        time.sleep(3)
        try:
            quick_suction(x, y, 200, f=10000, maintain_extension_direction=True, extension_angle=-90.0)
        except ValueError:
            pass

def read_detected_object_positions(
	json_path: str | Path = "detection_output/detected_objects.json",
	source: str = "arm",
) -> List[Tuple[float, float, float]]:
	"""Read detected objects and return a list of (x, y, z) tuples.

	Parameters
	----------
	json_path: str | Path
		Path to the JSON file produced by the detector.
	source: str
		Which coordinate source to use per object:
		- "arm": use object["arm_coordinates"] (default)
		- "camera": use object["camera_position"]

	Returns
	-------
	List[Tuple[float, float, float]]
		A list of (x, y, z) positions for each object where the requested
		coordinate source exists.

	Raises
	------
	FileNotFoundError
		If the JSON file does not exist.
	ValueError
		If the JSON structure is missing the expected top-level keys.
	"""

	path = Path(json_path)
	if not path.exists():
		raise FileNotFoundError(f"Detection results not found at: {path}")

	with path.open("r", encoding="utf-8") as f:
		data = json.load(f)

	if "detected_objects" not in data or not isinstance(data["detected_objects"], list):
		raise ValueError("Invalid detection JSON: missing 'detected_objects' list")

	coordinate_key = {
		"arm": "arm_coordinates",
		"camera": "camera_position",
	}.get(source)
	if coordinate_key is None:
		raise ValueError("source must be 'arm' or 'camera'")

	positions: List[Tuple[float, float, float]] = []
	for obj in data["detected_objects"]:
		coords = obj.get(coordinate_key)
		if not coords:
			continue
		try:
			x = float(coords["x"])  # type: ignore[index]
			y = float(coords["y"])  # type: ignore[index]
			z = float(coords["z"])  # type: ignore[index]
			positions.append((x, y, z))
		except (KeyError, TypeError, ValueError):
			# Skip malformed entries gracefully
			continue

	return positions


if __name__ == "__main__":
    grand_tour(read_detected_object_positions())