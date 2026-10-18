"""A coarse class set shared by every 2D backend, so maps labelled in ADE20K (150), COCO panoptic
(133) or by free-text prompts can be compared voxel by voxel. The coarse classes are the ones the
robot's map is for (traversability + a few landmark objects); everything else is `other`."""

from __future__ import annotations

import numpy as np

UNLABELED = 255

COARSE = [
    "floor",  # 0
    "wall",  # 1
    "ceiling",  # 2
    "door",  # 3
    "person",  # 4
    "stairs",  # 5
    "seat",  # 6  chair / sofa / bench / armchair / stool
    "table",  # 7  table / desk / counter / bar
    "column",  # 8
    "window",  # 9  windowpane / glass / window
    "storage",  # 10 shelf / cabinet / bookcase / wardrobe
    "plant",  # 11
    "sign",  # 12 signboard / poster / screen / billboard
    "light",  # 13 light / lamp / chandelier
    "escalator",  # 14 (its own class: ADE has it, and the site has one)
    "other",  # 15
]
COARSE_ID = {n: i for i, n in enumerate(COARSE)}
OTHER = COARSE_ID["other"]

COARSE_PALETTE = np.array(
    [
        (80, 200, 80),  # floor
        (160, 160, 160),  # wall
        (120, 170, 230),  # ceiling
        (160, 90, 30),  # door
        (255, 0, 0),  # person
        (255, 140, 0),  # stairs
        (240, 200, 60),  # seat
        (200, 120, 200),  # table
        (200, 200, 220),  # column
        (100, 220, 255),  # window
        (140, 100, 70),  # storage
        (60, 180, 120),  # plant
        (255, 255, 0),  # sign
        (255, 255, 200),  # light
        (255, 80, 200),  # escalator
        (110, 60, 140),  # other
    ]
    + [(30, 30, 30)] * (256 - 16),
    np.uint8,
)

# keyword -> coarse class; the first comma-separated name of a fine class is matched against these,
# as a whole or word by word (COCO "wall-other-merged" -> wall).
_RULES: list[tuple[tuple[str, ...], str]] = [
    (("floor", "rug", "carpet", "mat", "pavement", "sidewalk", "road", "path", "runway"), "floor"),
    (("wall", "fence", "screen door"), "wall"),
    (("ceiling",), "ceiling"),
    (("door", "doorframe"), "door"),
    (("person", "people"), "person"),
    (("stairs", "stairway", "step", "staircase"), "stairs"),
    (
        ("chair", "armchair", "sofa", "couch", "bench", "stool", "seat", "swivel chair", "ottoman"),
        "seat",
    ),
    (
        (
            "table",
            "desk",
            "counter",
            "bar",
            "countertop",
            "coffee table",
            "dining table",
            "kitchen island",
        ),
        "table",
    ),
    (("column", "pillar", "pole"), "column"),
    (("windowpane", "window"), "window"),  # not "glass": ADE's is a drinking glass
    (
        ("shelf", "cabinet", "bookcase", "wardrobe", "chest of drawers", "case", "showcase"),
        "storage",
    ),
    (("plant", "tree", "flower", "potted plant", "palm"), "plant"),
    (
        (
            "signboard",
            "poster",
            "screen",
            "billboard",
            "television receiver",
            "tv",
            "monitor",
            "bulletin board",
            "crt screen",
        ),
        "sign",
    ),
    (("light", "lamp", "chandelier", "sconce"), "light"),
    (("escalator",), "escalator"),
]


def coarse_of_name(name: str) -> int:
    base = name.split(",")[0].strip().lower()
    tokens = set(base.replace("-", " ").replace("_", " ").split())
    # whole names or whole words only: a substring match folded "streetlight" into plant (tree)
    # and "suitcase" into storage (case)
    for keys, cls in _RULES:
        if base in keys or tokens & set(keys):
            return COARSE_ID[cls]
    return OTHER


def coarse_lut(id2label: dict[int, str]) -> np.ndarray:
    """256-entry LUT fine id -> coarse id (255 stays 255)."""
    lut = np.full(256, UNLABELED, np.uint8)
    for i, name in id2label.items():
        lut[int(i)] = coarse_of_name(name)
    return lut


if __name__ == "__main__":
    import json
    import sys

    # taxonomy.py classes.json: print how a backend's classes fold into the coarse set
    classes = json.load(open(sys.argv[1]))
    lut = coarse_lut({int(k): v for k, v in classes["id2label"].items()})
    groups: dict[str, list[str]] = {}
    for k, v in classes["id2label"].items():
        groups.setdefault(COARSE[lut[int(k)]], []).append(v.split(",")[0])
    for c in COARSE:
        print(f"{c:10s} <- {', '.join(groups.get(c, []))}")
