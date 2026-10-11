"""The pinned upstream SO-101 design (not a part).

The SO-101 arm is TheRobotStudio's open hardware (SO-ARM100 repository,
Apache-2.0, designed with Hugging Face). Its printed parts are vendored, not
redrawn: the STLs in ``upstream/`` are byte-for-byte the files of the commit
below (SHA-256 pinned in each part module and checked at every build), and
``upstream/LICENSE`` is that commit's licence. tools/build_printable_parts.py
only turns each mesh into a print orientation for a Bambu Lab P1S.
"""
from hardware.parts.common import printing

REPOSITORY = "https://github.com/TheRobotStudio/SO-ARM100"
COMMIT = "a758567c3978dfeefe282ede0500085a48fe8f78"   # main, 2026-10-06
LICENCE = "Apache-2.0"
LICENCE_FILE = "upstream/LICENSE"
AUTHOR = ("TheRobotStudio with Hugging Face (SO-ARM100 CITATION.cff: R. Knight, P. Kooijmans, "
          "R. Cadene, S. Alibert et al.)")
README = f"{REPOSITORY}/blob/{COMMIT}/README.md"
ASSEMBLY_GUIDE = "https://huggingface.co/docs/lerobot/so101"

# The servo every arm part is shaped around, as the upstream design fits it.
STS3215 = {
    "part": "Feetech STS3215 servo (and its horns)",
    "dimensions": "servo pockets, horn screw circles and M2/M3 screw holes of the upstream design",
    "source": f"upstream design, {README}; assembly guide {ASSEMBLY_GUIDE}",
    "verified": True,
}

M2_SERVO = "M2 x 6 screw (STS3215 accessory bag)"
M3_SERVO = "M3 x 6 screw (STS3215 accessory bag)"


def vendored(filename, upstream_path, sha256):
    """The "vendored" entry of a part whose STL is ``upstream/<filename>``,
    published at ``upstream_path`` in the pinned commit."""
    return {
        "file": f"upstream/{filename}",
        "sha256": sha256,
        "source": f"{REPOSITORY}/blob/{COMMIT}/{upstream_path}",
        "licence": LICENCE,
        "licence_file": LICENCE_FILE,
        "author": AUTHOR,
    }


def settings(orientation, rotate=(), supports="tree(auto), everywhere"):
    """Upstream's tested settings (0.2 mm layers, 15 % infill, supports on
    overhangs steeper than 45 deg) in PETG with three walls."""
    return printing.settings("PETG", orientation, wall_loops=3, infill_pct=15, supports=supports,
                             rotate=rotate)


MATERIAL_REASON = ("indoors, but next to servos that run warm under load: PETG keeps its shape well above "
                   "the 55-60 C where PLA (upstream's PLA+) starts to soften")
UPSTREAM_NOTE = ("Upstream design, vendored unchanged and only turned to this print orientation; upstream prints "
                 "it in PLA+ at 0.2 mm, 15 % infill, supports on overhangs over 45 deg, none in horizontal screw holes.")
