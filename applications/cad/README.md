# CAD

Pinned [CadQuery](https://cadquery.readthedocs.io/) toolchain for the
3D-printable parts in `hardware/parts/` (see
[hardware/parts/README.md](../../hardware/parts/README.md)): CadQuery 2.8.0
on OCCT 7.9.3 (cadquery-ocp), trimesh and rtree for the mesh checks,
matplotlib for the preview and assembly images. Versions are pinned in
`requirements.txt`, which the `printable-parts` job in
`.github/workflows/ci.yml` installs too, so CI and the image build the same
geometry.

The image holds no repository files: mount the checkout and run
`tools/build_printable_parts.py` in it.

## Build

```bash
make -C applications build_cad        # salsicha/cyberphysics:cad-latest, about 2.2 GB
```

## Use

From the repository root, as your own user so the outputs are yours:

```bash
run_cad() { docker run --rm -u "$(id -u):$(id -g)" -v "$PWD":/repo -w /repo \
  salsicha/cyberphysics:cad-latest "$@"; }

run_cad python3 tools/build_printable_parts.py             # all parts -> hardware/parts/<platform>/stl/
run_cad python3 tools/build_printable_parts.py aerodrone --check
run_cad python3 tools/build_printable_parts.py aerodrone --out /repo/generated/scratch
run_cad python3 tools/build_printable_parts.py aerodrone --figures
run_cad python3 -m pytest tests/test_printable_parts_build.py
```

STEP files go to `generated/parts/<platform>/` (git-ignored); if that
directory is not writable (for example created by a root container), the
script skips them with a warning; use `--out` instead.

Without Docker, a Python 3.12 virtual environment with
`pip install -r applications/cad/requirements.txt` works the same way.

## Notes

- Headless: OCCT's VTK needs `libgl1` and `libxrender1`, installed in the
  image; nothing opens a window. matplotlib renders with the Agg backend.
- Changing a pinned version can change the tessellation of every STL: run
  the full build, check `--check` passes, and commit the STLs with the
  version bump.
