# CadQuery toolchain for the 3D-printable parts in hardware/parts: builds,
# validates and exports them with tools/build_printable_parts.py. Run it on
# the repository checkout (see README.md); nothing of the repository is
# baked into the image.
FROM python:3.12-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    MPLBACKEND=Agg \
    MPLCONFIGDIR=/tmp/matplotlib \
    NUMBA_CACHE_DIR=/tmp/numba

# OCP's bundled VTK needs libGL and X render libraries even headless
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 \
    libglib2.0-0 \
    libgomp1 \
    libxrender1 \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN pip install --upgrade pip && \
    pip install -r /tmp/requirements.txt && \
    python -c "import cadquery, trimesh, rtree; print('cadquery', cadquery.__version__)"

WORKDIR /repo
CMD ["python3", "tools/build_printable_parts.py", "--help"]
