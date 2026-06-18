FROM python:3.12-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    MARIMO_HOST=0.0.0.0 \
    MARIMO_PORT=2718 \
    MARIMO_WORKDIR=/notebooks

WORKDIR /workspace

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash \
    ca-certificates \
    curl \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --upgrade pip && \
    pip install "marimo[recommended]"

RUN mkdir -p /notebooks

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 2718

ENTRYPOINT ["/entrypoint.sh"]
CMD ["marimo", "edit", "--headless", "--host", "0.0.0.0", "--port", "2718", "--no-token", "/notebooks"]
