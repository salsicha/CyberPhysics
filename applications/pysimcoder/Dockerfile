
FROM robertobucher/pysimcoder:latest

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PYSUPSICTRL=/pysimCoder

SHELL ["/bin/bash","-c"]

RUN apt update && apt install -q -y --no-install-recommends \
    curl gnupg2 lsb-release vim wget && \
    rm -rf /var/lib/apt/lists/*

# pysimCoder and its Python packages live in the base image's system Python;
# "psc" is only a shell alias there, so provide it as a real command.
RUN printf '#!/bin/sh\nexec /pysimCoder/pysim-run.sh "$@"\n' > /usr/local/bin/psc && \
    chmod +x /usr/local/bin/psc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

CMD ["psc"]
