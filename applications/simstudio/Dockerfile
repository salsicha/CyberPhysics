
FROM ubuntu:noble AS build

# Pinned Sim release and the Bun version its package.json asks for
ARG SIM_REF=v0.9.16
ARG BUN_VERSION=1.4.2

# python3/make/g++ build the isolated-vm native module during bun install
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    ca-certificates curl git vim gnupg2 gnupg lsb-release python3 make g++ unzip && \
    curl -sL https://deb.nodesource.com/setup_24.x | bash - && \
    apt-get update && \
    apt-get install -y nodejs && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir /app

WORKDIR /app

## Clone the pinned release
RUN git clone --depth 1 --branch ${SIM_REF} https://github.com/simstudioai/sim.git

## Set working directory
WORKDIR /app/sim

## Install bun
RUN npm install -g bun@${BUN_VERSION}

## Install
RUN HUSKY=0 bun install --frozen-lockfile

WORKDIR /app/sim/apps/sim
RUN cp .env.example .env

EXPOSE 3000

# Database migrations need the Postgres service, so they run in the
# entrypoint at container start rather than here. Local LLMs are served by
# the ollama services in compositions/simstudio.yaml.
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


#####################################################################

FROM scratch

ENV NEXT_TELEMETRY_DISABLED=1

COPY --from=build / /

WORKDIR /app/sim

ENTRYPOINT ["/entrypoint.sh"]

# Next.js app (port 3000) and the realtime socket server (port 3002)
CMD ["bun", "run", "dev:full"]
