
FROM ubuntu:noble AS build

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    curl git vim gnupg2 gnupg lsb-release python3-pip python3-venv unzip && \
    curl -sL https://deb.nodesource.com/setup_22.x | bash - && \
    apt-get update && \
    apt-get install -y nodejs && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir /app

WORKDIR /app

## Clone your forked repository
RUN git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/simstudioai/sim.git

## Set working directory
WORKDIR /app/sim

## Install bun
RUN curl -fsSL https://bun.sh/install | bash

## Install
RUN bun install

RUN cd apps/sim && cp .env.example .env

RUN bunx drizzle-kit push

EXPOSE 3000

## Run the ollama_docker.sh script to pull the required models
RUN curl -fsSL https://ollama.com/install.sh | sh
COPY ./ollama.sh /ollama.sh
RUN chmod +x /ollama.sh
RUN /ollama.sh

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


#####################################################################

FROM scratch

COPY --from=build / /

WORKDIR /app/sim

ENTRYPOINT ["/entrypoint.sh"]

CMD ["npx", "drizzle-kit", "push", "&&", "npm", "run", "dev"]
