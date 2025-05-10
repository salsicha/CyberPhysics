
FROM ubuntu:noble AS build

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    curl git vim gnupg2 gnupg lsb-release python3-pip python3-venv && \
    curl -sL https://deb.nodesource.com/setup_22.x | bash - && \
    apt-get update && \
    apt-get install -y nodejs && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /

## Clone your forked repository
RUN git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/simstudioai/sim.git

RUN mkdir /app
RUN cp -r /sim/sim /app/sim

## Set working directory
WORKDIR /app/sim

## Create the .env file if it doesn't exist
RUN touch /app/.env

## Install dependencies
RUN npm install

## Generate database schema
RUN npx drizzle-kit generate

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

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

CMD ["npx", "drizzle-kit", "push", "&&", "npm", "run", "dev"]
