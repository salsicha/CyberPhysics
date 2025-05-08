
FROM node:20-alpine AS build

RUN apt update && apt install -q -y --no-install-recommends \
    curl git vim gnupg2 lsb-release python3-pip python3-venv && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /

## Clone your forked repository
RUN git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/salsicha/sim.git

WORKDIR /sim

## Create environment file and update with required environment variables (BETTER_AUTH_SECRET)
COPY .env /sim/.env

## Set working directory
WORKDIR /sim/app

## Copy the entire sim directory
COPY sim/ ./

## Create the .env file if it doesn't exist
RUN touch .env

## Install dependencies
RUN npm install

## Generate database schema
RUN npx drizzle-kit generate

EXPOSE 3000

## Run the ollama_docker.sh script to pull the required models
RUN ./sim/scripts/ollama_docker.sh pull <model_name>


COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


#####################################################################

FROM scratch

COPY --from=build / /

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

CMD ["npx", "drizzle-kit", "push", "&&", "npm", "run", "dev"]
