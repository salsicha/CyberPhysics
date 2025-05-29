#!/bin/sh
# npx drizzle-kit push
bun run dev
ollama serve &
exec "$@" 