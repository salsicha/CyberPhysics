#!/bin/sh
npx drizzle-kit push
ollama serve &
exec "$@" 