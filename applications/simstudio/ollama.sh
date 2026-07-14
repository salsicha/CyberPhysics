#!/bin/bash
set -e

echo "Starting Ollama server..."
ollama serve &

echo "Waiting for Ollama server to be active..."
timeout=60
while [ "$(ollama list 2>/dev/null | grep 'NAME')" = "" ]; do
  timeout=$((timeout - 1))
  if [ "$timeout" -le 0 ]; then
    echo "Timed out waiting for Ollama server to become active" >&2
    exit 1
  fi
  sleep 1
done

echo "Pulling model..."
ollama pull qwen3:8b

echo "Ollama model downloaded."
