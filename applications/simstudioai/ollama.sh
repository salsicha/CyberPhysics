#!/bin/bash

echo "Starting Ollama server..."
ollama serve &

echo "Waiting for Ollama server to be active..."
while [ "$(ollama list | grep 'NAME')" = "" ]; do
  sleep 1
done

echo "Pulling model..."
ollama pull qwen3:8b

echo "Ollama model downloaded."
