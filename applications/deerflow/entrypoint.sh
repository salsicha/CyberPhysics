#!/bin/bash
set -e
source /venv/bin/activate
# Run both the backend and frontend servers in development mode
/app/deer-flow/bootstrap.sh -d
exec "$@"
