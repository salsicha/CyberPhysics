#!/bin/sh
set -e

# Apply database migrations at start-up: the Postgres service is not
# reachable while the image is built. Set SKIP_DB_MIGRATIONS=1 to skip.
if [ -n "$DATABASE_URL" ] && [ "${SKIP_DB_MIGRATIONS:-0}" != "1" ]; then
    echo "Applying database migrations"
    (cd /app/sim/packages/db && bun ./scripts/migrate.ts)
fi

exec "$@"
