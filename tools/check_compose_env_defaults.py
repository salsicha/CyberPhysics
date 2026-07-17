#!/usr/bin/env python3
"""Check compose ``${VAR:-default}`` fallbacks against referenced env files.

Scenario values live in env files (systems/*/config/*.env), but compose
files repeat them as interpolation fallbacks so a bare ``docker compose
up`` still works. When the two drift, the stack half-works in confusing
ways — the airplane sim once shipped with fallbacks that left MAVROS
unable to connect. This tool freezes that class of bug:

for every composition, each ``${VAR:-literal}`` fallback must equal the
value of VAR in every env file the composition references (service-level
``env_file:`` entries and include-level ``env_file:``), whenever those env
files define VAR. Values compare as strings with a numeric-equality
fallback ("180" == "180.0"). Fallbacks whose default is itself a ``${...}``
expression are skipped (not statically resolvable). Env files referenced
by the same composition must also agree with each other.

Exits nonzero listing every mismatch.
"""
import glob
import os
import re
import sys

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COMPOSITIONS = os.path.join(REPO, 'compositions')


def extract_fallbacks(text):
    """Yield (var, default) for each ${VAR:-default} / ${VAR-default},
    including expressions nested inside other defaults."""
    results = []
    i = 0
    while True:
        start = text.find('${', i)
        if start < 0:
            break
        depth, j = 1, start + 2
        while j < len(text) and depth:
            if text.startswith('${', j):
                depth += 1
                j += 2
            elif text[j] == '}':
                depth -= 1
                j += 1
            else:
                j += 1
        expr = text[start + 2:j - 1]
        match = re.match(r'([A-Za-z_][A-Za-z0-9_]*):?-(.*)$', expr, re.S)
        if match:
            var, default = match.group(1), match.group(2)
            if '${' in default:
                results.extend(extract_fallbacks(default))
            else:
                results.append((var, default))
        i = start + 2
    return results


def env_file_refs(compose_path):
    """Env file paths referenced by a composition (service and include)."""
    with open(compose_path) as fh:
        model = yaml.safe_load(fh)
    if not isinstance(model, dict):
        return []
    refs = []

    def add(value):
        entries = value if isinstance(value, list) else [value]
        for entry in entries:
            path = entry.get('path') if isinstance(entry, dict) else entry
            if isinstance(path, str):
                refs.append(os.path.normpath(
                    os.path.join(os.path.dirname(compose_path), path)))

    for service in (model.get('services') or {}).values():
        if isinstance(service, dict) and 'env_file' in service:
            add(service['env_file'])
    for entry in model.get('include') or []:
        if isinstance(entry, dict) and 'env_file' in entry:
            add(entry['env_file'])
    return sorted(set(refs))


def parse_env_file(path):
    values = {}
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith('#') or '=' not in line:
                continue
            key, _, value = line.partition('=')
            values[key.strip()] = value.strip()
    return values


def equal(a, b):
    if a == b:
        return True
    try:
        return float(a) == float(b)
    except ValueError:
        return False


def main():
    failures = []
    for compose_path in sorted(glob.glob(os.path.join(COMPOSITIONS, '*.yaml'))):
        rel = os.path.relpath(compose_path, REPO)
        env_paths = env_file_refs(compose_path)
        if not env_paths:
            continue
        env_values = {}
        for env_path in env_paths:
            if not os.path.exists(env_path):
                failures.append(f'{rel}: missing env file {env_path}')
                continue
            for key, value in parse_env_file(env_path).items():
                if key in env_values and not equal(env_values[key][0], value):
                    failures.append(
                        f'{rel}: env files disagree on {key}: '
                        f'{env_values[key][0]!r} ({env_values[key][1]}) vs '
                        f'{value!r} ({os.path.relpath(env_path, REPO)})')
                env_values.setdefault(
                    key, (value, os.path.relpath(env_path, REPO)))

        with open(compose_path) as fh:
            text = fh.read()
        for var, default in extract_fallbacks(text):
            if var in env_values and not equal(env_values[var][0], default):
                failures.append(
                    f'{rel}: ${{{var}:-{default}}} but '
                    f'{env_values[var][1]} sets {var}={env_values[var][0]}')

    if failures:
        print('Compose fallbacks drifted from their env files:')
        for failure in sorted(set(failures)):
            print(f'  {failure}')
        return 1
    print('All compose ${VAR:-default} fallbacks match their env files')
    return 0


if __name__ == '__main__':
    sys.exit(main())
