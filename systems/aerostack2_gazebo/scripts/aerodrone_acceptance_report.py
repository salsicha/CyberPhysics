#!/usr/bin/env python3
"""Evaluate Aerodrone acceptance metrics against tracked thresholds."""

import argparse
import json
from pathlib import Path


def _load_thresholds(path):
    data = json.loads(Path(path).read_text())
    contract = data.get('metric_contract', {})
    if not contract:
        raise SystemExit('acceptance threshold file is missing metric_contract')
    for section, metrics in contract.items():
        if not metrics:
            raise SystemExit(f'{section}: missing metrics')
        for name, rule in metrics.items():
            if rule.get('op') not in {'min', 'max', 'equals'}:
                raise SystemExit(
                    f'{section}.{name}: invalid op {rule.get("op")}')
            if 'value' not in rule:
                raise SystemExit(f'{section}.{name}: missing threshold value')
    return data


def _flatten_results(thresholds, metrics):
    rows = []
    for section, section_rules in thresholds['metric_contract'].items():
        section_metrics = metrics.get(section, {})
        for name, rule in section_rules.items():
            actual = section_metrics.get(name)
            passed = False
            if actual is not None:
                if rule['op'] == 'min':
                    passed = float(actual) >= float(rule['value'])
                elif rule['op'] == 'max':
                    passed = float(actual) <= float(rule['value'])
                else:
                    passed = actual == rule['value']
            rows.append({
                'section': section,
                'metric': name,
                'op': rule['op'],
                'threshold': rule['value'],
                'actual': actual,
                'passed': passed,
            })
    return rows


def _markdown(rows):
    lines = [
        '# Aerodrone Acceptance Report',
        '',
        '| Section | Metric | Rule | Actual | Pass |',
        '| --- | --- | --- | --- | --- |',
    ]
    for row in rows:
        rule = f'{row["op"]} {row["threshold"]}'
        actual = 'missing' if row['actual'] is None else row['actual']
        passed = 'yes' if row['passed'] else 'no'
        lines.append(
            f'| {row["section"]} | {row["metric"]} | {rule} | '
            f'{actual} | {passed} |')
    return '\n'.join(lines) + '\n'


def _write_outputs(rows, output_json, output_markdown):
    report = {'passed': all(row['passed'] for row in rows), 'metrics': rows}
    if output_json:
        Path(output_json).parent.mkdir(parents=True, exist_ok=True)
        Path(output_json).write_text(json.dumps(report, indent=2) + '\n')
    if output_markdown:
        Path(output_markdown).parent.mkdir(parents=True, exist_ok=True)
        Path(output_markdown).write_text(_markdown(rows))
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--thresholds',
        default=(
            'systems/aerostack2_gazebo/validation/'
            'acceptance_thresholds.json'))
    parser.add_argument('--metrics', help='Metrics JSON to evaluate')
    parser.add_argument('--output-json')
    parser.add_argument('--output-markdown')
    args = parser.parse_args()

    thresholds = _load_thresholds(args.thresholds)
    if not args.metrics:
        print(json.dumps({
            'thresholds': args.thresholds,
            'sections': list(thresholds['metric_contract'].keys()),
            'contract_valid': True,
        }, indent=2))
        return

    metrics = json.loads(Path(args.metrics).read_text())
    rows = _flatten_results(thresholds, metrics)
    report = _write_outputs(rows, args.output_json, args.output_markdown)
    print(json.dumps(report, indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
