#!/usr/bin/env python3
"""Score Aerodrone navigation robustness telemetry."""

import argparse
import json
import math
from pathlib import Path


def _finite_or_default(value, default=0.0):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def _load_suite(path, test_id):
    suite = json.loads(Path(path).read_text())
    tests = suite.get('tests', [])
    if test_id:
        tests = [test for test in tests if test['id'] == test_id]
        if not tests:
            raise SystemExit(f'unknown robustness test id: {test_id}')
    return suite, tests


def _validate_contract(suite, tests):
    errors = []
    if not suite.get('global_thresholds'):
        errors.append('missing global_thresholds')
    if not tests:
        errors.append('missing tests')
    required = ['id', 'mission_id', 'faults', 'expected_behavior', 'thresholds']
    for test in tests:
        for field in required:
            if field not in test:
                errors.append(f'{test.get("id", "?")}: missing {field}')
        if not test.get('faults'):
            errors.append(f'{test.get("id", "?")}: missing fault injections')
    return errors


def _test_samples(samples, test_id):
    selected = [sample for sample in samples if sample.get('test_id') == test_id]
    return selected or samples


def _fraction(samples, predicate):
    if not samples:
        return 0.0
    return sum(1 for sample in samples if predicate(sample)) / len(samples)


def _max_metric(samples, name):
    return max(
        (_finite_or_default(sample.get(name), 0.0) for sample in samples),
        default=0.0)


def _score_test(suite, test, all_samples):
    samples = _test_samples(all_samples, test['id'])
    thresholds = {**suite['global_thresholds'], **test.get('thresholds', {})}
    expected = test.get('expected_behavior', {})
    allowed_sources = set(expected.get('allowed_fallback_sources', []))
    max_horizontal = _max_metric(samples, 'horizontal_error_m')
    max_vertical = _max_metric(samples, 'vertical_error_m')
    geofence_violations = sum(
        1 for sample in samples if sample.get('geofence_violation'))
    safe_command_fraction = _fraction(
        samples, lambda sample: sample.get('command_safe', True))
    demnav_valid_fraction = _fraction(
        samples, lambda sample: sample.get('demnav_valid', False))
    wildnav_valid_fraction = _fraction(
        samples, lambda sample: sample.get('wildnav_valid', False))
    stale_cache_corrections = sum(
        1 for sample in samples
        if sample.get('stale_cache_used')
        and str(sample.get('correction_source', '')) in {'demnav', 'wildnav'})

    demnav_false_positives = 0
    wildnav_false_positives = 0
    for sample in samples:
        source = str(sample.get('correction_source', ''))
        confidence = _finite_or_default(
            sample.get('correction_confidence'), 0.0)
        if confidence <= 0.0:
            continue
        if source == 'demnav' and not sample.get('demnav_valid', False):
            demnav_false_positives += 1
        if source == 'wildnav' and not sample.get('wildnav_valid', False):
            wildnav_false_positives += 1
    false_positive_corrections = demnav_false_positives + wildnav_false_positives

    fallback_violations = 0
    if allowed_sources:
        for sample in samples:
            source = str(sample.get('correction_source', ''))
            if source and source not in allowed_sources:
                fallback_violations += 1

    pass_fail = {
        'horizontal_error': max_horizontal <= float(
            thresholds['max_horizontal_error_m']),
        'vertical_error': max_vertical <= float(
            thresholds['max_vertical_error_m']),
        'geofence': geofence_violations <= int(
            thresholds['max_geofence_violations']),
        'safe_commands': safe_command_fraction >= float(
            thresholds['min_safe_command_fraction']),
        'false_positive_corrections': false_positive_corrections <= int(
            expected.get(
                'max_false_positive_corrections',
                thresholds['max_false_positive_corrections'])),
        'fallback_sources': fallback_violations == 0,
    }
    if 'max_demnav_false_positive_corrections' in expected:
        pass_fail['demnav_false_positive_corrections'] = (
            demnav_false_positives <= int(
                expected['max_demnav_false_positive_corrections']))
    if 'max_wildnav_false_positive_corrections' in expected:
        pass_fail['wildnav_false_positive_corrections'] = (
            wildnav_false_positives <= int(
                expected['max_wildnav_false_positive_corrections']))
    if 'min_demnav_valid_fraction' in expected:
        pass_fail['demnav_valid_fraction'] = demnav_valid_fraction >= float(
            expected['min_demnav_valid_fraction'])
    if 'min_wildnav_valid_fraction' in expected:
        pass_fail['wildnav_valid_fraction'] = wildnav_valid_fraction >= float(
            expected['min_wildnav_valid_fraction'])
    if expected.get('demnav_should_reject'):
        pass_fail['demnav_rejected'] = demnav_valid_fraction == 0.0
    if expected.get('wildnav_should_reject'):
        pass_fail['wildnav_rejected'] = wildnav_valid_fraction == 0.0
    if 'max_stale_cache_corrections' in expected:
        pass_fail['stale_cache_rejected'] = stale_cache_corrections <= int(
            expected['max_stale_cache_corrections'])
    if thresholds.get('require_landed'):
        pass_fail['landed'] = any(
            sample.get('landed', False) for sample in samples)

    return {
        'test_id': test['id'],
        'mission_id': test['mission_id'],
        'samples': len(samples),
        'max_horizontal_error_m': max_horizontal,
        'max_vertical_error_m': max_vertical,
        'geofence_violations': geofence_violations,
        'safe_command_fraction': safe_command_fraction,
        'demnav_valid_fraction': demnav_valid_fraction,
        'wildnav_valid_fraction': wildnav_valid_fraction,
        'demnav_false_positive_corrections': demnav_false_positives,
        'wildnav_false_positive_corrections': wildnav_false_positives,
        'false_positive_corrections': false_positive_corrections,
        'fallback_violations': fallback_violations,
        'stale_cache_corrections': stale_cache_corrections,
        'pass_fail': pass_fail,
        'passed': bool(samples) and all(pass_fail.values()),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--suite',
        default=(
            'systems/aerostack2_gazebo/validation/'
            'navigation_robustness_tests.json'))
    parser.add_argument('--test-id')
    parser.add_argument(
        '--telemetry',
        help='JSON file with {"samples": [...]} or a raw sample list')
    args = parser.parse_args()

    suite, tests = _load_suite(args.suite, args.test_id)
    errors = _validate_contract(suite, tests)
    if errors:
        raise SystemExit('invalid robustness contract: ' + '; '.join(errors))

    if not args.telemetry:
        print(json.dumps({
            'suite': args.suite,
            'tests': [test['id'] for test in tests],
            'contract_valid': True,
        }, indent=2))
        return

    telemetry = json.loads(Path(args.telemetry).read_text())
    samples = telemetry.get(
        'samples', telemetry if isinstance(telemetry, list) else [])
    results = [_score_test(suite, test, samples) for test in tests]
    print(json.dumps({
        'results': results,
        'passed': all(result['passed'] for result in results),
    }, indent=2))


if __name__ == '__main__':
    main()
