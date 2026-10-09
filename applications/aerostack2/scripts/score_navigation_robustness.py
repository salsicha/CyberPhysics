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


def _test_samples(samples, test_id, allow_untagged=False):
    """Samples tagged with test_id.

    Samples belonging to other tests are never scored against this one.
    Telemetry with no test tags at all is attributed to the test only when
    the caller scores a single, explicitly selected test.
    """
    selected = [sample for sample in samples if sample.get('test_id') == test_id]
    if not selected and allow_untagged and not any(
            sample.get('test_id') for sample in samples):
        return list(samples)
    return selected


def _source_components(sample):
    """Correction sources active in a sample.

    The fusion node joins simultaneously active sources with '+', e.g.
    'demnav+wildnav'. 'dead_reckoning' and '' mean no correction.
    """
    source = str(sample.get('correction_source', '') or '')
    return [part for part in source.split('+') if part]


def _contract_source(components):
    """Map fusion source components onto the contract's source labels:
    a combination of several sources is the contract's 'fused' source."""
    if len(components) > 1:
        return 'fused'
    return components[0] if components else ''


def _fraction(samples, predicate):
    if not samples:
        return 0.0
    return sum(1 for sample in samples if predicate(sample)) / len(samples)


def _metric(sample, name):
    """A finite metric value, or None when missing/non-numeric/non-finite."""
    return _finite_or_default(sample.get(name), None)


def _max_metric(samples, name):
    """Maximum of a metric; an unscorable sample makes the maximum inf so the
    threshold fails instead of silently scoring that sample as zero error."""
    values = [_metric(sample, name) for sample in samples]
    if any(value is None for value in values):
        return float('inf')
    return max(values, default=0.0)


def _score_test(suite, test, all_samples, allow_untagged=False):
    samples = _test_samples(all_samples, test['id'], allow_untagged)
    thresholds = {**suite['global_thresholds'], **test.get('thresholds', {})}
    expected = test.get('expected_behavior', {})
    allowed_sources = set(expected.get('allowed_fallback_sources', []))
    max_horizontal = _max_metric(samples, 'horizontal_error_m')
    max_vertical = _max_metric(samples, 'vertical_error_m')
    invalid_metric_samples = sum(
        1 for sample in samples
        if _metric(sample, 'horizontal_error_m') is None
        or _metric(sample, 'vertical_error_m') is None)
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
        and {'demnav', 'wildnav'} & set(_source_components(sample)))

    demnav_false_positives = 0
    wildnav_false_positives = 0
    for sample in samples:
        components = _source_components(sample)
        confidence = _finite_or_default(
            sample.get('correction_confidence'), 0.0)
        if confidence <= 0.0:
            continue
        if 'demnav' in components and not sample.get('demnav_valid', False):
            demnav_false_positives += 1
        if 'wildnav' in components and not sample.get('wildnav_valid', False):
            wildnav_false_positives += 1
    false_positive_corrections = demnav_false_positives + wildnav_false_positives

    fallback_violations = 0
    if allowed_sources:
        for sample in samples:
            source = _contract_source(_source_components(sample))
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
        'invalid_metric_samples': invalid_metric_samples,
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
            'applications/aerostack2/validation/'
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
    # Untagged telemetry can only be attributed to one explicitly chosen test.
    allow_untagged = bool(args.test_id)
    results = [
        _score_test(suite, test, samples, allow_untagged) for test in tests]
    print(json.dumps({
        'results': results,
        'passed': all(result['passed'] for result in results),
    }, indent=2))


if __name__ == '__main__':
    main()
