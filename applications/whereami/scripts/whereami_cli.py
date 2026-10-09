#!/usr/bin/env python3
"""The vendored `whereami` CLI with the container's WiFi scanner.

Runs whereami's own command line unchanged (learn, train, locations,
crossval, predict, predict_proba, rename) after replacing its
access_points.get_scanner() with whereami_localizer.make_scanner(), so
training uses exactly the scanner the ROS node uses. Configured by
environment variables:

    WHEREAMI_SCANNER       auto | nmcli | iwlist | file   (default auto)
    WHEREAMI_DEVICE        WiFi interface, e.g. wlan0      (default: any)
    WHEREAMI_SCAN_FILE     JSON scan to replay when WHEREAMI_SCANNER=file
    WHEREAMI_NMCLI_RESCAN  nmcli --rescan: yes | auto | no (default yes, so
                           every training sample is a fresh scan)

    python3 /scripts/whereami_cli.py learn -l kitchen -n 10
"""
import os
import sys

from whereami_localizer import ScanError, make_scanner


def scanner_from_env(device=''):
    env = os.environ.get
    return make_scanner(env('WHEREAMI_SCANNER', 'auto'),
                        device or env('WHEREAMI_DEVICE', ''),
                        scan_file=env('WHEREAMI_SCAN_FILE', ''),
                        rescan=env('WHEREAMI_NMCLI_RESCAN', 'yes'))


def install_scanner(factory=scanner_from_env):
    """Point the vendored modules' get_scanner at `factory`."""
    import whereami.get_data
    import whereami.predict
    whereami.get_data.get_scanner = factory
    whereami.predict.get_scanner = factory


def main():
    install_scanner()
    from whereami.__main__ import main as whereami_main
    try:
        whereami_main()
    except (ScanError, ValueError) as exc:
        sys.exit(f'whereami: {exc}')


if __name__ == '__main__':
    main()
