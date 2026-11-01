import argparse
import time

from wildnav_pkg.satellite_cache import SatelliteTileCache


def seed(cache, tiles, retries=3, retry_delay_s=2.0, sleep=time.sleep):
    """Fetch every tile's image and features; tiles that fail are tried
    again up to `retries` more times, retry_delay_s apart. A single failed
    download (about one seeding in three for the simulated site's 121
    tiles) otherwise left a hole that the terrain builder refuses
    (tools/gps_denied_sim_test.md, "Seeding"). Returns the tiles still
    missing."""
    missing = list(tiles)
    for attempt in range(retries + 1):
        if attempt:
            sleep(retry_delay_s)
            # The cache holds a failed tile back for a minute.
            cache.failed_until.clear()
        missing = [tile for tile in missing
                   if cache.get_features(*tile, allow_download=True) is None]
        if not missing:
            break
    return missing


def main():
    parser = argparse.ArgumentParser(description='Seed WildNav satellite tile cache.')
    parser.add_argument('--lat', type=float, required=True)
    parser.add_argument('--lon', type=float, required=True)
    parser.add_argument(
        '--origin-lat', type=float, default=None,
        help='Synthetic-world origin latitude; must match the initial_lat '
             'the wildnav node runs with (defaults to --lat).')
    parser.add_argument(
        '--origin-lon', type=float, default=None,
        help='Synthetic-world origin longitude; must match the initial_lon '
             'the wildnav node runs with (defaults to --lon).')
    parser.add_argument('--cache-dir', default='/data/wildnav_cache')
    parser.add_argument('--tile-url-template', default=(
        'https://server.arcgisonline.com/ArcGIS/rest/services/'
        'World_Imagery/MapServer/tile/{z}/{y}/{x}'
    ))
    parser.add_argument('--zoom', type=int, default=18)
    parser.add_argument('--search-radius-m', type=float, default=250.0)
    parser.add_argument('--max-tiles', type=int, default=49)
    parser.add_argument('--feature-backend', default='SIFT')
    parser.add_argument('--max-features', type=int, default=1500)
    parser.add_argument('--request-timeout', type=float, default=30.0)
    parser.add_argument('--retries', type=int, default=3,
                        help='further attempts for tiles that failed')
    parser.add_argument('--retry-delay-s', type=float, default=2.0)
    args = parser.parse_args()

    cache = SatelliteTileCache(
        cache_dir=args.cache_dir,
        url_template=args.tile_url_template,
        zoom=args.zoom,
        feature_backend=args.feature_backend,
        max_features=args.max_features,
        request_timeout=args.request_timeout,
        origin_lat=args.origin_lat if args.origin_lat is not None else args.lat,
        origin_lon=args.origin_lon if args.origin_lon is not None else args.lon,
    )
    tiles = cache.candidate_tiles(
        args.lat, args.lon, args.search_radius_m, args.max_tiles
    )
    missing = seed(cache, tiles, args.retries, args.retry_delay_s)
    seeded = len(tiles) - len(missing)
    if missing:
        print(f'Could not seed {len(missing)} WildNav tiles after '
              f'{args.retries} retries: {missing}')
    if seeded == 0:
        raise RuntimeError('failed to seed any WildNav satellite tiles')
    print(
        f'Seeded {seeded}/{len(tiles)} WildNav tiles at {args.cache_dir} '
        f'for {args.lat:.6f},{args.lon:.6f}'
    )


if __name__ == '__main__':
    main()
