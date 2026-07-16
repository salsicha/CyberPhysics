import argparse

from wildnav_pkg.satellite_cache import SatelliteTileCache


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
    seeded = 0
    for x, y in tiles:
        if cache.get_features(x, y, allow_download=True) is not None:
            seeded += 1
    if seeded == 0:
        raise RuntimeError('failed to seed any WildNav satellite tiles')
    print(
        f'Seeded {seeded}/{len(tiles)} WildNav tiles at {args.cache_dir} '
        f'for {args.lat:.6f},{args.lon:.6f}'
    )


if __name__ == '__main__':
    main()
