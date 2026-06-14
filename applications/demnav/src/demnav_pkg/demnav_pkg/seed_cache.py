import argparse

from demnav_pkg.heightmap import HeightMapCache


def main():
    parser = argparse.ArgumentParser(description='Seed DemNav DEM cache.')
    parser.add_argument('--lat', type=float, required=True)
    parser.add_argument('--lon', type=float, required=True)
    parser.add_argument('--area-km', type=float, default=10.0)
    parser.add_argument('--resolution-m', type=float, default=30.0)
    parser.add_argument('--cache-dir', default='/data/demnav_cache')
    args = parser.parse_args()

    HeightMapCache(
        lat=args.lat,
        lon=args.lon,
        area_km=args.area_km,
        resolution_m=args.resolution_m,
        cache_dir=args.cache_dir,
    )
    print(
        f'Seeded DemNav DEM cache at {args.cache_dir} for '
        f'{args.lat:.6f},{args.lon:.6f}'
    )


if __name__ == '__main__':
    main()
