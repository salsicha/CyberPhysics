import astropy.coordinates as SkyCoord
from astropy.time import Time
import astropy.units as u

# from astropy.coordinates import SkyCoord

# 0 = north, -90 = west, 90 = east
orientation = -10.2

# seconds since Jan 1, 1970, GMT
epoch = 1588704959.321

# Latitude and Longitude
lat = 49.2699648 * u.deg
lon = -123.1290368 * u.deg

car_location = SkyCoord.EarthLocation(lon=lon, lat=lat)
utc_time = Time(epoch, scale='utc', format='unix')

car_altaz = SkyCoord.AltAz(location=car_location, obstime=utc_time)
sun_skycoord = SkyCoord.get_sun(utc_time)
sun_altaz = sun_skycoord.transform_to(car_altaz)

print("orientation: ", orientation)
print("sun altaz alt: ", sun_altaz.alt.degree)
print("sun altaz az: ", sun_altaz.az.degree)

# print("lat: ", car_location.geodetic.lat.degree)
# print("lon: ", car_location.geodetic.lon.degree)

# azimuth_diff = abs(sun_altaz.az.degree - orientation)
altitude = sun_altaz.alt.degree


sun_coord = SkyCoord.SkyCoord(sun_altaz.az.degree * u.deg, 0 *u.deg)
car_coord = SkyCoord.SkyCoord(orientation * u.deg, 0 * u.deg)
sep = sun_coord.separation(car_coord)

print("sep: ", sep.deg)


glare = False

# if azimuth_diff < 30.0 and altitude < 45.0:
#     glare = True


