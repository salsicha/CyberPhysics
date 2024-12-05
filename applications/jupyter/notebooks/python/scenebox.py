"""Endpoint for checking image metadata for glare

RUN APP:
export FLASK_APP=sb.py
flask run

TEST:
curl -X POST -H "Content-Type: application/json" \
    -d '{"lat": 49.2699648, "lon": -123.1290368, \
        "epoch": 1588704959.321, "orientation": -10.2}' \
    http://127.0.0.1:5000

EXPECTED RESPONSE:
{
“glare”: “false”,
}

"""


import astropy.coordinates as SkyCoord
from astropy.time import Time
import astropy.units as u

import json

from flask import Flask, request, jsonify
from traitlets import Bool

app = Flask(__name__)


def compute_altaz(ori: float, epoch: float, lat: float, lon: float) -> Bool:
    """ Checks image metadata for possible glare

    Parameters
    ----------
    ori : float
        Azimuth of car

    epoch : float
        Epoch time of image

    lat : float
        Latitude of car

    lon : float
        Longitude of car

    Returns
    ------
    bool
        Possible glare

    """

    # Thresholds
    max_alt = 45.0
    max_az_sep = 30.0

    # Car alt is unknown:
    car_alt = 0.0

    # Create time and position objects
    car_location = SkyCoord.EarthLocation(lon=lon, lat=lat)
    utc_time = Time(epoch, scale='utc', format='unix')

    # Compute position of sun, given time and gps position
    car_altaz = SkyCoord.AltAz(location=car_location, obstime=utc_time)
    sun_skycoord = SkyCoord.get_sun(utc_time)
    sun_altaz = sun_skycoord.transform_to(car_altaz)

    # Compute separation of sun and car azimuths
    alt = sun_altaz.alt.degree
    sun_az = sun_altaz.az.degree
    sun_coord = SkyCoord.SkyCoord(sun_az * u.deg, car_alt * u.deg)
    car_coord = SkyCoord.SkyCoord(ori * u.deg, car_alt * u.deg)
    azimuth_sep = sun_coord.separation(car_coord)

    # Test assumed possible glare thresholds
    if alt < max_alt and azimuth_sep.deg < max_az_sep:
        return True

    return False


@app.route("/", methods=['POST'])
def check_glare():
    """ Endpoint checks if glare is possible based on metadata """

    try:
        record = json.loads(request.data)

        orientation = record.get("orientation")
        epoch = record.get("epoch")
        lat = record.get("lat")
        lon = record.get("lon")

        glare = compute_altaz(orientation, epoch, lat, lon)
    except Exception:
        return jsonify({"status": "error"})

    return jsonify({'glare': str(glare).lower()})
