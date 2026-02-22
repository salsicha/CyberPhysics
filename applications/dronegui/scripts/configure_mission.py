#!/usr/bin/env python3

import json
import sys
import os

try:
    import requests
except ImportError:
    print("Error: 'requests' library is required. Install with: pip install requests")
    sys.exit(1)

def get_location():
    """Get current location based on IP address."""
    try:
        response = requests.get("https://ipinfo.io/json", timeout=10)
        response.raise_for_status()
        data = response.json()
        if "loc" in data:
            lat, lon = data["loc"].split(",")
            return float(lat), float(lon)
    except Exception as e:
        print(f"Failed to retrieve location: {e}")
    return None

def download_static_map(lat, lon, zoom=18, size=(640, 640), filename="map.png"):
    """
    Download a static map image.
    Note: This is a placeholder implementation. 
    Real implementation would require a map provider API key (e.g., Google Maps, Mapbox).
    """
    print(f"Downloading map for location ({lat}, {lon})...")
    # In a real scenario, you would make a request to a static map API.
    # For this demo, we will just create a placeholder file if it doesn't exist.
    if not os.path.exists(filename):
        with open(filename, "wb") as f:
            f.write(b"") # Create empty file as placeholder
    print(f"Map saved to {filename}")

def main():
    print("Configuring drone mission...")
    
    location = get_location()
    if location:
        lat, lon = location
        print(f"Detected location: {lat}, {lon}")
    else:
        print("Could not detect location. Using default.")
        lat, lon = 0.0, 0.0

    config = {
        "start_latitude": lat,
        "start_longitude": lon,
        "zoom_level": 18
    }

    with open("mission_config.json", "w") as f:
        json.dump(config, f, indent=4)
    print("Configuration saved to mission_config.json")

    download_static_map(lat, lon)

if __name__ == "__main__":
    main()