import numpy as np
from datetime import datetime
import pytz
import logging
import json
import os

# This is the corrected import for modern astral libraries (v2 and newer)
from astral.location import LocationInfo
from astral.sun import sun

class HolyGrailController:
    """
    The 'brains' of the Holy Grail timelapse.
    (Corrected for modern Astral v2.x/v3.x syntax)
    """
    DEFAULT_SETTINGS_TABLE = {
        # sun_angle (degrees): [target_EV, kelvin_temp, tint_val]
        "-20.0": [ -5.0, 3400, 10],
        "-12.0": [ -3.0, 3600, 8],
        "-6.0":  [  1.0, 4000, 5],
        "0.0":   [  8.0, 5200, 0],
        "6.0":   [ 12.0, 5800, 0],
        "20.0":  [ 14.0, 5500, -5]
    }

    def __init__(self, lat, lon, settings_table_str):
        self.latitude = lat
        self.longitude = lon
        
        try:
            with open('/etc/timezone') as f:
                self.timezone_str = f.read().strip()
        except (FileNotFoundError, IOError):
            logging.warning("Could not detect system timezone from /etc/timezone. Falling back to UTC.")
            self.timezone_str = 'UTC'
        logging.info(f"Auto-detected system timezone: {self.timezone_str}")
        
        try:
            self.settings_table = json.loads(settings_table_str)
            sorted_angles = sorted([float(k) for k in self.settings_table.keys()])
            self.sun_angles = np.array(sorted_angles)
            self.target_evs = np.array([self.settings_table[str(k)][0] for k in sorted_angles])
            self.kelvin_vals = np.array([self.settings_table[str(k)][1] for k in sorted_angles])
            self.tint_vals = np.array([self.settings_table[str(k)][2] for k in sorted_angles])
            logging.info("Holy Grail Controller initialized successfully with custom settings.")
        except Exception as e:
            logging.error(f"Failed to parse Holy Grail settings table: {e}. Falling back to default.")
            self.settings_table = self.DEFAULT_SETTINGS_TABLE
            sorted_angles = sorted([float(k) for k in self.settings_table.keys()])
            self.sun_angles = np.array(sorted_angles)
            self.target_evs = np.array([self.settings_table[str(k)][0] for k in sorted_angles])
            self.kelvin_vals = np.array([self.settings_table[str(k)][1] for k in sorted_angles])
            self.tint_vals = np.array([self.settings_table[str(k)][2] for k in sorted_angles])

        try:
            # This block uses the correct modern astral syntax
            self.location = LocationInfo(
                "PiSliderLocation",
                "Region",
                self.timezone_str,
                self.latitude,
                self.longitude
            )
            self.tz = pytz.timezone(self.timezone_str)
        except Exception as e:
            logging.error(f"Failed to initialize Astral location object: {e}", exc_info=True)
            raise ValueError("Invalid Latitude or Longitude for Holy Grail.")

    def get_current_settings(self, min_aperture, max_aperture, base_iso, max_iso):
        now = datetime.now(self.tz)
        
        try:
            # --- THIS IS THE CORRECTED CALL FOR MODERN ASTRAL ---
            # Call the 'sun' function and get the 'elevation' from the resulting dictionary
            s = sun(self.location.observer, date=now, tzinfo=self.tz)
            elevation = s['elevation']
        except Exception as e:
            logging.error(f"Could not calculate sun elevation: {e}")
            elevation = 0
            
        target_ev = np.interp(elevation, self.sun_angles, self.target_evs)
        target_kelvin = int(round(np.interp(elevation, self.sun_angles, self.kelvin_vals) / 100.0)) * 100
        target_tint = int(round(np.interp(elevation, self.sun_angles, self.tint_vals)))

        if min_aperture == max_aperture:
            target_aperture_val = min_aperture
        else:
            min_sun_angle = self.sun_angles[0]
            max_sun_angle = self.sun_angles[-1]
            if max_sun_angle == min_sun_angle:
                 target_aperture_val = min_aperture
            else:
                target_aperture_val = np.interp(elevation,
                                                [min_sun_angle, max_sun_angle],
                                                [min_aperture, max_aperture])
        
        final_aperture_val = np.clip(target_aperture_val, min(min_aperture, max_aperture), max(min_aperture, max_aperture))
        final_aperture_str = self._find_closest_aperture(final_aperture_val)

        exposure = self._calculate_exposure(target_ev, final_aperture_val, base_iso, max_iso)

        final_settings = {
            'aperture': final_aperture_str,
            'kelvin': target_kelvin,
            'tint': target_tint,
            **exposure
        }
        return final_settings

    def _calculate_exposure(self, target_ev, aperture_val, base_iso, max_iso):
        current_iso = base_iso
        required_time = (aperture_val**2) / (2**(target_ev + np.log2(current_iso / 100.0)))

        if 0.000125 < required_time <= 30:
            return { 'mode': 'standard', 'iso': current_iso, 'shutter': self._find_closest_shutter(required_time), 'bulb_duration': None }
        if required_time > 30:
            required_iso = current_iso * (required_time / 30.0)
            if required_iso <= max_iso:
                final_iso = self._find_closest_iso(required_iso)
                final_time = (aperture_val**2) / (2**(target_ev + np.log2(final_iso / 100.0)))
                return { 'mode': 'standard', 'iso': final_iso, 'shutter': self._find_closest_shutter(final_time), 'bulb_duration': None }
            else:
                final_time_at_max_iso = (aperture_val**2) / (2**(target_ev + np.log2(max_iso / 100.0)))
                return { 'mode': 'bulb', 'iso': max_iso, 'shutter': 'bulb', 'bulb_duration': final_time_at_max_iso }
        if required_time <= 0.000125:
            return { 'mode': 'standard', 'iso': base_iso, 'shutter': '1/8000', 'bulb_duration': None }

    def _find_closest_aperture(self, f_val):
        values = [1.0, 1.1, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.5, 2.8, 3.2, 3.5, 4.0, 4.5, 5.0, 5.6, 6.3, 7.1, 8.0, 9.0, 10, 11, 13, 14, 16, 18, 20, 22]
        strings = [f"{v:.1f}" for v in values]
        idx = (np.abs(np.array(values) - f_val)).argmin()
        return strings[idx]

    def _find_closest_shutter(self, time_sec):
        speeds = [30, 25, 20, 15, 13, 10, 8, 6, 5, 4, 3.2, 2.5, 2, 1.6, 1.3, 1, 0.8, 0.6, 0.5, 0.4, 1/3, 1/4, 1/5, 1/6, 1/8, 1/10, 1/13, 1/15, 1/20, 1/25, 1/30, 1/40, 1/50, 1/60, 1/80, 1/100, 1/125, 1/160, 1/200, 1/250, 1/320, 1/400, 1/500, 1/640, 1/800, 1/1000, 1/1250, 1/1600, 1/2000, 1/2500, 1/3200, 1/4000, 1/5000, 1/6400, 1/8000]
        strings = ['30', '25', '20', '15', '13', '10', '8', '6', '5', '4', '3.2', '2.5', '2', '1.6', '1.3', '1', '0.8', '0.6', '0.5', '0.4', '1/3', '1/4', '1/5', '1/6', '1/8', '1/10', '1/13', '1/15', '1/20', '1/25', '1/30', '1/40', '1/50', '1/60', '1/80', '1/100', '1/125', '1/160', '1/200', '1/250', '1/320', '1/400', '1/500', '1/640', '1/800', '1/1000', '1/1250', '1/1600', '1/2000', '1/2500', '1/3200', '1/4000', '1/5000', '1/6400', '1/8000']
        idx = (np.abs(np.array(speeds) - time_sec)).argmin()
        return strings[idx]

    def _find_closest_iso(self, iso_val):
        isos = [50, 64, 80, 100, 125, 160, 200, 250, 320, 400, 500, 640, 800, 1000, 1250, 1600, 2000, 2500, 3200, 4000, 5000, 6400, 8000, 10000, 12800, 16000, 20000, 25600]
        idx = (np.abs(np.array(isos) - iso_val)).argmin()
        return isos[idx]
