import time
import numpy as np
from datetime import datetime, timedelta
import pytz
import logging
import math
import os
from PIL import Image, ImageStat

try:
    from astral.location import LocationInfo
    from astral.sun import sun, elevation
except ImportError:
    raise ImportError("Astral and Pytz are required for Holy Grail mode.")

class HolyGrailController:
    STANDARD_APERTURES_F = [1.4, 1.8, 2.0, 2.8, 4.0, 5.6, 8.0, 11, 16, 22]
    STANDARD_ISOS = [100, 125, 160, 200, 250, 320, 400, 500, 640, 800, 1000, 1250, 1600, 2000, 2500, 3200, 4000, 5000, 6400, 12800, 25600]
    STANDARD_SHUTTER_SPEEDS = {'30': 30.0, '25': 25.0, '20': 20.0, '15': 15.0, '13': 13.0, '10': 10.0, '8': 8.0, '6': 6.0, '5': 5.0, '4': 4.0, '3.2': 3.2, '2.5': 2.5, '2': 2.0, '1.6': 1.6, '1.3': 1.3, '1': 1.0, '0.8': 0.8, '0.6': 0.6, '0.5': 0.5, '0.4': 0.4, '1/3': 0.3333, '1/4': 0.25, '1/5': 0.2, '1/6': 0.1667, '1/8': 0.125, '1/10': 0.1, '1/13': 0.0769, '1/15': 0.0667, '1/20': 0.05, '1/25': 0.04, '1/30': 0.0333, '1/40': 0.025, '1/50': 0.02, '1/60': 0.0167, '1/80': 0.0125, '1/100': 0.01, '1/125': 0.008, '1/160': 0.0063, '1/200': 0.005, '1/250': 0.004, '1/320': 0.0031, '1/400': 0.0025, '1/500': 0.002, '1/640': 0.0016, '1/800': 0.0013, '1/1000': 0.001, '1/1250': 0.0008, '1/1600': 0.0006, '1/2000': 0.0005, '1/2500': 0.0004, '1/3200': 0.0003, '1/4000': 0.00025, '1/5000': 0.0002, '1/6400': 0.000156, '1/8000': 0.000125}
    SHUTTER_VALUES = sorted(STANDARD_SHUTTER_SPEEDS.values())
    SHUTTER_STRINGS = sorted(STANDARD_SHUTTER_SPEEDS, key=STANDARD_SHUTTER_SPEEDS.get)
    DEFAULT_EV_TABLE = { -20.0: [-5.0, 3400], -12.0: [-3.0, 3600], -6.0:  [1.0, 4000], 0.0: [8.0, 5200], 6.0: [12.0, 5800], 20.0: [14.0, 5500] }
    MIN_INTERVAL_SAFETY_BUFFER_S = 3.0
    INTERVAL_SMOOTHING_FACTOR = 0.025

    def __init__(self, lat, lon, settings_table_dict, day_interval_s, sunset_interval_s, night_interval_s,
                 timezone_str=None, interval_smoothing_factor=None, iso_transition_duration_frames=30):
        self.latitude, self.longitude = lat, lon
        self.day_interval_s, self.sunset_interval_s, self.night_interval_s = day_interval_s, sunset_interval_s, night_interval_s
        self.anchor_offset, self.reactive_ev_offset, self.last_exposure_duration = 0.0, 0.0, 1.0
        self.target_brightness, self.deadband_threshold, self.last_sun_elevation = 115, 10.0, 0.0
        self.last_used_interval = 0.0
        self.interval_smoothing_factor = interval_smoothing_factor if interval_smoothing_factor is not None else self.INTERVAL_SMOOTHING_FACTOR
        self.is_in_iso_transition = False
        self.iso_transition_frame_count = 0
        self.iso_transition_duration_frames = iso_transition_duration_frames
        
        if timezone_str:
            self.timezone_str = timezone_str
        else:
            try:
                tz_path = os.path.realpath('/etc/localtime')
                self.timezone_str = tz_path.split('/usr/share/zoneinfo/')[-1]
                logging.info("Auto-detected timezone from system.")
            except Exception:
                self.timezone_str = 'UTC'
                logging.warning("Could not auto-detect timezone, defaulting to UTC. Please set manually.")
        
        ev_table = settings_table_dict or self.DEFAULT_EV_TABLE
        table_items = sorted(list(ev_table.items()), key=lambda item: item[0])
        self.ev_sun_angles = np.array([item[0] for item in table_items])
        self.target_evs = np.array([item[1][0] for item in table_items])
        self.kelvin_vals = np.array([item[1][1] for item in table_items])
        
        self.interval_sun_angles, self.target_intervals = None, None

        try:
            self.tz = pytz.timezone(self.timezone_str)
            self.location = LocationInfo("PiSlider", "Earth", self.timezone_str, self.latitude, self.longitude)
            logging.info(f"HolyGrailController initialized with timezone: {self.timezone_str}")
            
            now = datetime.now(self.tz)
            self._create_dynamic_interval_table(now)
            self._update_sun_elevation(now)
            
            initial_blended_interval = self._eased_interp(self.last_sun_elevation, self.interval_sun_angles, self.target_intervals)
            self.last_used_interval = initial_blended_interval
            logging.info(f"Dynamically set initial interval to {self.last_used_interval:.2f}s based on current sun elevation.")
        except Exception as e: raise ValueError(f"Invalid timezone or location for Holy Grail: {e}")

    def _create_dynamic_interval_table(self, now):
        try:
            s = sun(self.location.observer, date=now.date(), tzinfo=self.tz)
            sunset_dt = s['sunset']
            
            pre_ramp_start_dt = sunset_dt - timedelta(minutes=50)
            pre_ramp_end_dt = sunset_dt - timedelta(minutes=10)
            post_ramp_start_dt = sunset_dt + timedelta(minutes=10)
            post_ramp_end_dt = sunset_dt + timedelta(minutes=50)

            # --- THE FIX ---
            # Remove the 'at=' keyword argument
            pre_ramp_start_angle = elevation(self.location.observer, pre_ramp_start_dt)
            pre_ramp_end_angle = elevation(self.location.observer, pre_ramp_end_dt)
            post_ramp_start_angle = elevation(self.location.observer, post_ramp_start_dt)
            post_ramp_end_angle = elevation(self.location.observer, post_ramp_end_dt)

            keyframes = {-18.0: self.night_interval_s, post_ramp_end_angle: self.night_interval_s, post_ramp_start_angle: self.sunset_interval_s, pre_ramp_end_angle: self.sunset_interval_s, pre_ramp_start_angle: self.day_interval_s, 20.0: self.day_interval_s}
            sorted_angles = sorted(keyframes.keys())
            self.interval_sun_angles, self.target_intervals = np.array(sorted_angles), np.array([keyframes[angle] for angle in sorted_angles])
        except Exception as e:
            logging.error(f"Could not calculate dynamic sunset-based ramps: {e}. Falling back to simple table.", exc_info=True)
            self.interval_sun_angles, self.target_intervals = np.array([-18.0, -6.0, 0.0, 20.0]), np.array([self.night_interval_s, self.sunset_interval_s, self.day_interval_s, self.day_interval_s])

    def _update_sun_elevation(self, execution_datetime):
        try:
            # --- THE FIX ---
            # Remove the 'at=' keyword argument
            self.last_sun_elevation = elevation(self.location.observer, execution_datetime)
        except Exception as e:
            logging.error(f"Could not calculate sun elevation: {e}", exc_info=True)
            self.last_sun_elevation = 0

    def calibrate(self, image_path, camera_iso, camera_aperture, camera_shutter_s):
        logging.info(f"Calibrating with settings: ISO {camera_iso}, f/{camera_aperture}, {camera_shutter_s}s")
        self.reactive_ev_offset, self.anchor_offset = 0.0, 0.0
        measured_ev = self._calculate_ev_from_settings(camera_iso, camera_aperture, camera_shutter_s)
        measured_brightness = self._get_image_brightness(image_path)
        if measured_brightness is None: return False
        metering_error_ev = math.log2(self.target_brightness / measured_brightness)
        true_scene_ev = measured_ev - metering_error_ev
        
        self._update_sun_elevation(datetime.now(self.tz))
        
        predictive_ev_at_calib = np.interp(self.last_sun_elevation, self.ev_sun_angles, self.target_evs)
        self.anchor_offset = true_scene_ev - predictive_ev_at_calib
        logging.info(f"Calibration complete. Anchor Offset set to: {self.anchor_offset:+.2f} EV")
        return True

    @staticmethod
    def _ease_in_out_normalized(t):
        if t < 0.5: return 2 * t * t
        return -1 + (4 - 2 * t) * t

    def _eased_interp(self, x, xp, fp):
        i = np.searchsorted(xp, x, side='right')
        if i == 0: return fp[0]
        if i >= len(xp): return fp[-1]
        x_start, x_end, y_start, y_end = xp[i-1], xp[i], fp[i-1], fp[i]
        segment_width = x_end - x_start
        if segment_width == 0: return y_start
        progress = (x - x_start) / segment_width
        return y_start + self._ease_in_out_normalized(progress) * (y_end - y_start)
    
    def _calculate_ev_from_settings(self, iso, aperture, shutter_s):
        if shutter_s == 0: return -np.inf
        return math.log2((aperture**2 * 100) / (iso * shutter_s))

    def get_next_shot_parameters(self, min_aperture, max_aperture, day_iso, night_native_iso, max_transition_iso, execution_timestamp=None):
        shot_datetime = datetime.fromtimestamp(execution_timestamp, self.tz) if execution_timestamp else datetime.now(self.tz)
        
        self._update_sun_elevation(shot_datetime)
        
        predictive_ev_now = np.interp(self.last_sun_elevation, self.ev_sun_angles, self.target_evs)
        ideal_target_ev = predictive_ev_now + self.anchor_offset + self.reactive_ev_offset
        logging.info(f"Ideal Target EV: {ideal_target_ev:.2f} (Sun Angle: {self.last_sun_elevation:.2f}°, Pred. EV: {predictive_ev_now:.2f}, Anchor: {self.anchor_offset:+.2f}, Reactive: {self.reactive_ev_offset:+.2f})")
        target_kelvin = int(np.interp(self.last_sun_elevation, self.ev_sun_angles, self.kelvin_vals))
        blended_interval = self._eased_interp(self.last_sun_elevation, self.interval_sun_angles, self.target_intervals)
        
        exposure_settings = {}
        current_aperture = min_aperture

        if self.is_in_iso_transition:
            self.iso_transition_frame_count += 1
            progress = min(1.0, self.iso_transition_frame_count / self.iso_transition_duration_frames)
            eased_progress = self._ease_in_out_normalized(progress)
            current_iso = max_transition_iso - (eased_progress * (max_transition_iso - night_native_iso))
            bulb_duration_s = (current_aperture**2 * 100) / (2**ideal_target_ev * current_iso)
            logging.info(f"ISO Transition frame {self.iso_transition_frame_count}/{self.iso_transition_duration_frames}: ISO {current_iso:.0f}, Bulb {bulb_duration_s:.2f}s")
            exposure_settings = self._package_settings('bulb', current_iso, 0, bulb_duration_s, current_aperture, target_kelvin)
            if progress >= 1.0:
                self.is_in_iso_transition, self.iso_transition_frame_count = False, 0
                logging.info("ISO transition to Night Native ISO complete.")
        else:
            shutter_for_day_iso = (current_aperture**2 * 100) / (2**ideal_target_ev * day_iso)

            if shutter_for_day_iso <= 30.0:
                logging.info(f"Day/Bright state detected. Using Day ISO ({day_iso}).")
                exposure_settings = self._package_settings('normal', day_iso, shutter_for_day_iso, 0, current_aperture, target_kelvin)
            else:
                found_iso = False
                for iso_candidate in self.STANDARD_ISOS:
                    if iso_candidate <= day_iso: continue
                    if iso_candidate > max_transition_iso: break
                    shutter_s = (current_aperture**2 * 100) / (2**ideal_target_ev * iso_candidate)
                    if shutter_s <= 30.0:
                        logging.info(f"Dark Transition state detected. Found suitable ISO: {iso_candidate}")
                        exposure_settings = self._package_settings('normal', iso_candidate, shutter_s, 0, current_aperture, target_kelvin)
                        found_iso = True
                        break
                
                if not found_iso:
                    logging.info(f"Night state detected. Entering bulb mode and starting ISO transition from {max_transition_iso}.")
                    self.is_in_iso_transition, self.iso_transition_frame_count = True, 1
                    bulb_duration_s = (current_aperture**2 * 100) / (2**ideal_target_ev * max_transition_iso)
                    exposure_settings = self._package_settings('bulb', max_transition_iso, 0, bulb_duration_s, current_aperture, target_kelvin)

        actual_shutter_s = exposure_settings['bulb_duration'] if exposure_settings['mode'] == 'bulb' else self.get_shutter_s(exposure_settings['shutter'])
        actual_settings_ev = self._calculate_ev_from_settings(exposure_settings['iso'], float(exposure_settings['aperture']), actual_shutter_s)
        exposure_settings['actual_settings_ev'] = actual_settings_ev
        exposure_settings['xmp_ev_offset'] = ideal_target_ev - actual_settings_ev
        exposure_settings['reactive_ev_offset'] = self.reactive_ev_offset
        required_interval = self.last_exposure_duration + self.MIN_INTERVAL_SAFETY_BUFFER_S
        new_target_interval = max(blended_interval, required_interval)
        
        if self.last_used_interval == 0.0:
            final_interval = new_target_interval
        else:
            final_interval = (self.last_used_interval * (1 - self.interval_smoothing_factor)) + (new_target_interval * self.interval_smoothing_factor)
        
        self.last_used_interval = final_interval
        exposure_settings['target_interval'] = final_interval
        return exposure_settings

    def update_exposure_offset(self, image_path, xmp_correction_ev):
        measured_brightness = self._get_image_brightness(image_path)
        if measured_brightness is None: return
        measured_ev_error = math.log2(self.target_brightness / measured_brightness)
        environmental_ev_error = measured_ev_error - xmp_correction_ev
        if abs(environmental_ev_error) > 0.05:
            smoothing_factor = self._calculate_dynamic_smoothing_factor()
            self.reactive_ev_offset += environmental_ev_error * smoothing_factor
            self.reactive_ev_offset = np.clip(self.reactive_ev_offset, -1.5, 1.5)
            logging.debug(f"ReactiveUpdate: MeasuredError={measured_ev_error:+.2f}, XMP_Correction={xmp_correction_ev:+.2f}, FinalEnvError={environmental_ev_error:+.2f}, NewReactiveOffset={self.reactive_ev_offset:+.2f}")

    def _calculate_dynamic_smoothing_factor(self):
        base_factor = 0.02; sun_points = [-18, -12, -6, 0, 6, 12, 18]; sun_mults  = [1.0, 1.5, 2.5, 3.0, 2.5, 1.5, 1.0]
        return base_factor * np.interp(self.last_sun_elevation, sun_points, sun_mults)

    def _get_image_brightness(self, image_path: str) -> float | None:
        try:
            with Image.open(image_path) as img:
                img_gray = img.convert('L'); stats = ImageStat.Stat(img_gray)
                return max(1.0, stats.mean[0])
        except Exception as e:
            logging.error(f"Failed to analyze image brightness for '{image_path}': {e}"); return None

    def _find_closest(self, value, candidates, return_values=None):
        candidates = np.asarray(candidates); idx = (np.abs(candidates - value)).argmin()
        return return_values[idx] if return_values is not None else candidates[idx]
    
    def get_shutter_s(self, shutter_str):
        if '/' in shutter_str: num, den = shutter_str.split('/'); return float(num) / float(den)
        return float(shutter_str)

    def _package_settings(self, mode, iso, shutter_s, bulb_s, aperture_f, kelvin):
        final_iso = self._find_closest(iso, self.STANDARD_ISOS)
        final_aperture = self._find_closest(aperture_f, self.STANDARD_APERTURES_F)
        shutter_str = 'bulb' if mode == 'bulb' else self._find_closest(shutter_s, self.SHUTTER_VALUES, self.SHUTTER_STRINGS)
        self.last_exposure_duration = bulb_s if mode == 'bulb' else self.get_shutter_s(shutter_str)
        return {'mode': mode, 'iso': final_iso, 'shutter': shutter_str, 'bulb_duration': bulb_s, 'aperture': f"{final_aperture:.1f}", 'kelvin': kelvin}
