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
    from astral.sun import SunDirection
except ImportError:
    raise ImportError("Astral and Pytz are required for Holy Grail mode.")

class HolyGrailController:
    """
    Manages Holy Grail timelapse exposure with a dynamic, time-based interval ramping model.
    """
    STANDARD_APERTURES_F = [1.4, 1.8, 2.0, 2.8, 4.0, 5.6, 8.0, 11, 16, 22]
    STANDARD_ISOS = [100, 125, 160, 200, 250, 320, 400, 500, 640, 800, 1000, 1250, 1600, 2000, 2500, 3200, 4000, 5000, 6400, 12800, 25600]
    STANDARD_SHUTTER_SPEEDS = {
        '30': 30.0, '25': 25.0, '20': 20.0, '15': 15.0, '13': 13.0, '10': 10.0, '8': 8.0, '6': 6.0, '5': 5.0, '4': 4.0, '3.2': 3.2, '2.5': 2.5, '2': 2.0, '1.6': 1.6, '1.3': 1.3, '1': 1.0,
        '0.8': 0.8, '0.6': 0.6, '0.5': 0.5, '0.4': 0.4, '1/3': 1/3.0, '1/4': 1/4.0, '1/5': 1/5.0, '1/6': 1/6.0, '1/8': 1/8.0, '1/10': 1/10.0, '1/13': 1/13.0, '1/15': 1/15.0, '1/20': 1/20.0, '1/25': 1/25.0,
        '1/30': 1/30.0, '1/40': 1/40.0, '1/50': 1/50.0, '1/60': 1/60.0, '1/80': 1/80.0, '1/100': 1/100.0, '1/125': 1/125.0, '1/160': 1/160.0, '1/200': 1/200.0, '1/250': 1/250.0,
        '1/320': 1/320.0, '1/400': 1/400.0, '1/500': 1/500.0, '1/640': 1/640.0, '1/800': 1/800.0, '1/1000': 1/1000.0, '1/1250': 1/1250.0, '1/1600': 1/1600.0,
        '1/2000': 1/2000.0, '1/2500': 1/2500.0, '1/3200': 1/3200.0, '1/4000': 1/4000.0, '1/5000': 1/5000.0, '1/6400': 1/6400.0, '1/8000': 1/8000.0
    }
    SHUTTER_VALUES = sorted(STANDARD_SHUTTER_SPEEDS.values())
    SHUTTER_STRINGS = sorted(STANDARD_SHUTTER_SPEEDS, key=STANDARD_SHUTTER_SPEEDS.get)
    DEFAULT_EV_TABLE = { -20.0: [-5.0, 3400], -12.0: [-3.0, 3600], -6.0:  [1.0, 4000], 0.0: [8.0, 5200], 6.0: [12.0, 5800], 20.0: [14.0, 5500] }
    MIN_INTERVAL_SAFETY_BUFFER_S = 3.0
    INTERVAL_SMOOTHING_FACTOR = 0.025

    # --- START OF MODIFICATION: Add ISO transition parameters ---
    def __init__(self, lat, lon, settings_table_dict, day_interval_s, sunset_interval_s, night_interval_s,
                 interval_smoothing_factor=None, iso_transition_duration_frames=30):
    # --- END OF MODIFICATION ---
        self.latitude, self.longitude = lat, lon
        self.day_interval_s, self.sunset_interval_s, self.night_interval_s = day_interval_s, sunset_interval_s, night_interval_s
        self.anchor_ev, self.anchor_sun_angle, self.reactive_ev_offset, self.last_exposure_duration = None, None, 0.0, 1.0
        self.target_brightness, self.deadband_threshold, self.last_sun_elevation = 115, 10.0, 0.0
        self.last_used_interval = self.day_interval_s
        self.interval_smoothing_factor = interval_smoothing_factor if interval_smoothing_factor is not None else self.INTERVAL_SMOOTHING_FACTOR
        
        # --- START OF MODIFICATION: Add transition state variables ---
        self.is_in_iso_transition = False
        self.iso_transition_frame_count = 0
        self.iso_transition_duration_frames = iso_transition_duration_frames
        # --- END OF MODIFICATION ---

        try:
            tz_path = os.path.realpath('/etc/localtime')
            self.timezone_str = tz_path.split('/usr/share/zoneinfo/')[-1]
        except Exception: self.timezone_str = 'UTC'
        
        ev_table = settings_table_dict or self.DEFAULT_EV_TABLE
        ev_sorted_angles = sorted(ev_table.keys())
        self.ev_sun_angles = np.array(ev_sorted_angles)
        self.target_evs = np.array([ev_table[k][0] for k in ev_sorted_angles])
        self.kelvin_vals = np.array([ev_table[k][1] for k in ev_sorted_angles])
        
        self.interval_sun_angles = None
        self.target_intervals = None

        try:
            self.tz = pytz.timezone(self.timezone_str)
            self.location = LocationInfo("PiSlider", "Earth", self.timezone_str, self.latitude, self.longitude)
            logging.info(f"HolyGrailController initialized with timezone: {self.timezone_str}")
            self._create_dynamic_interval_table()
        except Exception as e:
            raise ValueError(f"Invalid timezone or location for Holy Grail: {e}")

    def _create_dynamic_interval_table(self):
        logging.info("Creating dynamic interval ramping table based on today's sunset.")
        now = datetime.now(self.tz)
        try:
            sunset_dt = self.location.sun(date=now.date(), local=True, direction=SunDirection.SETTING)['sunset']
            pre_ramp_start_dt = sunset_dt - timedelta(minutes=50)
            pre_ramp_end_dt = sunset_dt - timedelta(minutes=10)
            post_ramp_start_dt = sunset_dt + timedelta(minutes=10)
            post_ramp_end_dt = sunset_dt + timedelta(minutes=50)
            pre_ramp_start_angle = self.location.solar_elevation(pre_ramp_start_dt)
            pre_ramp_end_angle = self.location.solar_elevation(pre_ramp_end_dt)
            post_ramp_start_angle = self.location.solar_elevation(post_ramp_start_dt)
            post_ramp_end_angle = self.location.solar_elevation(post_ramp_end_dt)
            logging.info(f"Day->Sunset ramp: {pre_ramp_start_angle:.2f}° to {pre_ramp_end_angle:.2f}°")
            logging.info(f"Sunset->Night ramp: {post_ramp_start_angle:.2f}° to {post_ramp_end_angle:.2f}°")
            keyframes = { -18.0: self.night_interval_s, post_ramp_end_angle: self.night_interval_s, post_ramp_start_angle: self.sunset_interval_s, pre_ramp_end_angle: self.sunset_interval_s, pre_ramp_start_angle: self.day_interval_s, 20.0: self.day_interval_s }
            sorted_angles = sorted(keyframes.keys())
            self.interval_sun_angles = np.array(sorted_angles)
            self.target_intervals = np.array([keyframes[angle] for angle in sorted_angles])
        except Exception as e:
            logging.error(f"Could not calculate dynamic sunset-based ramps: {e}. Falling back to simple table.", exc_info=True)
            self.interval_sun_angles = np.array([-18.0, -6.0, 0.0, 20.0])
            self.target_intervals = np.array([self.night_interval_s, self.sunset_interval_s, self.day_interval_s, self.day_interval_s])

    def _update_sun_elevation(self, execution_timestamp):
        try:
            self.location.observer.date = datetime.fromtimestamp(execution_timestamp, self.tz)
            self.last_sun_elevation = self.location.observer.elevation
        except Exception as e: logging.error(f"Could not calculate sun elevation: {e}", exc_info=True)

    def calibrate(self, image_path, camera_iso, camera_aperture, camera_shutter_s):
        logging.info(f"Calibrating with user settings: ISO {camera_iso}, f/{camera_aperture}, {camera_shutter_s}s")
        measured_ev = math.log2((camera_aperture**2 * 100) / (camera_iso * camera_shutter_s))
        measured_brightness = self._get_image_brightness(image_path)
        if measured_brightness is None: return False
        metering_error_ev = math.log2(self.target_brightness / measured_brightness)
        self.anchor_ev = measured_ev - metering_error_ev
        self._update_sun_elevation(time.time())
        self.anchor_sun_angle = self.last_sun_elevation
        self.reactive_ev_offset = 0.0
        self.last_used_interval = self.day_interval_s
        logging.info(f"Calibration complete. Anchor EV set to: {self.anchor_ev:+.2f} at sun angle {self.anchor_sun_angle:.2f}°")
        return True

    @staticmethod
    def _ease_in_out_normalized(t):
        if t < 0.5: return 2 * t * t
        return -1 + (4 - 2 * t) * t

    def _eased_interp(self, x, xp, fp):
        i = np.searchsorted(xp, x, side='right')
        if i == 0: return fp[0]
        if i >= len(xp): return fp[-1]
        x_start, x_end = xp[i-1], xp[i]
        y_start, y_end = fp[i-1], fp[i]
        segment_width = x_end - x_start
        if segment_width == 0: return y_start
        progress = (x - x_start) / segment_width
        eased_progress = self._ease_in_out_normalized(progress)
        return y_start + eased_progress * (y_end - y_start)

    # --- START OF MODIFICATION: Complete rework for "Crossfade" State Machine ---
    def get_next_shot_parameters(self, min_aperture, max_aperture, day_iso, night_native_iso, max_transition_iso, execution_timestamp=None):
        if not execution_timestamp:
            execution_timestamp = time.time()
        self._update_sun_elevation(execution_timestamp)

        predictive_ev_now = np.interp(self.last_sun_elevation, self.ev_sun_angles, self.target_evs)
        if self.anchor_ev is not None and self.anchor_sun_angle is not None:
            predictive_ev_at_anchor = np.interp(self.anchor_sun_angle, self.ev_sun_angles, self.target_evs)
            delta_ev = predictive_ev_now - predictive_ev_at_anchor
            target_ev = self.anchor_ev + delta_ev + self.reactive_ev_offset
        else:
            target_ev = predictive_ev_now + self.reactive_ev_offset
        logging.info(f"Target EV: {target_ev:.2f} (Reactive Offset: {self.reactive_ev_offset:+.2f})")

        target_kelvin = int(np.interp(self.last_sun_elevation, self.ev_sun_angles, self.kelvin_vals))
        blended_interval = self._eased_interp(self.last_sun_elevation, self.interval_sun_angles, self.target_intervals)
        exposure_duration = 0
        exposure_settings = {}
        
        current_aperture = min_aperture # Aperture is assumed constant for this logic

        # --- STATE 1: In the ISO ramp-down transition ---
        if self.is_in_iso_transition:
            self.iso_transition_frame_count += 1
            progress = min(1.0, self.iso_transition_frame_count / self.iso_transition_duration_frames)
            eased_progress = self._ease_in_out_normalized(progress)
            
            # Interpolate ISO from high to low
            transition_iso_range = max_transition_iso - night_native_iso
            current_iso = max_transition_iso - (eased_progress * transition_iso_range)

            # Calculate the required bulb duration at this new interpolated ISO
            bulb_duration_s = (current_aperture**2 * 100) / (2**target_ev * current_iso)
            
            logging.info(f"ISO Transition frame {self.iso_transition_frame_count}/{self.iso_transition_duration_frames}: ISO {current_iso:.0f}, Bulb {bulb_duration_s:.2f}s")
            
            exposure_settings = self._package_settings('bulb', current_iso, 0, bulb_duration_s, current_aperture, target_kelvin)
            exposure_duration = bulb_duration_s

            if progress >= 1.0:
                self.is_in_iso_transition = False
                logging.info("ISO transition to Night Native ISO complete.")
        
        # --- STATE 2: Normal operation (not yet transitioning) ---
        else:
            current_iso = day_iso # Start with base day ISO
            while True:
                shutter_s = (current_aperture**2 * 100) / (2**target_ev * current_iso)

                if shutter_s > 30.0:
                    # Shutter is too long. Can we increase ISO?
                    if current_iso < max_transition_iso:
                        next_iso_index = np.searchsorted(self.STANDARD_ISOS, current_iso, side='right')
                        if next_iso_index < len(self.STANDARD_ISOS):
                            next_iso = self.STANDARD_ISOS[next_iso_index]
                            if next_iso <= max_transition_iso:
                                current_iso = next_iso
                                continue # Rerun loop with higher ISO
                    
                    # If we can't/won't increase ISO, it's time to START the transition
                    logging.info(f"Max transition ISO ({max_transition_iso}) reached. Starting smooth ISO ramp-down to bulb mode.")
                    self.is_in_iso_transition = True
                    self.iso_transition_frame_count = 0
                    
                    # For this first frame, use bulb mode at the CURRENT high ISO
                    bulb_duration_s = shutter_s
                    exposure_settings = self._package_settings('bulb', current_iso, 0, bulb_duration_s, current_aperture, target_kelvin)
                    exposure_duration = bulb_duration_s
                    break

                elif shutter_s <= 1/8000: # Shutter is too fast
                    exposure_settings = self._package_settings('normal', current_iso, 1/8000, 0, current_aperture, target_kelvin)
                    exposure_duration = 1/8000
                    break
                
                else: # Shutter is in normal range
                    exposure_settings = self._package_settings('normal', current_iso, shutter_s, 0, current_aperture, target_kelvin)
                    exposure_duration = shutter_s
                    break

        required_interval = exposure_duration + self.MIN_INTERVAL_SAFETY_BUFFER_S
        new_target_interval = max(blended_interval, required_interval)
        
        final_interval = (self.last_used_interval * (1 - self.interval_smoothing_factor)) + (new_target_interval * self.interval_smoothing_factor)
        self.last_used_interval = final_interval
        exposure_settings['target_interval'] = final_interval
        
        return exposure_settings
    # --- END OF MODIFICATION ---

    def update_exposure_offset(self, image_path):
        measured_brightness = self._get_image_brightness(image_path)
        if measured_brightness is None: return
        if abs(measured_brightness - self.target_brightness) < self.deadband_threshold: return
        smoothing_factor = self._calculate_dynamic_smoothing_factor()
        immediate_ev_error = math.log2(self.target_brightness / measured_brightness)
        self.reactive_ev_offset += immediate_ev_error * smoothing_factor
        self.reactive_ev_offset = np.clip(self.reactive_ev_offset, -1.5, 1.5)
        logging.debug(f"ReactiveUpdate: Measured={measured_brightness:.1f}, ErrorEV={immediate_ev_error:+.2f}, NewReactiveOffset={self.reactive_ev_offset:+.2f}")

    def _calculate_dynamic_smoothing_factor(self):
        base_factor = 0.02; sun_points = [-18, -12, -6, 0, 6, 12, 18]; sun_mults  = [1.0, 1.5, 2.5, 3.0, 2.5, 1.5, 1.0]
        sun_angle_multiplier = np.interp(self.last_sun_elevation, sun_points, sun_mults)
        return base_factor * sun_angle_multiplier

    def _get_image_brightness(self, image_path: str) -> float | None:
        try:
            with Image.open(image_path) as img:
                width, height = img.size; crop_percent = 0.60
                left, top = (width - width * crop_percent) / 2, (height - height * crop_percent) / 2
                right, bottom = (width + width * crop_percent) / 2, (height + height * crop_percent) / 2
                center_cropped_img = img.crop((left, top, right, bottom))
                grayscale_img = center_cropped_img.convert('L')
                stats = ImageStat.Stat(grayscale_img)
                return max(1.0, stats.mean[0])
        except Exception as e:
            logging.error(f"Failed to analyze image brightness for '{image_path}': {e}"); return None

    def _find_closest(self, value, candidates, return_values=None):
        candidates = np.asarray(candidates)
        idx = (np.abs(candidates - value)).argmin()
        return return_values[idx] if return_values is not None else candidates[idx]

    def _package_settings(self, mode, iso, shutter_s, bulb_s, aperture_f, kelvin):
        final_iso = self._find_closest(iso, self.STANDARD_ISOS)
        final_aperture = self._find_closest(aperture_f, self.STANDARD_APERTURES_F)
        shutter_str = 'bulb' if mode == 'bulb' else self._find_closest(shutter_s, self.SHUTTER_VALUES, self.SHUTTER_STRINGS)
        self.last_exposure_duration = bulb_s if mode == 'bulb' else shutter_s
        return {'mode': mode, 'iso': final_iso, 'shutter': shutter_str, 'bulb_duration': bulb_s, 'aperture': f"{final_aperture:.1f}", 'kelvin': kelvin, 'ev_offset': self.reactive_ev_offset, 'target_interval': 0}
