import tkinter
from tkinter import ttk, messagebox, font
from PIL import Image, ImageTk
import os
import logging
from holygrail import HolyGrailController

class PiSliderGUI:
    def __init__(self, master_root, app_controller):
        self.root = master_root
        self.app = app_controller
        self.bold_font = ('Arial', 12, 'bold')

        self._create_main_layout()
        self._create_main_settings_widgets(self.main_settings_frame)
        self._create_holygrail_widgets(self.holygrail_frame)
        self._create_control_panel_widgets(self.control_panel_frame)
        self._create_preview_widgets(self.preview_frame)
        self._create_live_settings_widgets(self.live_settings_frame)

        self.update_mode_widgets()
        self.update_distribution_image()
        self.toggle_holygrail_widgets()
        self.show_live_settings(False)

    def _create_main_layout(self):
        self.root.columnconfigure(0, weight=2)
        self.root.columnconfigure(1, minsize=260)
        self.root.columnconfigure(2, weight=3)
        self.root.rowconfigure(0, weight=1)

        settings_container = ttk.Frame(self.root, padding=5)
        settings_container.grid(row=0, column=0, sticky="nsew")

        self.notebook = ttk.Notebook(settings_container)
        self.notebook.pack(fill=tkinter.BOTH, expand=True)
        self.main_settings_frame = ttk.Frame(self.notebook, padding="10")
        self.holygrail_frame = ttk.Frame(self.notebook, padding="10")
        self.notebook.add(self.main_settings_frame, text='Main Settings')
        self.notebook.add(self.holygrail_frame, text='Holy Grail')

        self.control_panel_frame = ttk.Frame(self.root, padding=10)
        self.control_panel_frame.grid(row=0, column=1, sticky="ns")

        self.preview_frame = ttk.Frame(self.root, padding=5)
        self.preview_frame.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)

        self.live_settings_frame = ttk.LabelFrame(self.control_panel_frame, text="Live Settings", padding="10")

    def _create_preview_widgets(self, parent_frame):
        parent_frame.columnconfigure(0, weight=1)
        parent_frame.rowconfigure(1, weight=1)
        ttk.Label(parent_frame, text="Live Preview", font=self.bold_font).grid(row=0, column=0, pady=5)
        self.preview_image_label = ttk.Label(parent_frame, text="Preview will appear here", anchor="center", relief="solid")
        self.preview_image_label.grid(row=1, column=0, sticky="nsew", pady=5)
        self.take_preview_button = ttk.Button(parent_frame, text="Take Preview Photo", command=self.app.take_preview)
        self.take_preview_button.grid(row=2, column=0, pady=10, sticky="ew")

    def _create_main_settings_widgets(self, parent_frame):
        parent_frame.columnconfigure(1, weight=1); current_row = 0
        
        save_frame = ttk.Frame(parent_frame)
        save_frame.grid(row=current_row, column=0, columnspan=2, sticky='ew', pady=(0, 10))
        save_frame.columnconfigure(1, weight=1)
        ttk.Label(save_frame, text="Save Location:").grid(row=0, column=0, sticky='w')
        self.save_path_combo = ttk.Combobox(save_frame, textvariable=self.app.save_path_var, state="readonly")
        self.save_path_combo.grid(row=0, column=1, sticky='ew', padx=5)
        self.refresh_drives_button = ttk.Button(save_frame, text="Refresh", command=self.app.detect_storage_paths, width=8)
        self.refresh_drives_button.grid(row=0, column=2, sticky='e')
        current_row += 1

        ttk.Label(parent_frame, text="Slider Settings", font=self.bold_font).grid(row=current_row, column=0, columnspan=2, pady=5, sticky=tkinter.W); current_row += 1
        ttk.Label(parent_frame, text="Direction:").grid(row=current_row, column=0, sticky=tkinter.W); self.direction_combo = ttk.Combobox(parent_frame, textvariable=self.app.direction_var, values=['right', 'left'], width=18, state="readonly"); self.direction_combo.grid(row=current_row, column=1, sticky=tkinter.E); current_row += 1
        ttk.Label(parent_frame, text="Length (mm):").grid(row=current_row, column=0, sticky=tkinter.W); self.length_entry = ttk.Entry(parent_frame, textvariable=self.app.length_var, width=20); self.length_entry.grid(row=current_row, column=1, sticky=tkinter.E); current_row += 1
        ttk.Label(parent_frame, text="Min Slider Speed (mm/photo):").grid(row=current_row, column=0, sticky=tkinter.W); self.minimum_slider_speed_entry = ttk.Entry(parent_frame, textvariable=self.app.minimum_slider_speed_var, width=20); self.minimum_slider_speed_entry.grid(row=current_row, column=1, sticky=tkinter.E); current_row += 1
        ttk.Label(parent_frame, text="Slider Distribution:").grid(row=current_row, column=0, sticky=tkinter.W); self.distribution_combo = ttk.Combobox(parent_frame, textvariable=self.app.distribution_var, values=list(self.app.distribution_images.keys()), width=18, state="readonly"); self.distribution_combo.grid(row=current_row, column=1, sticky=tkinter.E); self.distribution_combo.bind("<<ComboboxSelected>>", self.update_distribution_image); current_row += 1
        ttk.Label(parent_frame, text="Rotation Settings", font=self.bold_font).grid(row=current_row, column=0, columnspan=2, pady=(10,5), sticky=tkinter.W); current_row += 1
        ttk.Label(parent_frame, text="Rotation Angle (°):").grid(row=current_row, column=0, sticky=tkinter.W); self.rotation_angle_entry = ttk.Entry(parent_frame, textvariable=self.app.rotation_angle_var, width=20); self.rotation_angle_entry.grid(row=current_row, column=1, sticky=tkinter.E); current_row += 1
        ttk.Label(parent_frame, text="Rotation Direction:").grid(row=current_row, column=0, sticky=tkinter.W); self.rotation_direction_combo = ttk.Combobox(parent_frame, textvariable=self.app.rotation_direction_var, values=["CW", "CCW"], width=18, state="readonly"); self.rotation_direction_combo.grid(row=current_row, column=1, sticky=tkinter.E); self.rotation_direction_combo.bind("<<ComboboxSelected>>", self.app.update_rotation_direction); current_row += 1
        ttk.Label(parent_frame, text="Rotation Distribution:").grid(row=current_row, column=0, sticky=tkinter.W); self.rotation_distribution_combo = ttk.Combobox(parent_frame, textvariable=self.app.rotation_distribution_var, values=list(self.app.distribution_functions.keys()), width=18, state="readonly"); self.rotation_distribution_combo.grid(row=current_row, column=1, sticky=tkinter.E); current_row += 1
        ttk.Label(parent_frame, text="General Settings", font=self.bold_font).grid(row=current_row, column=0, columnspan=2, pady=(10,5), sticky=tkinter.W); current_row += 1
        ttk.Label(parent_frame, text="Camera Trigger:").grid(row=current_row, column=0, sticky=tkinter.W); camera_modes = ["S2 Cable"];
        if self.app.camera and self.app.camera.gphoto2_available: camera_modes.append("USB Control")
        self.camera_trigger_combo = ttk.Combobox(parent_frame, textvariable=self.app.camera_trigger_mode_var, values=camera_modes, width=18, state="readonly"); self.camera_trigger_combo.bind("<<ComboboxSelected>>", self.app.on_camera_mode_change); self.camera_trigger_combo.grid(row=current_row, column=1, sticky=tkinter.E); current_row += 1
        self.camera_status_label = ttk.Label(parent_frame, textvariable=self.app.camera_status_var); self.camera_status_label.grid(row=current_row, column=0, columnspan=2, sticky=tkinter.W); current_row += 1
        self.check_camera_button = ttk.Button(parent_frame, text="Check USB Connection", command=self.app.check_camera_connection); self.check_camera_button.grid(row=current_row, column=0, columnspan=2, sticky='ew', pady=(0, 5)); current_row += 1
        ttk.Label(parent_frame, text="Number of Photos:").grid(row=current_row, column=0, sticky=tkinter.W); self.num_photos_entry = ttk.Entry(parent_frame, textvariable=self.app.num_photos_var, width=20); self.num_photos_entry.grid(row=current_row, column=1, sticky=tkinter.E); current_row +=1
        self.main_settings_current_row = current_row
        self.interval_label = ttk.Label(parent_frame, text="Interval (s):"); self.interval_entry = ttk.Entry(parent_frame, textvariable=self.app.interval_var, width=20); self.interval_entry.grid(row=current_row, column=1, sticky=tkinter.E)
        
        main_entries = [self.length_entry, self.minimum_slider_speed_entry, self.rotation_angle_entry, self.num_photos_entry, self.interval_entry]
        for entry in main_entries: entry.bind("<FocusIn>", self.app.on_entry_focus); entry.bind("<FocusOut>", self.app.on_entry_defocus)
        self.app.entry_widgets.extend(main_entries)
        for child in parent_frame.winfo_children(): child.grid_configure(padx=5, pady=2)

    def _create_holygrail_widgets(self, parent_frame):
        parent_frame.columnconfigure(1, weight=1); current_row = 0
        top_frame = ttk.Frame(parent_frame); top_frame.grid(row=current_row, column=0, columnspan=4, sticky='w', pady=(0, 10))
        ttk.Checkbutton(top_frame, text="Enable Holy Grail Mode", variable=self.app.holygrail_enabled_var, command=self.app.toggle_holygrail_mode).pack(side=tkinter.LEFT)
        help_button = ttk.Button(top_frame, text="?", command=self.show_holygrail_help, width=3); help_button.pack(side=tkinter.LEFT, padx=10); current_row += 1
        
        self.loc_frame = ttk.LabelFrame(parent_frame, text="HG Settings", padding=5)
        self.loc_frame.grid(row=current_row, column=0, columnspan=4, sticky='ew', pady=(0, 10))
        self.loc_frame.columnconfigure(1, weight=1)
        
        loc_row = 0
        ttk.Label(self.loc_frame, text="Latitude:").grid(row=loc_row, column=0, sticky='w', padx=5, pady=2); lat_entry = ttk.Entry(self.loc_frame, textvariable=self.app.holygrail_latitude_var); lat_entry.grid(row=loc_row, column=1, sticky='ew', padx=5); loc_row += 1
        ttk.Label(self.loc_frame, text="Longitude:").grid(row=loc_row, column=0, sticky='w', padx=5, pady=2); lon_entry = ttk.Entry(self.loc_frame, textvariable=self.app.holygrail_longitude_var); lon_entry.grid(row=loc_row, column=1, sticky='ew', padx=5); loc_row += 1
        ttk.Label(self.loc_frame, text="Min Aperture (f/):").grid(row=loc_row, column=0, sticky='w', padx=5, pady=2); min_ap_entry = ttk.Entry(self.loc_frame, textvariable=self.app.holygrail_min_aperture_var); min_ap_entry.grid(row=loc_row, column=1, sticky='ew', padx=5); loc_row += 1
        ttk.Label(self.loc_frame, text="Max Aperture (f/):").grid(row=loc_row, column=0, sticky='w', padx=5, pady=2); max_ap_entry = ttk.Entry(self.loc_frame, textvariable=self.app.holygrail_max_aperture_var); max_ap_entry.grid(row=loc_row, column=1, sticky='ew', padx=5); loc_row += 1
        
        ttk.Label(self.loc_frame, text="Day ISO:").grid(row=loc_row, column=0, sticky='w', padx=5, pady=2); day_iso_entry = ttk.Entry(self.loc_frame, textvariable=self.app.holygrail_day_iso_var); day_iso_entry.grid(row=loc_row, column=1, sticky='ew', padx=5); loc_row += 1
        ttk.Label(self.loc_frame, text="Night Native ISO:").grid(row=loc_row, column=0, sticky='w', padx=5, pady=2); night_iso_entry = ttk.Entry(self.loc_frame, textvariable=self.app.holygrail_night_iso_var); night_iso_entry.grid(row=loc_row, column=1, sticky='ew', padx=5); loc_row += 1
        ttk.Label(self.loc_frame, text="Max Transition ISO:").grid(row=loc_row, column=0, sticky='w', padx=5, pady=2); max_trans_iso_entry = ttk.Entry(self.loc_frame, textvariable=self.app.holygrail_max_transition_iso_var); max_trans_iso_entry.grid(row=loc_row, column=1, sticky='ew', padx=5); loc_row += 1
        ttk.Label(self.loc_frame, text="ISO Transition Frames:").grid(row=loc_row, column=0, sticky='w', padx=5, pady=2); iso_trans_entry = ttk.Entry(self.loc_frame, textvariable=self.app.holygrail_iso_transition_frames_var); iso_trans_entry.grid(row=loc_row, column=1, sticky='ew', padx=5); loc_row += 1

        self.day_interval_label = ttk.Label(self.loc_frame, text="Day Interval (s):"); self.day_interval_entry = ttk.Entry(self.loc_frame, textvariable=self.app.holygrail_day_interval_var)
        self.sunset_interval_label = ttk.Label(self.loc_frame, text="Sunset Interval (s):"); self.sunset_interval_entry = ttk.Entry(self.loc_frame, textvariable=self.app.holygrail_sunset_interval_var)
        self.night_interval_label = ttk.Label(self.loc_frame, text="Night Interval (s):"); self.night_interval_entry = ttk.Entry(self.loc_frame, textvariable=self.app.holygrail_night_interval_var)
        current_row += 1

        cal_frame = ttk.LabelFrame(parent_frame, text="Exposure Calibration", padding=5); cal_frame.grid(row=current_row, column=0, columnspan=4, sticky='ew', pady=(0, 10)); cal_frame.columnconfigure(0, weight=1); cal_frame.columnconfigure(1, weight=1)
        self.calibrate_button = ttk.Button(cal_frame, text="Calibrate HG Exposure", command=self.app.calibrate_holygrail_exposure); self.calibrate_button.grid(row=0, column=0, padx=5, pady=5, sticky='ew')
        ttk.Label(cal_frame, textvariable=self.app.calibration_offset_var, anchor='center').grid(row=0, column=1, padx=5, pady=5, sticky='ew'); current_row += 1
        
        table_frame = ttk.LabelFrame(parent_frame, text="Keyframe Ramping Settings", padding=5); table_frame.grid(row=current_row, column=0, columnspan=4, sticky='ew'); table_frame.columnconfigure(1, weight=1); table_frame.columnconfigure(2, weight=1)
        ttk.Label(table_frame, text="Sun Angle (°)", font=self.bold_font).grid(row=0, column=0, padx=5); ttk.Label(table_frame, text="Target EV", font=self.bold_font).grid(row=0, column=1, padx=5); ttk.Label(table_frame, text="Kelvin", font=self.bold_font).grid(row=0, column=2, padx=5)
        table_row = 1
        for angle in sorted(HolyGrailController.DEFAULT_EV_TABLE.keys()):
            default_ev, default_kelvin = HolyGrailController.DEFAULT_EV_TABLE[angle]
            ttk.Label(table_frame, text=f"{angle}°").grid(row=table_row, column=0, sticky='w', padx=5)
            ev_var = tkinter.DoubleVar(value=default_ev); ev_entry = ttk.Entry(table_frame, textvariable=ev_var, width=8); ev_entry.grid(row=table_row, column=1, padx=2)
            kelvin_var = tkinter.IntVar(value=default_kelvin); kelvin_entry = ttk.Entry(table_frame, textvariable=kelvin_var, width=8); kelvin_entry.grid(row=table_row, column=2, padx=2)
            self.app.holygrail_entries[angle] = {'ev_var': ev_var, 'kelvin_var': kelvin_var}
            self.app.entry_widgets.extend([ev_entry, kelvin_entry])
            for entry in [ev_entry, kelvin_entry]: entry.bind("<FocusIn>", self.app.on_entry_focus); entry.bind("<FocusOut>", self.app.on_entry_defocus)
            table_row += 1
            
        hg_entries = [lat_entry, lon_entry, min_ap_entry, max_ap_entry, day_iso_entry, night_iso_entry, max_trans_iso_entry, iso_trans_entry, self.day_interval_entry, self.sunset_interval_entry, self.night_interval_entry]
        self.app.entry_widgets.extend(hg_entries)
        for entry in hg_entries: entry.bind("<FocusIn>", self.app.on_entry_focus); entry.bind("<FocusOut>", self.app.on_entry_defocus)

    def _create_control_panel_widgets(self, parent_frame):
        parent_frame.columnconfigure(0, weight=1); current_row = 0
        ttk.Label(parent_frame, text="Control Panel", font=self.bold_font).grid(row=current_row, column=0, pady=5); current_row += 1
        self.start_button = ttk.Button(parent_frame, text="Start Timelapse", command=self.app.start_timelapse); self.start_button.grid(row=current_row, column=0, pady=5, sticky="ew"); current_row += 1
        self.preview_button = ttk.Button(parent_frame, text="Run Preview", command=self.app.start_preview); self.preview_button.grid(row=current_row, column=0, pady=5, sticky="ew"); current_row += 1
        self.stop_button = ttk.Button(parent_frame, text="Stop", command=self.app.request_stop_operations, state=tkinter.DISABLED); self.stop_button.grid(row=current_row, column=0, pady=5, sticky="ew"); current_row += 1
        self.live_settings_frame.grid(row=current_row, column=0, sticky='ew', pady=5); current_row += 1
        mode_frame = ttk.LabelFrame(parent_frame, text="Mode Selection", padding=5); mode_frame.grid(row=current_row, column=0, sticky='ew', pady=5); current_row += 1
        self.timed_radio = ttk.Radiobutton(mode_frame, text="Timed", variable=self.app.mode_var, value="timed", command=self.app.update_mode); self.timed_radio.pack(side=tkinter.LEFT, padx=5)
        self.manual_radio = ttk.Radiobutton(mode_frame, text="Manual Trigger", variable=self.app.mode_var, value="manual", command=self.app.update_mode); self.manual_radio.pack(side=tkinter.LEFT, padx=5)
        post_frame = ttk.LabelFrame(parent_frame, text="Post-Timelapse Action", padding=5); post_frame.grid(row=current_row, column=0, sticky='ew', pady=5); current_row += 1
        self.post_timelapse_combo = ttk.Combobox(post_frame, textvariable=self.app.post_timelapse_position_var, values=["Do Nothing", "Return to Start", "Move to End"], state="readonly"); self.post_timelapse_combo.pack(fill=tkinter.X, expand=True)
        self.distribution_image_label = ttk.Label(parent_frame, anchor="center"); self.distribution_image_label.grid(row=current_row, column=0, pady=10, sticky="ew")

    def _create_live_settings_widgets(self, parent_frame):
        parent_frame.columnconfigure(1, weight=1); row = 0
        ttk.Label(parent_frame, text="ISO:").grid(row=row, column=0, sticky='w'); ttk.Label(parent_frame, textvariable=self.app.live_iso_var).grid(row=row, column=1, sticky='w'); row += 1
        ttk.Label(parent_frame, text="Shutter:").grid(row=row, column=0, sticky='w'); ttk.Label(parent_frame, textvariable=self.app.live_shutter_var).grid(row=row, column=1, sticky='w'); row += 1
        ttk.Label(parent_frame, text="Aperture:").grid(row=row, column=0, sticky='w'); ttk.Label(parent_frame, textvariable=self.app.live_aperture_var).grid(row=row, column=1, sticky='w'); row += 1
        ttk.Label(parent_frame, text="Kelvin:").grid(row=row, column=0, sticky='w'); ttk.Label(parent_frame, textvariable=self.app.live_kelvin_var).grid(row=row, column=1, sticky='w'); row += 1
        ttk.Label(parent_frame, text="EV Offset:").grid(row=row, column=0, sticky='w'); ttk.Label(parent_frame, textvariable=self.app.live_ev_offset_var).grid(row=row, column=1, sticky='w'); row += 1
        ttk.Label(parent_frame, text="Interval:", font=self.bold_font).grid(row=row, column=0, sticky='w', pady=(5,0)); ttk.Label(parent_frame, textvariable=self.app.live_interval_var, font=self.bold_font).grid(row=row, column=1, sticky='w', pady=(5,0)); row += 1
        separator = ttk.Separator(parent_frame, orient='horizontal'); separator.grid(row=row, column=0, columnspan=2, sticky='ew', pady=5); row += 1
        ttk.Label(parent_frame, text="Slider Steps:").grid(row=row, column=0, sticky='w'); ttk.Label(parent_frame, textvariable=self.app.live_slider_steps_var).grid(row=row, column=1, sticky='w'); row += 1
        ttk.Label(parent_frame, text="Rotation Steps:").grid(row=row, column=0, sticky='w'); ttk.Label(parent_frame, textvariable=self.app.live_rotation_steps_var).grid(row=row, column=1, sticky='w'); row += 1
        ttk.Label(parent_frame, textvariable=self.app.photo_count_var).grid(row=row, column=0, columnspan=2, pady=(5,0)); row += 1
        ttk.Label(parent_frame, textvariable=self.app.time_remaining_var).grid(row=row, column=0, columnspan=2); row += 1

    def show_live_settings(self, show: bool):
        if show: self.live_settings_frame.grid()
        else: self.live_settings_frame.grid_remove()

    def update_ui_for_run_state(self, is_running):
        state = tkinter.DISABLED if is_running else tkinter.NORMAL
        self.start_button.config(state=state); self.preview_button.config(state=state); self.take_preview_button.config(state=state); self.check_camera_button.config(state=state); self.refresh_drives_button.config(state=state)
        if is_running or not (self.app.holygrail_enabled_var.get() and self.app.camera.gphoto2_available): self.calibrate_button.config(state=tkinter.DISABLED)
        else: self.calibrate_button.config(state=tkinter.NORMAL)
        self.stop_button.config(state=tkinter.NORMAL if is_running else tkinter.DISABLED)
        for widget in self.app.entry_widgets:
            if widget.winfo_exists(): widget.config(state=state)
        for combo in [self.direction_combo, self.distribution_combo, self.rotation_direction_combo, self.rotation_distribution_combo, self.camera_trigger_combo, self.post_timelapse_combo, self.save_path_combo]:
            if combo.winfo_exists(): combo.config(state="readonly" if not is_running else tkinter.DISABLED)

    def update_mode_widgets(self):
        is_manual = self.app.mode_var.get() == "manual"; is_hg = self.app.holygrail_enabled_var.get()
        row = self.main_settings_current_row
        if not is_manual and not is_hg:
            self.interval_label.grid(row=row, column=0, sticky=tkinter.W, padx=5, pady=2); self.interval_entry.grid(row=row, column=1, sticky=tkinter.E, padx=5, pady=2)
        else:
            self.interval_label.grid_remove(); self.interval_entry.grid_remove()

    def toggle_holygrail_widgets(self):
        is_hg = self.app.holygrail_enabled_var.get()
        state = tkinter.NORMAL if is_hg and self.app.camera.gphoto2_available else tkinter.DISABLED
        self.calibrate_button.config(state=state)
        row_num = 8  # Start row for interval entries after new ISO and transition fields
        if is_hg:
            self.day_interval_label.grid(row=row_num, column=0, sticky='w', padx=5, pady=2); self.day_interval_entry.grid(row=row_num, column=1, sticky='ew', padx=5); row_num+=1
            self.sunset_interval_label.grid(row=row_num, column=0, sticky='w', padx=5, pady=2); self.sunset_interval_entry.grid(row=row_num, column=1, sticky='ew', padx=5); row_num+=1
            self.night_interval_label.grid(row=row_num, column=0, sticky='w', padx=5, pady=2); self.night_interval_entry.grid(row=row_num, column=1, sticky='ew', padx=5)
        else:
            self.day_interval_label.grid_remove(); self.day_interval_entry.grid_remove()
            self.sunset_interval_label.grid_remove(); self.sunset_interval_entry.grid_remove()
            self.night_interval_label.grid_remove(); self.night_interval_entry.grid_remove()
        self.update_mode_widgets()

    def update_preview_image(self, image_path):
        if not os.path.exists(image_path) or os.path.getsize(image_path) == 0:
            logging.error(f"Invalid preview image at path: {image_path}")
            self.preview_image_label.config(image='', text="Preview Failed"); return
        try:
            img = Image.open(image_path)
            img_w, img_h = img.size
            container_w = self.preview_frame.winfo_width()
            
            if container_w < 100: target_w = 800; target_h = int(target_w * (img_h / img_w))
            else: target_w = self.preview_frame.winfo_width(); target_h = self.preview_frame.winfo_height()
            
            img.thumbnail((target_w - 20, target_h - 20), Image.Resampling.LANCZOS)
            self.app.preview_photo_image = ImageTk.PhotoImage(img)
            self.preview_image_label.config(image=self.app.preview_photo_image, text="")
        except Exception as e:
            logging.error(f"Error updating preview image: {e}", exc_info=True)
            self.preview_image_label.config(image='', text="Preview Error")

    def update_distribution_image(self, event=None):
        key = self.app.distribution_var.get(); image_name = self.app.distribution_images.get(key)
        if image_name and isinstance(image_name, str):
            try:
                script_dir = os.path.dirname(os.path.realpath(__file__))
                image_path = os.path.join(script_dir, "images", image_name)
                if not os.path.exists(image_path): image_path = os.path.join(script_dir, image_name)
                img = Image.open(image_path)
                img.thumbnail((250, 100), Image.Resampling.LANCZOS)
                self.app.distribution_photo_image = ImageTk.PhotoImage(img)
                self.distribution_image_label.config(image=self.app.distribution_photo_image)
            except Exception as e:
                logging.error(f"Error loading distribution image {image_path}: {e}")
                self.distribution_image_label.config(image='', text="Img not found")

    def show_holygrail_help(self):
        help_text = "For Holy Grail mode to work, please set your camera to:\n\n" \
                    "1. Mode Dial: Manual (M)\n\n" \
                    "2. File Format: RAW Only\n" \
                    "   (The system will generate previews automatically)\n\n" \
                    "3. USB Connection: PC Remote / Tethering\n\n" \
                    "4. Turn OFF Long Exposure Noise Reduction (LENR)\n" \
                    "   (This prevents the camera from taking dark frames)"
                    
        messagebox.showinfo("Holy Grail Setup Help", help_text, parent=self.root)
