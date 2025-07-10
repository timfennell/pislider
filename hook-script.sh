#!/bin/bash
# This script is called by gphoto2 after a file is downloaded during tethered capture.
# Its purpose is to efficiently extract the embedded preview JPEG from the RAW file
# without needing to run a slow, separate Python process for the task.

# The destination path for the preview is passed from PiSlider.py via an environment variable.
PREVIEW_PATH="$PREVIEW_PATH"

# gphoto2 provides the ACTION and ARGUMENT variables.
# We only care about the "download" action, which occurs after the file is on the Pi.
if [ "$ACTION" = "download" ]; then
    
    # Log that the script has been triggered for debugging purposes.
    echo "Hook script triggered for file: $ARGUMENT"
    
    # Check if the PREVIEW_PATH variable was actually set.
    if [ -z "$PREVIEW_PATH" ]; then
        echo "Error: PREVIEW_PATH environment variable not set. Cannot extract preview."
        exit 1
    fi

    # Use dcraw, a fast and efficient tool for handling RAW files.
    # -e : Extract the camera's embedded thumbnail (this is much faster than processing the full RAW).
    # -c : Write the output to standard out instead of creating a file with a .thumb extension.
    # > "$PREVIEW_PATH" : Redirect the standard output (the JPEG data) into our desired preview file.
    # 2>/dev/null : Redirect standard error to null to suppress non-critical warnings from dcraw.
    dcraw -e -c "$ARGUMENT" > "$PREVIEW_PATH" 2>/dev/null
    
    # Final check to see if the preview file was successfully created.
    if [ -f "$PREVIEW_PATH" ] && [ -s "$PREVIEW_PATH" ]; then
        # -f checks if the file exists, -s checks if it has a size greater than zero.
        echo "Successfully extracted preview to $PREVIEW_PATH"
    else
        echo "Error: Failed to extract or create preview file at $PREVIEW_PATH"
        exit 1
    fi
fi

# Exit with a success code
exit 0
