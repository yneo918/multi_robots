import time
import json
import board
import busio
import adafruit_bno055

# Create I2C connection and sensor object
i2c = busio.I2C(board.SCL, board.SDA)
bno = adafruit_bno055.BNO055_I2C(i2c)

OFFSETS_FILE = "bno055_offsets.json"

def load_offsets():
    """Load saved offsets from a JSON file, if it exists."""
    try:
        with open(OFFSETS_FILE, "r") as f:
            data = json.load(f)
        # Apply accelerometer offsets and radius
        bno.offsets_accelerometer = tuple(data["offsets_accelerometer"])
        bno.radius_accelerometer = data["radius_accelerometer"]
        # Apply magnetometer offsets and radius
        bno.offsets_magnetometer  = tuple(data["offsets_magnetometer"])
        bno.radius_magnetometer  = data["radius_magnetometer"]
        # Apply gyroscope offsets
        bno.offsets_gyroscope    = tuple(data["offsets_gyroscope"])
        print("Loaded offsets from file.")
        return True
    except (OSError, KeyError, ValueError):
        return False

def save_offsets():
    """Read current offsets from sensor and save to a JSON file."""
    data = {
        "offsets_accelerometer": list(bno.offsets_accelerometer),
        "radius_accelerometer":    bno.radius_accelerometer,
        "offsets_magnetometer":  list(bno.offsets_magnetometer),
        "radius_magnetometer":     bno.radius_magnetometer,
        "offsets_gyroscope":    list(bno.offsets_gyroscope),
    }
    with open(OFFSETS_FILE, "w") as f:
        json.dump(data, f)
    print("Saved offsets to file:", data)

def calibrate():
    """
    Loop until all sensors are fully calibrated (status==3).
    Status tuple: (sys, gyro, accel, mag)
    """
    print("Starting calibration. Move sensor until all values reach 3.")
    while True:
        sys, gyro, accel, mag = bno.calibration_status
        print(f"Sys={sys} Gyro={gyro} Accel={accel} Mag={mag}")
        if sys == 3 and gyro == 3 and accel == 3 and mag == 3:
            print("Calibration complete!")
            return
        time.sleep(1)

def main():
    # Try to load existing offsets; if none, perform manual calibration
    if not load_offsets():
        calibrate()
        save_offsets()

    # Now the sensor is calibrated—read and print Euler angles
    print("Reading orientation...")
    while True:
        heading, roll, pitch = bno.euler
        print(f"Heading={heading:.1f} Roll={roll:.1f} Pitch={pitch:.1f}")
        time.sleep(0.5)

if __name__ == "__main__":
    main()
