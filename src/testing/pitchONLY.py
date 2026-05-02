from machine import I2C, Pin, PWM
import time
import math

MPU_ADDR = 0x68

# ---------------- I2C setup ----------------
i2c = I2C(0, sda=Pin("A4"), scl=Pin("A5"), freq=100000)
print("I2C scan:", [hex(x) for x in i2c.scan()])

# ---------------- Global state ----------------
RatePitch = 0.0

AccX = 0.0
AccY = 0.0
AccZ = 0.0

acc_pitch = 0.0
AnglePitch = 0.0

# Gyro zero offset
gyro_bias_y = 0.0

# Complementary filter weight
# Angle = alpha * gyro + (1 - alpha) * accel
alpha = 0.4

# ---------------- Servo settings ----------------
pitch_center = 130
pitch_sign = -1

pitch_min = 40
pitch_max = 180

angle_deadband = 3.0
servo_smoothing = 0.2   # lower = smoother/slower, higher = faster/sharper

# ---------------- Control law settings ----------------
# 0 to 15 deg   -> normal proportional response
# 15 to 30 deg  -> smooth exponential ramp
# 30+ deg       -> full servo actuation
boost_threshold = 15.0
full_actuation_tilt = 30.0
boost_exp_gain = 0.5

# Base proportional gain for normal region
kp_pitch = 1.5

# 50 Hz servo pulse range mapped to 16-bit PWM duty
min_duty = 1638
max_duty = 8192

servo_angle_pitch = pitch_center

# ---------------- Helper functions ----------------
def write_reg(reg, val):
    i2c.writeto_mem(MPU_ADDR, reg, bytes([val]))

def read_bytes(reg, n):
    return i2c.readfrom_mem(MPU_ADDR, reg, n)

def to_int16(msb, lsb):
    v = (msb << 8) | lsb
    if v & 0x8000:
        v -= 65536
    return v

def apply_deadband(x, deadband):
    if abs(x) < deadband:
        return 0.0
    return x

def low_pass(prev, new, smoothing):
    return prev + smoothing * (new - prev)

def angle_to_duty(angle):
    angle = max(pitch_min, min(pitch_max, angle))
    return int(min_duty + (angle / 180.0) * (max_duty - min_duty))

# ---------------- MPU6050 init ----------------
def mpu_init():
    # Wake up
    write_reg(0x6B, 0x00)
    time.sleep_ms(100)

    # DLPF config
    write_reg(0x1A, 0x03)

    # Accelerometer range ±8g
    write_reg(0x1C, 0x10)

    # Gyro range ±500 deg/s
    write_reg(0x1B, 0x08)

    time.sleep_ms(100)

# ---------------- Gyro calibration ----------------
def calibrate_gyro(samples=500):
    global gyro_bias_y

    sum_y = 0
    print("Calibrating gyro... keep system still")

    for _ in range(samples):
        data = read_bytes(0x43, 6)
        gy = to_int16(data[2], data[3])
        sum_y += gy
        time.sleep_ms(2)

    gyro_bias_y = sum_y / samples
    print("Gyro pitch bias:", gyro_bias_y)

# ---------------- Sensor fusion ----------------
def update_imu(dt):
    global RatePitch
    global AccX, AccY, AccZ
    global acc_pitch, AnglePitch

    # Read accelerometer
    acc_data = read_bytes(0x3B, 6)
    acc_x_lsb = to_int16(acc_data[0], acc_data[1])
    acc_y_lsb = to_int16(acc_data[2], acc_data[3])
    acc_z_lsb = to_int16(acc_data[4], acc_data[5])

    # Read gyro
    gyro_data = read_bytes(0x43, 6)
    gyro_y_raw = to_int16(gyro_data[2], gyro_data[3]) - gyro_bias_y

    # Convert gyro to deg/s (±500 dps => 65.5 LSB/deg/s)
    RatePitch = gyro_y_raw / 65.5

    # Convert accel to g (±8g => 4096 LSB/g)
    AccX = acc_x_lsb / 4096.0
    AccY = acc_y_lsb / 4096.0
    AccZ = acc_z_lsb / 4096.0

    # Accelerometer-only pitch angle
    acc_pitch = math.degrees(
        math.atan2(AccX, math.sqrt(AccY * AccY + AccZ * AccZ))
    ) - 2.7   # adjust this offset for your mounting

    # Gyro integration
    gyro_pitch = AnglePitch + RatePitch * dt

    # Complementary filter
    AnglePitch = alpha * gyro_pitch + (1.0 - alpha) * acc_pitch

# ---------------- Servo control ----------------
servo = PWM(Pin("D5"))
servo.freq(50)

def set_servo_angle(filtered_pitch):
    global servo_angle_pitch

    # Remove tiny movements
    filtered_pitch = apply_deadband(filtered_pitch, angle_deadband)

    # Determine sign and magnitude of tilt
    if filtered_pitch >= 0:
        sign = 1
    else:
        sign = -1

    mag = abs(filtered_pitch)

    # Determine which direction the servo will move for this tilt
    servo_direction = pitch_sign * sign

    # Available travel depends on direction
    if servo_direction > 0:
        max_control = pitch_max - pitch_center
    else:
        max_control = pitch_center - pitch_min

    if max_control < 0:
        max_control = 0

    # Control value at threshold using normal proportional law
    control_at_threshold = kp_pitch * boost_threshold
    if control_at_threshold > max_control:
        control_at_threshold = max_control

    # Region 1: normal proportional response up to 15 deg
    if mag <= boost_threshold:
        control_mag = kp_pitch * mag

    # Region 2: smooth exponential ramp from 15 deg to 30 deg
    elif mag < full_actuation_tilt:
        extra = mag - boost_threshold
        span = full_actuation_tilt - boost_threshold

        # 0 at 15 deg, 1 at 30 deg
        num = math.exp(boost_exp_gain * extra) - 1.0
        den = math.exp(boost_exp_gain * span) - 1.0

        if den == 0:
            ramp = extra / span
        else:
            ramp = num / den

        control_mag = control_at_threshold + (max_control - control_at_threshold) * ramp

    # Region 3: 30 deg and above = full available actuation
    else:
        control_mag = max_control

    # Restore sign
    control = sign * control_mag

    # Convert control output to servo target
    target_pitch = pitch_center + (pitch_sign * control)

    # Constrain target to servo limits
    target_pitch = max(pitch_min, min(pitch_max, target_pitch))

    # Smooth servo motion
    servo_angle_pitch = low_pass(servo_angle_pitch, target_pitch, servo_smoothing)

    # Output PWM
    duty = angle_to_duty(servo_angle_pitch)
    servo.duty_u16(duty)

# ---------------- Startup ----------------
mpu_init()
calibrate_gyro()

# Let IMU settle
for _ in range(20):
    update_imu(0.01)
    time.sleep_ms(10)

# Initialise filtered angle from accelerometer estimate
AnglePitch = acc_pitch

# Move servo to centre
servo_angle_pitch = pitch_center
servo.duty_u16(angle_to_duty(servo_angle_pitch))
time.sleep_ms(1000)

# ---------------- Main loop ----------------
last_us = time.ticks_us()

while True:
    now_us = time.ticks_us()
    dt = time.ticks_diff(now_us, last_us) / 1000000.0
    last_us = now_us

    # Clamp bad dt values
    if dt <= 0 or dt > 0.1:
        dt = 0.02

    update_imu(dt)
    set_servo_angle(AnglePitch)

    print(
        "acc_pitch={:.2f} filt_pitch={:.2f} srv_pitch={:.2f} dt={:.4f}".format(
            acc_pitch, AnglePitch, servo_angle_pitch, dt
        )
    )

    time.sleep_ms(10)