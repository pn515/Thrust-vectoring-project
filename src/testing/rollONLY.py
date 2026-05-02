from machine import I2C, Pin, PWM
import time
import math

MPU_ADDR = 0x68

# ---------------- I2C setup ----------------
i2c = I2C(0, sda=Pin("A4"), scl=Pin("A5"), freq=100000)
print("I2C scan:", [hex(x) for x in i2c.scan()])

# ---------------- Global state ----------------
RateRoll = 0.0

AccX = 0.0
AccY = 0.0
AccZ = 0.0

acc_roll = 0.0
AngleRoll = 0.0

# Gyro zero offset
gyro_bias_x = 0.0

# Complementary filter weight
# Angle = alpha * gyro + (1 - alpha) * accel
alpha = 0.98

# ---------------- Servo settings ----------------
roll_center = 80
roll_sign = 1

roll_min = 35
roll_max = 130

angle_deadband = 3.0
servo_smoothing = 0.8   # lower = smoother/slower, higher = faster/sharper

# ---------------- Control law settings ----------------
# 0 to 15 deg   -> normal proportional response
# 15 to 30 deg  -> smooth exponential ramp
# 30+ deg       -> full servo actuation
boost_threshold = 15.0
full_actuation_tilt = 30.0
boost_exp_gain = 0.2

# Base proportional gain for normal region
kp_roll = 0.8

# 50 Hz servo pulse range mapped to 16-bit PWM duty
min_duty = 1638
max_duty = 8192

servo_angle_roll = roll_center

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
    angle = max(roll_min, min(roll_max, angle))
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
    global gyro_bias_x

    sum_x = 0
    print("Calibrating gyro... keep system still")

    for _ in range(samples):
        data = read_bytes(0x43, 6)
        gx = to_int16(data[0], data[1])
        sum_x += gx
        time.sleep_ms(2)

    gyro_bias_x = sum_x / samples
    print("Gyro roll bias:", gyro_bias_x)

# ---------------- Sensor fusion ----------------
def update_imu(dt):
    global RateRoll
    global AccX, AccY, AccZ
    global acc_roll, AngleRoll

    # Read accelerometer
    acc_data = read_bytes(0x3B, 6)
    acc_x_lsb = to_int16(acc_data[0], acc_data[1])
    acc_y_lsb = to_int16(acc_data[2], acc_data[3])
    acc_z_lsb = to_int16(acc_data[4], acc_data[5])

    # Read gyro
    gyro_data = read_bytes(0x43, 6)
    gyro_x_raw = to_int16(gyro_data[0], gyro_data[1]) - gyro_bias_x

    # Convert gyro to deg/s (±500 dps => 65.5 LSB/deg/s)
    RateRoll = gyro_x_raw / 65.5

    # Convert accel to g (±8g => 4096 LSB/g)
    AccX = acc_x_lsb / 4096.0
    AccY = acc_y_lsb / 4096.0
    AccZ = acc_z_lsb / 4096.0

    # Accelerometer-only roll angle
    acc_roll = math.degrees(
        math.atan2(AccY, math.sqrt(AccX * AccX + AccZ * AccZ))
    ) + 3.5   # adjust this offset for your mounting

    # Gyro integration
    gyro_roll = AngleRoll + RateRoll * dt

    # Complementary filter
    AngleRoll = alpha * gyro_roll + (1.0 - alpha) * acc_roll

# ---------------- Servo control ----------------
servo = PWM(Pin("D4"))
servo.freq(50)

def set_servo_angle(filtered_roll):
    global servo_angle_roll

    # Remove tiny movements
    filtered_roll = apply_deadband(filtered_roll, angle_deadband)

    # Get sign and magnitude
    sign = 1
    if filtered_roll < 0:
        sign = -1

    mag = abs(filtered_roll)

    # Available servo travel from centre to each end
    max_up = roll_max - roll_center
    max_down = roll_center - roll_min

    # Use smaller side for symmetric safe control
    max_control = min(max_up, max_down)

    # Control value at threshold using normal proportional law
    control_at_threshold = kp_roll * boost_threshold
    if control_at_threshold > max_control:
        control_at_threshold = max_control

    # Region 1: normal proportional response up to 15 deg
    if mag <= boost_threshold:
        control = kp_roll * mag

    # Region 2: smooth exponential ramp from 15 deg to 30 deg
    elif mag < full_actuation_tilt:
        extra = mag - boost_threshold
        span = full_actuation_tilt - boost_threshold

        # Normalized exponential ramp:
        # 0 at 15 deg, 1 at 30 deg
        num = math.exp(boost_exp_gain * extra) - 1.0
        den = math.exp(boost_exp_gain * span) - 1.0

        if den == 0:
            ramp = extra / span
        else:
            ramp = num / den

        control = control_at_threshold + (max_control - control_at_threshold) * ramp

    # Region 3: 30 deg and above = full control
    else:
        control = max_control

    # Restore sign
    control = sign * control

    # Convert control output to servo target
    target_roll = roll_center + (roll_sign * control)

    # Constrain target
    target_roll = max(roll_min, min(roll_max, target_roll))

    # Smooth servo motion
    servo_angle_roll = low_pass(servo_angle_roll, target_roll, servo_smoothing)

    # Output PWM
    duty = angle_to_duty(servo_angle_roll)
    servo.duty_u16(duty)

# ---------------- Startup ----------------
mpu_init()
calibrate_gyro()

# Let IMU settle
for _ in range(20):
    update_imu(0.01)
    time.sleep_ms(10)

# Initialise filtered angle from accelerometer estimate
AngleRoll = acc_roll

# Move servo to centre
servo_angle_roll = roll_center
servo.duty_u16(angle_to_duty(servo_angle_roll))
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
    set_servo_angle(AngleRoll)

    print(
        "acc_roll={:.2f} filt_roll={:.2f} srv_roll={:.2f} dt={:.4f}".format(
            acc_roll, AngleRoll, servo_angle_roll, dt
        )
    )

    time.sleep_ms(10)