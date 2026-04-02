#ac3847, pn515, hck49
from machine import I2C, Pin, PWM
import time
import math

# =========================================================
# MPU6050 + 2-servo TVC stabilisation
# =========================================================

MPU_ADDR = 0x68

# ---------------- I2C setup ----------------
i2c = I2C(0, sda=Pin("A4"), scl=Pin("A5"), freq=100000)
print("I2C scan:", [hex(x) for x in i2c.scan()])

# ---------------- Global state ----------------
RateRoll = 0.0
RatePitch = 0.0
RateYaw = 0.0

AccX = 0.0
AccY = 0.0
AccZ = 0.0

acc_roll = 0.0
acc_pitch = 0.0

AngleRoll = 0.0
AnglePitch = 0.0

# Gyro zero offsets
gyro_bias_x = 0.0
gyro_bias_y = 0.0
gyro_bias_z = 0.0

# Complementary filter weight
alpha = 0.40

# ---------------- Servo settings ----------------
roll_center = 80
pitch_center = 130

kp_roll = 0.8
kp_pitch = 1.5

roll_sign = 1
pitch_sign = 1

roll_min = 35
roll_max = 130

pitch_min = 40
pitch_max = 180

angle_deadband = 3.0
servo_smoothing = 0.2

boost_threshold = 15.0
full_actuation_tilt = 30.0
boost_exp_gain_roll = 0.2
boost_exp_gain_pitch = 0.5
    
# 50 Hz servo pulse range mapped to 16-bit PWM duty
min_duty = 1638
max_duty = 8192

servo_angle_roll = roll_center
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

def clamp(x, low, high):
    if x < low:
        return low
    if x > high:
        return high
    return x

def apply_deadband(x, deadband):
    if abs(x) < deadband:
        return 0.0
    return x

def low_pass(prev, new, smoothing):
    return prev + smoothing * (new - prev)

def angle_to_duty(angle):
    angle = clamp(angle, 0, 180)
    return int(min_duty + (angle / 180.0) * (max_duty - min_duty))

# ---------------- MPU6050 init ----------------
def mpu_init():
    write_reg(0x6B, 0x00)
    time.sleep_ms(100)

    write_reg(0x1A, 0x03)
    write_reg(0x1C, 0x10)
    write_reg(0x1B, 0x08)

    time.sleep_ms(100)

# ---------------- Gyro calibration ----------------
def calibrate_gyro(samples=500):
    global gyro_bias_x, gyro_bias_y

    sum_x = 0
    sum_y = 0

    print("Calibrating gyro... keep system still")

    for _ in range(samples):
        data = read_bytes(0x43, 6)
        gx = to_int16(data[0], data[1])
        gy = to_int16(data[2], data[3])

        sum_x += gx
        sum_y += gy

        time.sleep_ms(2)

    gyro_bias_x = sum_x / samples
    gyro_bias_y = sum_y / samples

    print("Gyro roll bias:", gyro_bias_x)
    print("Gyro pitch bias:", gyro_bias_y)

# ---------------- Sensor fusion ----------------
def update_imu(dt):
    global RateRoll, RatePitch
    global AccX, AccY, AccZ
    global acc_roll, acc_pitch
    global AngleRoll, AnglePitch

    acc_data = read_bytes(0x3B, 6)
    acc_x_lsb = to_int16(acc_data[0], acc_data[1])
    acc_y_lsb = to_int16(acc_data[2], acc_data[3])
    acc_z_lsb = to_int16(acc_data[4], acc_data[5])

    gyro_data = read_bytes(0x43, 6)
    gyro_x_raw = to_int16(gyro_data[0], gyro_data[1]) - gyro_bias_x
    gyro_y_raw = to_int16(gyro_data[2], gyro_data[3]) - gyro_bias_y

    RateRoll = gyro_x_raw / 65.5
    RatePitch = gyro_y_raw / 65.5


    AccX = (acc_x_lsb / 4096.0) 
    AccY = (acc_y_lsb / 4096.0) 
    AccZ = (acc_z_lsb / 4096.0) 

    acc_roll = math.degrees(math.atan2(AccY, math.sqrt(AccX * AccX + AccZ * AccZ))) + 3.5
    acc_pitch = math.degrees(math.atan2(AccX, math.sqrt(AccY * AccY + AccZ * AccZ))) - 4.7

    gyro_roll = AngleRoll + RateRoll * dt
    gyro_pitch = AnglePitch + RatePitch * dt

    AngleRoll = alpha * gyro_roll + (1.0 - alpha) * acc_roll
    AnglePitch = alpha * gyro_pitch + (1.0 - alpha) * acc_pitch

# ---------------- Shared control law ----------------
def control_axis(angle, center, axis_sign, min_angle, max_angle, kp, boost_exp_gain):
    # Determine sign and magnitude of tilt
    if angle >= 0:
        sign_angle = 1
    else:
        sign_angle = -1

    mag = abs(angle)
    mag = apply_deadband(mag, angle_deadband)

    # Determine which direction servo will move for this tilt
    servo_direction = axis_sign * sign_angle

    # Available servo travel depends on direction
    if servo_direction > 0:
        max_control = max_angle - center
    else:
        max_control = center - min_angle

    if max_control < 0:
        max_control = 0

    # Control at threshold using normal proportional law
    control_at_threshold = kp * boost_threshold
    if control_at_threshold > max_control:
        control_at_threshold = max_control

    # Region 1: normal proportional response up to 15 deg
    if mag <= boost_threshold:
        control_mag = kp * mag

    # Region 2: smooth exponential ramp from 15 deg to 30 deg
    elif mag < full_actuation_tilt:
        extra = mag - boost_threshold
        span = full_actuation_tilt - boost_threshold

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
    control = sign_angle * control_mag

    # Convert to servo target
    target = center + (axis_sign * control)

    # Constrain target
    target = max(min_angle, min(max_angle, target))

    return target
    
# ---------------- Startup ----------------
mpu_init()
calibrate_gyro()

for _ in range(20):
    update_imu(0.01)
    time.sleep_ms(10)

AngleRoll = acc_roll
AnglePitch = acc_pitch

servo_angle_roll = roll_center
servo_angle_pitch = pitch_center
    
servo_roll.duty_u16(angle_to_duty(servo_angle_roll, roll_min, roll_max))
servo_pitch.duty_u16(angle_to_duty(servo_angle_pitch, pitch_min, pitch_max))
    
time.sleep_ms(1000)

# ---------------- Main loop ----------------
last_us = time.ticks_us()

while True:
    now_us = time.ticks_us()
    dt = time.ticks_diff(now_us, last_us) / 1000000.0
    last_us = now_us

    if dt <= 0 or dt > 0.1:
        dt = 0.02

    update_imu(dt)

    target_roll = control_axis(AngleRoll, roll_center, roll_sign, roll_min, roll_max, kp_roll,boost_exp_gain_roll)

    target_pitch = control_axis(AnglePitch, pitch_center, pitch_sign, pitch_min, pitch_max, kp_pitch, boost_exp_gain_pitch)

    # Smooth servo motion
    servo_angle_roll = low_pass(servo_angle_roll, target_roll, servo_smoothing)
    servo_angle_pitch = low_pass(servo_angle_pitch, target_pitch, servo_smoothing)

    # Output PWM
    servo_roll.duty_u16(angle_to_duty(servo_angle_roll, roll_min, roll_max))
    servo_pitch.duty_u16(angle_to_duty(servo_angle_pitch, pitch_min, pitch_max))

    print(
        "roll={:.2f} pitch={:.2f} srv_roll={:.2f} srv_pitch={:.2f} dt={:.4f}".format(
            AngleRoll, AnglePitch, servo_angle_roll, servo_angle_pitch, dt
        )
    )

    time.sleep_ms(10)
