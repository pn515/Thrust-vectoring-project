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
alpha = 0.98

# ---------------- Servo settings ----------------
roll_center = 80
pitch_center = 85

kp_roll = 0.8
kp_pitch = 2

roll_sign = 1
pitch_sign = -1

roll_min = 30
roll_max = 110

pitch_min = 0
pitch_max = 130

angle_deadband = 0.3

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
    global gyro_bias_x, gyro_bias_y, gyro_bias_z

    sum_x = 0
    sum_y = 0
    sum_z = 0

    print("Calibrating gyro... keep system still")

    for _ in range(samples):
        data = read_bytes(0x43, 6)
        gx = to_int16(data[0], data[1])
        gy = to_int16(data[2], data[3])
        gz = to_int16(data[4], data[5])

        sum_x += gx
        sum_y += gy
        sum_z += gz

        time.sleep_ms(2)

    gyro_bias_x = sum_x / samples
    gyro_bias_y = sum_y / samples
    gyro_bias_z = sum_z / samples

    print("Gyro bias:", gyro_bias_x, gyro_bias_y, gyro_bias_z)

# ---------------- Sensor fusion ----------------
def update_imu(dt):
    global RateRoll, RatePitch, RateYaw
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
    gyro_z_raw = to_int16(gyro_data[4], gyro_data[5]) - gyro_bias_z

    RateRoll = gyro_x_raw / 65.5
    RatePitch = gyro_y_raw / 65.5
    RateYaw = gyro_z_raw / 65.5

    AccX = (acc_x_lsb / 4096.0) + 0.014
    AccY = (acc_y_lsb / 4096.0) + 0.010
    AccZ = (acc_z_lsb / 4096.0) - 0.010

    acc_roll = math.degrees(math.atan2(AccY, math.sqrt(AccX * AccX + AccZ * AccZ))) - 0.5
    acc_pitch = math.degrees(math.atan2(AccX, math.sqrt(AccY * AccY + AccZ * AccZ))) - 3.0

    gyro_roll = AngleRoll + RateRoll * dt
    gyro_pitch = AnglePitch + RatePitch * dt

    AngleRoll = alpha * gyro_roll + (1.0 - alpha) * acc_roll
    AnglePitch = alpha * gyro_pitch + (1.0 - alpha) * acc_pitch

# ---------------- Servo control ----------------
servo1 = PWM(Pin("D4"))
servo2 = PWM(Pin("D5"))
servo1.freq(50)
servo2.freq(50)

def set_servo_angle(filtered_roll, filtered_pitch):
    global servo_angle_roll, servo_angle_pitch

    filtered_roll = apply_deadband(filtered_roll, angle_deadband)
    filtered_pitch = apply_deadband(filtered_pitch, angle_deadband)

    target_roll = roll_center + (roll_sign * kp_roll * filtered_roll)
    target_pitch = pitch_center + (pitch_sign * kp_pitch * filtered_pitch)

    target_roll = clamp(target_roll, roll_min, roll_max)
    target_pitch = clamp(target_pitch, pitch_min, pitch_max)

    # No smoothing
    servo_angle_roll = target_roll
    servo_angle_pitch = target_pitch

    duty1 = angle_to_duty(servo_angle_roll)
    duty2 = angle_to_duty(servo_angle_pitch)

    servo1.duty_u16(duty1)
    servo2.duty_u16(duty2)

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
servo1.duty_u16(angle_to_duty(servo_angle_roll))
servo2.duty_u16(angle_to_duty(servo_angle_pitch))
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
    set_servo_angle(AngleRoll, AnglePitch)

    print(
        "acc_roll={:.2f} acc_pitch={:.2f} filt_roll={:.2f} filt_pitch={:.2f} srv_roll={:.2f} srv_pitch={:.2f} dt={:.4f}".format(
            acc_roll, acc_pitch, AngleRoll, AnglePitch, servo_angle_roll, servo_angle_pitch, dt
        )
    )

    time.sleep_ms(10)
