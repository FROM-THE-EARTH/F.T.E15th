import RPi.GPIO as GPIO
import time

# GPIOピンの番号設定（BCMモード）
# 使用するモータードライバーの仕様に合わせてピン番号を変更してください
MOTOR_A1 = 17  # IN1
MOTOR_A2 = 27  # IN2
MOTOR_PWM = 22 # ENA (PWM)

# 初期設定
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_A1, GPIO.OUT)
GPIO.setup(MOTOR_A2, GPIO.OUT)
GPIO.setup(MOTOR_PWM, GPIO.OUT)

# PWMオブジェクトの作成（100Hz）
pwm = GPIO.PWM(MOTOR_PWM, 100)
pwm.start(0) # PWMを開始（デューティ比0%）

def motor_forward(speed):
    """モーターを正回転させる関数"""
    print(f"モーターを {speed}% の速度で正回転させます。")
    GPIO.output(MOTOR_A1, GPIO.HIGH)
    GPIO.output(MOTOR_A2, GPIO.LOW)
    pwm.ChangeDutyCycle(speed)

def motor_stop():
    """モーターを停止させる関数"""
    print("モーターを停止します。")
    GPIO.output(MOTOR_A1, GPIO.LOW)
    GPIO.output(MOTOR_A2, GPIO.LOW)
    pwm.ChangeDutyCycle(0)

try:
    # 50%の速度で3秒間回転
    motor_forward(50)
    time.sleep(3)

    # 100%の速度で3秒間回転
    motor_forward(100)
    time.sleep(3)

    # 停止
    motor_stop()

except KeyboardInterrupt:
    # Ctrl+Cで終了した場合の処理
    print("プログラムを終了します。")

finally:
    # 終了時にGPIOをクリーンアップ
    pwm.stop()
    GPIO.cleanup()