import RPi.GPIO as GPIO
from library import BNO055 as bno055
import math
import datetime
import csv
import time

nowTime = datetime.datetime.now()
bno=bno055.BNO055()

if not bno.setUp():
    print("BNO055のセットアップに失敗しました。プログラムを終了します。")
    exit()

# センサーが安定するまで1秒待機
print("センサーの安定を待っています...")
time.sleep(1) 
print("データ取得を開始します。")

fileName = '/home/raspberry/log/ground_shock/testlog_' + nowTime.strftime('%Y-%m%d-%H%M%S') + '.csv'
start=time.time()

# --- 変更点1: ヘッダーの書き込み ---
# ファイルを新規作成モード('w')で開き、ヘッダーを書き込む
try:
    with open(fileName, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time[s]', 'Acc_X[m/s^2]', 'Acc_Y[m/s^2]', 'Acc_Z[m/s^2]', 'Acc_Combined[m/s^2]'])
except IOError as e:
    print(f"ファイルの初期化に失敗しました: {e}")
    exit()

try:
    print("センサーの生データを確認します。")
    while True:
        # --- ▼▼▼ 生データを直接読み取る ▼▼▼ ---
        raw_x = bno._read_signed_word(bno.BNO055_ACCEL_DATA_X_LSB_ADDR)
        raw_y = bno._read_signed_word(bno.BNO055_ACCEL_DATA_Y_LSB_ADDR)
        raw_z = bno._read_signed_word(bno.BNO055_ACCEL_DATA_Z_LSB_ADDR)
        
        # 計算後の値も取得
        acc = bno.getAcc()
        cal_status = bno.getCalibrationStatus()
        
        # 生データと計算後の値を表示
        print(f"計算後の値 Z: {acc[2]:.2f} | 生データ [X,Y,Z]: {raw_x}, {raw_y}, {raw_z} | Cal: {cal_status}  ", end='\r')
        # --- ▲▲▲ ここまで変更 ▲▲▲ ---

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nプログラムが中断されました。")
except IOError as e:
    print(f"I2C通信エラーが発生しました: {e}")
    print("配線やセンサーの電源を確認してください。")
finally:
    # GPIOのクリーンアップなど、終了処理があればここに記述
    pass
