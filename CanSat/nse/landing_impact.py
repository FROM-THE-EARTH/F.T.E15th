import RPi.GPIO as GPIO
from library import BNO055 as bno055
import Adafruit_BMP.BMP085 as BMP085
import serial
import pynmea2
import math
import datetime
import csv
import time

# --- ユーザー設定項目 ---
SERIAL_PORT = "/dev/serial0"
LOG_DIRECTORY = "/home/raspberry/log/landing_impact/"

# --- センサーの初期化 ---
def setup_sensors():
    """各センサーのセットアップを行う"""
    bno = None
    bmp = None
    gps_serial = None

    print("BNO055をセットアップ中...")
    try:
        bno = bno055.BNO055()
        if not bno.setUp():
            print("BNO055のセットアップに失敗しました。")
            bno = None
        else:
            print("BNO055のセットアップ完了。")
    except Exception as e:
        print(f"BNO055の初期化中にエラーが発生しました: {e}")

    print("BMP180をセットアップ中...")
    try:
        bmp = BMP085.BMP085(busnum=1)
        temp = bmp.read_temperature()
        print(f"BMP180のセットアップ完了。現在の温度: {temp:.2f}C")
    except Exception as e:
        print(f"BMP180の初期化中にエラーが発生しました: {e}")

    print("GPSをセットアップ中...")
    try:
        gps_serial = serial.Serial(SERIAL_PORT, 9600, timeout=1.0)
        print("GPSのセットアップ完了。")
    except Exception as e:
        print(f"GPSのシリアルポート({SERIAL_PORT})を開けませんでした: {e}")

    return bno, bmp, gps_serial

# --- データ取得関数 ---
# --- データ取得関数 ---
def get_all_data(bno, bmp, gps_serial):
    """全てのセンサーからデータを取得し、辞書形式で返す（デバッグ強化版）"""
    data = {
        'acc_x': 0.0, 'acc_y': 0.0, 'acc_z': 0.0, 'acc_combined': 0.0, 'gyro_x': 0.0,
        'gyro_y': 0.0, 'gyro_z': 0.0, 'mag_x': 0.0, 'mag_y': 0.0, 'mag_z': 0.0,
        'temp': 0.0, 'pressure': 0.0, 'altitude_bmp': 0.0, 'latitude': 0.0,
        'longitude': 0.0, 'altitude_gps': 0.0, 'num_sats': 0, 'gps_timestamp': '00:00:00'
    }

    # BNO055のデータ取得を一つずつ実行して確認
    if bno:
        try:
            print("  [BNO055] getAcc() を実行中...")
            acc = bno.getAcc()
            data.update({'acc_x': acc[0], 'acc_y': acc[1], 'acc_z': acc[2],
                         'acc_combined': math.sqrt(acc[0]**2 + acc[1]**2 + acc[2]**2)})
            print("  [BNO055] getAcc() 成功")

            print("  [BNO055] getGyro() を実行中...")
            gyro = bno.getGyro()
            data.update({'gyro_x': gyro[0], 'gyro_y': gyro[1], 'gyro_z': gyro[2]})
            print("  [BNO055] getGyro() 成功")

            print("  [BNO055] getMag() を実行中...")
            mag = bno.getMag()
            data.update({'mag_x': mag[0], 'mag_y': mag[1], 'mag_z': mag[2]})
            print("  [BNO055] getMag() 成功")

        except Exception as e:
            print(f"  [BNO055] データ取得中にエラーが発生: {e}")

    # BMP180のデータ取得
    if bmp:
        try:
            print("  [BMP180] データ取得中...")
            data.update({'temp': bmp.read_temperature(), 'pressure': bmp.read_pressure(), 'altitude_bmp': bmp.read_altitude()})
            print("  [BMP180] 成功")
        except Exception as e:
            print(f"  [BMP180] データ取得エラー: {e}")

    # GPSのデータ取得
    if gps_serial:
        try:
            print("  [GPS]    readline() を実行中...")
            line = gps_serial.readline().decode('utf-8', errors='ignore')
            if line:
                if line.startswith('$GPGGA'):
                    msg = pynmea2.parse(line)
                    data.update({'latitude': msg.latitude, 'longitude': msg.longitude,
                                 'altitude_gps': msg.altitude, 'num_sats': msg.num_sats,
                                 'gps_timestamp': msg.timestamp.strftime('%H:%M:%S')})
            else:
                print("  [GPS]    データ受信待機中... (タイムアウト)")
        except Exception as e:
            print(f"  [GPS]    データ処理エラー: {e}")

    return data

# --- メイン処理 ---
if __name__ == "__main__":
    bno, bmp, gps_serial = setup_sensors()
    if not any([bno, bmp, gps_serial]):
        print("全てのセンサーの初期化に失敗しました。プログラムを終了します。"); exit()

    print("\nセンサーの安定を待っています..."); time.sleep(1)
    print("データ取得を開始します。Ctrl+Cで停止します。\n")

    now_time = datetime.datetime.now()
    file_name = f"{LOG_DIRECTORY}impact_log_{now_time.strftime('%Y%m%d_%H%M%S')}.csv"
    header = ['Time[s]', 'Acc_X[m/s^2]', 'Acc_Y[m/s^2]', 'Acc_Z[m/s^2]', 'Acc_Combined[m/s^2]', 'Gyro_X[dps]', 'Gyro_Y[dps]', 'Gyro_Z[dps]', 'Mag_X[uT]', 'Mag_Y[uT]', 'Mag_Z[uT]', 'Temp[C]', 'Pressure[hPa]', 'Altitude_BMP[m]', 'Latitude', 'Longitude', 'Altitude_GPS[m]', 'Num_Satellites', 'GPS_Timestamp']
    try:
        import os
        os.makedirs(LOG_DIRECTORY, exist_ok=True)
        with open(file_name, 'w', newline='') as f:
            writer = csv.writer(f); writer.writerow(header)
    except IOError as e: print(f"ログファイルの初期化に失敗しました: {e}"); exit()

    start_time = time.time()
    last_gps_data = {}
    loop_count = 0
    try:
        while True:
            loop_count += 1
            elapsed_time = time.time() - start_time
            print(f"--- ループ: {loop_count} | 経過時間: {elapsed_time:.2f}s ---")
            sensor_data = get_all_data(bno, bmp, gps_serial)
            print(f"  [BNO055] Acc_Combined: {sensor_data['acc_combined']:.3f}")
            if bmp: print(f"  [BMP180] Temp: {sensor_data['temp']:.2f}, Press: {sensor_data['pressure']:.2f}")
            else: print("  [BMP180] スキップ（初期化失敗）")

            if sensor_data['latitude'] != 0.0:
                 last_gps_data = {k: v for k, v in sensor_data.items() if k in ['latitude', 'longitude', 'altitude_gps', 'num_sats', 'gps_timestamp']}
                 print(f"  [GPS]    データ更新 -> 衛星数: {last_gps_data['num_sats']}, 緯度: {last_gps_data['latitude']:.4f}")
            
            merged_data = {**sensor_data, **last_gps_data}
            row_data = [f"{elapsed_time:.3f}", f"{merged_data.get('acc_x', 0):.4f}", f"{merged_data.get('acc_y', 0):.4f}", f"{merged_data.get('acc_z', 0):.4f}", f"{merged_data.get('acc_combined', 0):.4f}", f"{merged_data.get('gyro_x', 0):.4f}", f"{merged_data.get('gyro_y', 0):.4f}", f"{merged_data.get('gyro_z', 0):.4f}", f"{merged_data.get('mag_x', 0):.4f}", f"{merged_data.get('mag_y', 0):.4f}", f"{merged_data.get('mag_z', 0):.4f}", f"{merged_data.get('temp', 0):.2f}", f"{merged_data.get('pressure', 0):.2f}", f"{merged_data.get('altitude_bmp', 0):.2f}", f"{merged_data.get('latitude', 0):.6f}", f"{merged_data.get('longitude', 0):.6f}", f"{merged_data.get('altitude_gps', 0)}", merged_data.get('num_sats', 0), merged_data.get('gps_timestamp', '00:00:00')]
            try:
                with open(file_name, 'a', newline='') as f: writer = csv.writer(f); writer.writerow(row_data)
                print(f"  [CSV]    ファイルへの書き込み完了")
            except IOError as e: print(f"  [CSV]    ファイル書き込みエラー: {e}")
            print("-" * 35 + "\n")
            time.sleep(0.2)
    except KeyboardInterrupt: print("\n\nプログラムが中断されました。")
    except Exception as e: print(f"\n予期せぬエラーが発生しました: {e}")
    finally:
        if gps_serial and gps_serial.is_open: gps_serial.close()
        GPIO.cleanup()
        print(f"ログファイル '{file_name}' の保存を完了しました。")
