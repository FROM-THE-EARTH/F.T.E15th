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
SERIAL_PORT = "/dev/ttyS0"
LOG_DIRECTORY = "/home/raspberry/log/landing_impact/"

# --- センサーの初期化 ---
def setup_sensors():
    """各センサーのセットアップを行う"""
    bno = None
    bmp = None
    gps_serial = None

    # BNO055 (9軸センサー)
    try:
        print("BNO055をセットアップ中...")
        bno = bno055.BNO055()
        if not bno.setUp():
            print("BNO055のセットアップに失敗しました。")
            bno = None
        else:
            print("BNO055のセットアップ完了。")
    except Exception as e:
        print(f"BNO055の初期化中にエラーが発生しました: {e}")

    # BMP180 (気圧センサー)
    try:
        print("BMP180をセットアップ中...")
        # ### 変更点 1: I2Cバス番号を明示的に指定 ###
        # Raspberry Piの多くはI2Cバス1を使用します。
        bmp = BMP085.BMP085(busnum=1)
        # 動作確認のため、一度温度を読み込んでみる
        temp = bmp.read_temperature()
        print(f"BMP180のセットアップ完了。現在の温度: {temp:.2f}C")
    except Exception as e:
        print(f"BMP180の初期化中にエラーが発生しました: {e}")

    # GEP-M10 (GPS)
    try:
        print("GPSをセットアップ中...")
        gps_serial = serial.Serial(SERIAL_PORT, 112500, timeout=1.0)
        print("GPSのセットアップ完了。")
    except Exception as e:
        print(f"GPSのシリアルポート({SERIAL_PORT})を開けませんでした: {e}")

    return bno, bmp, gps_serial

# --- データ取得関数 (変更なし) ---
def get_all_data(bno, bmp, gps_serial):
    # (この関数の中身は変更ありません)
    data = {
        'acc_x': 0.0, 'acc_y': 0.0, 'acc_z': 0.0, 'acc_combined': 0.0,
        'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0,
        'mag_x': 0.0, 'mag_y': 0.0, 'mag_z': 0.0,
        'temp': 0.0, 'pressure': 0.0, 'altitude_bmp': 0.0,
        'latitude': 0.0, 'longitude': 0.0, 'altitude_gps': 0.0,
        'num_sats': 0, 'gps_timestamp': '00:00:00'
    }
    if bno:
        try:
            acc = bno.getAcc()
            gyro = bno.getGyro()
            mag = bno.getMag()
            data.update({
                'acc_x': acc[0], 'acc_y': acc[1], 'acc_z': acc[2],
                'acc_combined': math.sqrt(acc[0]**2 + acc[1]**2 + acc[2]**2),
                'gyro_x': gyro[0], 'gyro_y': gyro[1], 'gyro_z': gyro[2],
                'mag_x': mag[0], 'mag_y': mag[1], 'mag_z': mag[2]
            })
        except Exception: pass
    if bmp:
        try:
            data.update({
                'temp': bmp.read_temperature(),
                'pressure': bmp.read_pressure(),
                'altitude_bmp': bmp.read_altitude()
            })
        except Exception: pass
    if gps_serial:
        try:
            line = gps_serial.readline().decode('utf-8', errors='ignore')
            if line.startswith('$GPGGA'):
                msg = pynmea2.parse(line)
                data.update({
                    'latitude': msg.latitude, 'longitude': msg.longitude,
                    'altitude_gps': msg.altitude, 'num_sats': msg.num_sats,
                    'gps_timestamp': msg.timestamp.strftime('%H:%M:%S')
                })
        except Exception: pass
    return data

# --- メイン処理 ---
if __name__ == "__main__":
    bno, bmp, gps_serial = setup_sensors()

    if not any([bno, bmp, gps_serial]):
        print("全てのセンサーの初期化に失敗しました。プログラムを終了します。")
        exit()

    print("\nセンサーの安定を待っています...")
    time.sleep(1)
    print("データ取得を開始します。Ctrl+Cで停止します。\n")

    now_time = datetime.datetime.now()
    file_name = f"{LOG_DIRECTORY}impact_log_{now_time.strftime('%Y%m%d_%H%M%S')}.csv"
    
    header = [
        'Time[s]', 'Acc_X[m/s^2]', 'Acc_Y[m/s^2]', 'Acc_Z[m/s^2]', 'Acc_Combined[m/s^2]',
        'Gyro_X[dps]', 'Gyro_Y[dps]', 'Gyro_Z[dps]', 'Mag_X[uT]', 'Mag_Y[uT]', 'Mag_Z[uT]',
        'Temp[C]', 'Pressure[hPa]', 'Altitude_BMP[m]',
        'Latitude', 'Longitude', 'Altitude_GPS[m]', 'Num_Satellites', 'GPS_Timestamp'
    ]
    try:
        import os
        os.makedirs(LOG_DIRECTORY, exist_ok=True)
        with open(file_name, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
    except IOError as e:
        print(f"ログファイルの初期化に失敗しました: {e}")
        exit()

    start_time = time.time()
    last_gps_data = {}

    try:
        loop_count = 0
        while True:
            loop_count += 1
            elapsed_time = time.time() - start_time
            
            # ### 変更点 2: デバッグ表示の強化 ###
            print(f"--- ループ: {loop_count} | 経過時間: {elapsed_time:.2f}s ---")

            # 全てのセンサーからデータを取得
            sensor_data = get_all_data(bno, bmp, gps_serial)
            print(f"  [BNO055] Acc_Combined: {sensor_data['acc_combined']:.3f}")
            if bmp:
                print(f"  [BMP180] Temp: {sensor_data['temp']:.2f}, Press: {sensor_data['pressure']:.2f}")
            else:
                print("  [BMP180] スキップ（初期化失敗）")

            # GPSデータ処理と表示
            if sensor_data['latitude'] != 0.0:
                 last_gps_data = {k: v for k, v in sensor_data.items() if k in ['latitude', 'longitude', 'altitude_gps', 'num_sats', 'gps_timestamp']}
                 print(f"  [GPS]    データ更新 -> 衛星数: {last_gps_data['num_sats']}, 緯度: {last_gps_data['latitude']:.4f}")
            else:
                print("  [GPS]    新規データなし")
            
            # 結合
            merged_data = {**sensor_data, **last_gps_data}

            # CSV書き込み
            row_data = [
                f"{elapsed_time:.3f}",
                f"{merged_data.get('acc_x', 0):.4f}", f"{merged_data.get('acc_y', 0):.4f}", f"{merged_data.get('acc_z', 0):.4f}", f"{merged_data.get('acc_combined', 0):.4f}",
                f"{merged_data.get('gyro_x', 0):.4f}", f"{merged_data.get('gyro_y', 0):.4f}", f"{merged_data.get('gyro_z', 0):.4f}",
                f"{merged_data.get('mag_x', 0):.4f}", f"{merged_data.get('mag_y', 0):.4f}", f"{merged_data.get('mag_z', 0):.4f}",
                f"{merged_data.get('temp', 0):.2f}", f"{merged_data.get('pressure', 0):.2f}", f"{merged_data.get('altitude_bmp', 0):.2f}",
                f"{merged_data.get('latitude', 0):.6f}", f"{merged_data.get('longitude', 0):.6f}", f"{merged_data.get('altitude_gps', 0)}",
                merged_data.get('num_sats', 0), merged_data.get('gps_timestamp', '00:00:00')
            ]
            try:
                with open(file_name, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(row_data)
                print(f"  [CSV]    ファイルへの書き込み完了: {file_name}")
            except IOError as e:
                print(f"  [CSV]    ファイル書き込みエラー: {e}")

            print("-" * 35 + "\n")
            
            # ### 変更点 3: デバッグ用に待機時間を長くする ###
            # ログが見やすくなるように1秒待機。実際の計測時は0.1などに変更してください。
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n\nプログラムが中断されました。")
    except Exception as e:
        print(f"\n予期せぬエラーが発生しました: {e}")
    finally:
        if gps_serial and gps_serial.is_open:
            gps_serial.close()
        GPIO.cleanup()
        print(f"ログファイル '{file_name}' の保存を完了しました。")
