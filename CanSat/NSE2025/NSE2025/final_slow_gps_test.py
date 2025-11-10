import serial
import pynmea2
import time
import os

def clear_screen():
    """画面をクリアする関数"""
    # Windowsの場合
    if os.name == 'nt':
        _ = os.system('cls')
    # MacやLinuxの場合
    else:
        _ = os.system('clear')

# --- 設定 ---
SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 115200
UPDATE_INTERVAL_SECONDS = 5 # 表示を更新する間隔（秒）

try:
    ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
    print("🛰️ GPS受信待機中... (Ctrl+C で終了)")
    last_print_time = 0
    is_first_fix = True

    while True:
        line = ser.readline().decode('utf-8', errors='ignore')

        # GGAセンテンス（位置情報）の場合のみ処理
        if line.startswith(('$GPGGA', '$GNGGA')):
            try:
                msg = pynmea2.parse(line)
                
                current_time = time.time()
                # 緯度・経度が有効で、かつ指定した更新間隔が過ぎていれば表示
                if msg.latitude != 0.0 and msg.longitude != 0.0 and (current_time - last_print_time > UPDATE_INTERVAL_SECONDS):
                    
                    if is_first_fix:
                        clear_screen() # 最初の測位成功時に画面をクリア
                        is_first_fix = False

                    # 結果を整形して表示
                    print("✅ **GPS測位成功！**")
                    print("-" * 35)
                    print(f"  タイムスタンプ: {msg.timestamp}")
                    print(f"  緯度        : {msg.latitude:.6f} {msg.lat_dir}")
                    print(f"  経度        : {msg.longitude:.6f} {msg.lon_dir}")
                    print(f"  高度        : {msg.altitude} {msg.altitude_units}")
                    print(f"  使用衛星数  : {msg.num_sats}")
                    print("-" * 35)
                    print(f"({UPDATE_INTERVAL_SECONDS}秒ごとに更新。 Ctrl+Cで終了)")
                    
                    last_print_time = current_time

            except pynmea2.ParseError:
                # 解析エラーは無視して次の行へ
                continue

except serial.SerialException as e:
    print(f"エラー: {e}")
except KeyboardInterrupt:
    print("\nプログラムを終了します。")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
