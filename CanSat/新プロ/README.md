# FTE14th CanSatの環境構築
## 1. MicroController & Sensor
### CanSat : NSE2024機体 SAKURA  

|  デバイス  |          型番           |
|:------:|:---------------------:|
|  マイコン  | Raspberry Pi  Zero 2W |
| 9軸センサ  |        BMX055         |
|  GPS   |        GEP-M10        |
| 気圧センサ  |        BMP180         |
| camera |     Raspi-camera      |

## 2. Install Raspberry Pi  OS
Raspberry Pi OS 32bit Bullseye Full OSを使用。Pythonは3.xで開発を行う。
Raspberry Pi はMicroSDにOSを入れてからRaspberry Piに電源を入れることで起動することができます。まずはこれらをインストールします。

  

1. Raspberry Pi Imager を使用しRaspberrypiにOSを書き込む[Install Raspi Imager](https://www.indoorcorgielec.com/resources/raspberry-pi/raspberry-pi-os%E3%81%AE%E3%82%A4%E3%83%B3%E3%82%B9%E3%83%88%E3%83%BC%E3%83%AB/).
2. OSからRaspberry Pi OS (other)を選択し上と同じversionのものを選ぶ
3. インストール後、raspiにセットして起動



## 3 Update & Upgrade
RaspberryPiのセットアップが完了したら、ターミナルから必要なパッケージのインストールを行います.

システムのすべてのパッケージをアップグレード
```
sudo apt-get upgrade
```

ローカルのパッケージリストの最新化
```
sudo apt-get update
```

## 4 Install Packages
パッケージのアップデートができれば、モーターやセンサーを動かすために必要なパッケージやライブラリをインストールしていきます。

### 4.1 Install WiringPi 
wiringpiはRaspberryPiからPWMの信号を出すためのpython用ライブラリです。PWMとは[PWM](https://zakkuri-kaisetsu.com/pwm/).  









 