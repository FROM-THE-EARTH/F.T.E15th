# ロケット
## レガシー電装引継ぎ資料
### PDF

### ソースコード(2025伊豆)
- IZU_dataloger：bme280とbno055から取得したデータをsdカードに保存する  
  (https://github.com/FROM-THE-EARTH/F.T.E15th/tree/main/Rocket/%E6%98%A5%E9%99%B8/Datalogger/code/IZU_datalogger)
- izu_master:bmeとGPSから取得したデータをダウンリンク用のarduinoにI2Cで送る  
  (https://github.com/FROM-THE-EARTH/F.T.E15th/tree/main/Rocket/%E6%98%A5%E9%99%B8/Rocket%20controller/code/IZU_CONT_LOGGER/izu_master)
- izu_slave:izu_masterから送られてきたデータをsdカード、IMにダウンリンク  
  (https://github.com/FROM-THE-EARTH/F.T.E15th/tree/main/Rocket/%E6%98%A5%E9%99%B8/Rocket%20controller/code/IZU_CONT_LOGGER/izu_slave)

### ソースコード(2025能代)
- bme_alt:bme280の値を取得し、閾値を超えたら、モーター回転用のarduinoのピンにHGIHの信号を送る  
(https://github.com/FROM-THE-EARTH/F.T.E15th/tree/main/Rocket/nse%E6%A2%9F/rockecon/bme_alt)
- servo_bme:フライトピン、高度、タイマーの情報から、DCモーターを回す  
(https://github.com/FROM-THE-EARTH/F.T.E15th/tree/main/Rocket/nse%E6%A2%9F/rockecon/servo_bme)  
## ジンバル引継ぎ資料  
### PDF
https://drive.google.com/file/d/1tlNN9snPiJ_Vbj3hQbi9qIZ8SEv8Pp8z/view?usp=drive_link  
アルゴリズムや搭載部品、マイコンのセットアップ方法などが記述されています  
### STM32プロジェクトフォルダ
https://github.com/FROM-THE-EARTH/F.T.E15th/tree/main/Rocket/nse%E6%A2%9F/gimbal
- bno：bno055のテストコード(Tera Termでprintの出力が見れます、ArduinoIDEで見る方法もあるらしいです)  
- gimbalver5：能代梟ジンバル  
- gimbal_15th：15期代ジンバル(仮)初期設定以外何もしてません  
- SDカードへのデータログ保存については経験がなく無知なので、伊豆梟のときに大也さんが作ったものを貼るに留めます、ご了承ください  
https://github.com/grafen1202/SDencoder
# CanSat
- 16期新プロ：https://github.com/yamasuge-kazuki/newpro/tree/main
- 能代2025 (plane2):https://github.com/yamasuge-kazuki/plane2
