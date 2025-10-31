# encoding : utf-8
import cv2
import numpy as np
from picamera2 import Picamera2 # camera

class detector():
    # コンストラクタ
    # 引数
    #   roi : 対象領域
    def __init__(self, picam2_instance):
        """
        検出器を初期化します。
        :param picam2_instance: メインプログラムで初期化済みのPicamera2オブジェクト
        """
        # 渡されたカメラオブジェクトが有効か確認
        if picam2_instance is None:
            raise ValueError("カメラオブジェクトが正しく渡されませんでした。")
        
        # 渡されたカメラオブジェクトを、このクラス内で使えるように保存
        self.picam2 = picam2_instance
        
        self.input_img = None
        self.projected_img = None
        self.binarized_img = None
        self.detected = None
        self.probability = None
        self.centroids = None
        self.cone_direction = None
        self.is_detected = None
        self.is_reached = None
        self.cone_ratio = 0.5
    
    # 対象領域をセット
    def set_roi_img(self, roi):
        # 対象領域のヒストグラムをあらかじめ算出
        self.__roi = roi
        self.__roi_hsv = cv2.cvtColor(self.__roi, cv2.COLOR_BGR2HSV)
        self.__roi_hist = cv2.calcHist([self.__roi_hsv], [0, 1], None, [180, 256], [0, 180, 0, 256])
    
    # コーンの縦横比 (横/縦) を設定
    def set_cone_ratio(self, ratio):
        self.cone_ratio = ratio
    
    # get camera img
    def __get_camera_img(self):
        self.input_img = cv2.blur(self.picam2.capture_array(), (8, 8))
        
        
    # 検出
    def detect_cone(self):
        self.__get_camera_img()
        self.__back_projection()
        self.__binarization()
        # ★★★ ここから診断コード ★★★
        # 処理の各段階の画像を保存し、何が起きているかを確認する
        try:
            print("Saving debug images to ./log/ folder...")
            cv2.imwrite("./log/DEBUG_0_input.png", self.input_img)         # 元の入力画像
            cv2.imwrite("./log/DEBUG_1_projected.png", self.projected_img) # 逆投影後の画像
            cv2.imwrite("./log/DEBUG_2_binarized.png", self.binarized_img) # 最終的な二値化マスク画像
        except Exception as e:
            print(f"Debug image save error: {e}")
        # ★★★ ここまで診断コード ★★★
        self.__find_cone_centroid()
    
    # 逆投影法を用いて, 興味領域のヒストグラムにマッチする領域を抽出
    def __back_projection(self):
        img_hsv = cv2.cvtColor(self.input_img, cv2.COLOR_BGR2HSV)
        cv2.normalize(self.__roi_hist, self.__roi_hist, 0, 255, cv2.NORM_MINMAX)
        self.projected_img =  cv2.calcBackProject([img_hsv], [0, 1], self.__roi_hist, [0, 180, 0, 256], 1)
    
    # 二値化・モルフォロジー変換 (クロージング)
    # gray : 入力画像 (グレースケール)
    def __binarization(self):
        ret, th = cv2.threshold(self.projected_img, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU) # 大津の二値化
        self.binarized_img = cv2.morphologyEx(th, cv2.MORPH_DILATE, cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))) # モルフォロジー変換
    
    # ラベリング処理によって, 特定の比の長方形 (i.e. カラーコーン) を探し, その重心と確からしさを返す
    # 確からしさ abs(長方形の縦横比 - コーンの縦横比) でとりあえず定義. 小さいほど良い
    # detect_corn.py の __find_cone_centroid メソッドを以下で完全に置き換える

    def __find_cone_centroid(self):
        # 諸変数の準備
        img_height, img_width = self.binarized_img.shape[:2]
        img_size = img_height * img_width
        
        # ラベリング処理を実行
        nlabels, labels_img, stats, centroids = cv2.connectedComponentsWithStats(self.binarized_img.astype(np.uint8))
        
        # 状態変数をリセット
        self.is_detected = False
        self.is_reached = False
        
        valid_blobs = [] # ノイズ除去後の候補を格納するリスト

        # --- 1. まず「到達判定」と「ノイズ除去」を先に行う ---
        for idx in range(1, nlabels): # 0は背景なので1からループ
            area = stats[idx, cv2.CC_STAT_AREA]
            
            # あまりに小さい領域はノイズとして無視
            if area < img_size / 20000: # この閾値は要調整
                continue

            # 領域が画面の一定割合より大きい場合、「到達」とみなし、処理を即座に終了する
            if area > img_size / 25: # この閾値は要調整
                self.is_detected = True
                self.is_reached = True
                self.centroids = centroids[idx]
                self.cone_direction = self.centroids[0] / img_width
                # 目的達成なので、これ以上他の領域を探す必要はない
                return # ★★★ ここで関数を抜けるのがポイント ★★★

            # 「到達」はしていないが、ノイズではない有効な領域候補としてリストに追加
            valid_blobs.append(idx)

        # --- 2. 「到達」していない場合、残った候補から最もコーンらしい形状のものを探す ---
        if not valid_blobs:
            # 有効な候補が一つもなければ、何も見つからなかったとして終了
            return

        # 有効な候補が見つかったので、検出フラグを立てる
        self.is_detected = True
        
        best_candidate_idx = -1
        min_ratio_error = float('inf')

        for idx in valid_blobs:
            width = stats[idx, cv2.CC_STAT_WIDTH]
            height = stats[idx, cv2.CC_STAT_HEIGHT]
            if height == 0: continue # ゼロ除算を避ける

            # 設定されたコーンの縦横比との誤差を計算
            ratio_error = abs(width / height - self.cone_ratio)
            
            if ratio_error < min_ratio_error:
                min_ratio_error = ratio_error
                best_candidate_idx = idx

        # 最も形状が近かった候補の情報を採用する
        if best_candidate_idx != -1:
            self.detected = stats[best_candidate_idx, :]
            self.centroids = centroids[best_candidate_idx]
            self.probability = min_ratio_error
            self.cone_direction = self.centroids[0] / img_width

        try:
            # 検出状況を元の画像に描画して分かりやすくする
            debug_frame = self.input_img.copy()
            if self.is_detected and not self.is_reached:
                # 追跡中の候補（最もコーンらしい形状のもの）を緑色の四角で囲む
                x, y, w, h, _ = self.detected
                cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # 現在の作業ディレクトリは new_forpwm_end_to_end_test.py がある場所なので、
            # そこから見て log/ フォルダに画像を保存する
            cv2.imwrite("./log/0_input_image.png", self.input_img)      # 入力画像
            cv2.imwrite("./log/1_binarized_mask.png", self.binarized_img) # 二値化後のマスク画像
            cv2.imwrite("./log/2_debug_output.png", debug_frame)        # 検出結果を描画した画像
        except Exception as e:
            print(f"Debug image save error: {e}")
        # ★★★★★ ここまで追加 ★★★★★