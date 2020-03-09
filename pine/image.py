# -*- coding: utf-8 -*-
"""
Marvelmind 2台のデータを読み込み、
nd.array型(120, 160, 3)形式のイメージ配列を取得する
Donkeycar パーツクラスを提供する。
"""
import donkeycar as dk
import numpy as np
from time import sleep
from marvelmind import MarvelmindHedge

class DefaultConfig:
    """
    MapImageCreator関連の設定をカプセル化したクラス。
    config.py/myconfig.py がない(引数cfgを指定しない)場合
    デフォルト値を使用する。
    """
    def __init__(self, cfg=None, debug=False):
        """
        インスタンス変数へ設定値を格納する。
        引数：
            cfg     config.py/myconfig.py を読み込んだオブジェクト
        戻り値：
            なし
        """
        # デバッグオプション
        self.debug = debug
        
        # 前方モバイルビーコン
        self.HEAD_HEDGE_ID = 6 if cfg is None else int(cfg.HEAD_HEDGE_ID) 
        self.HEAD_HEDGE_TTY = '/dev/ttyACM0' if cfg is None else cfg.HEAD_HEDGE_TTY

        # 後方モバイルビーコン
        self.TAIL_HEDGE_ID = 5 if cfg is None else int(cfg.TAIL_HEDGE_ID)
        self.TAIL_HEDGE_TTY = '/dev/ttyACM1' if cfg is None else cfg.TAIL_HEDGE_TTY

        # 箱庭倉庫俯瞰図サイズ
        self.NUM_OF_GRID_X = 152 if cfg is None else int(cfg.NUM_OF_GRID_X)
        self.NUM_OF_GRID_Y = 120 if cfg is None else int(cfg.NUM_OF_GRID_Y)

        # AIへの入力データサイズ
        self.VISION_SIZE_X = 160 if cfg is None else cfg.IMAGE_W
        self.VISION_SIZE_Y = 120 if cfg is None else cfg.IMAGE_H
        self.GRID_SIZE = 1 if cfg is None else cfg.GRID_SIZE

        self.WAIT_INTERVAL = 0.1 if cfg is None else float(cfg.WAIT_INTERVAL)

        # 連結されたノード間の重み付け。移動コストによるコース選択判定に利用す。
        #self.weight_list = DefaultConfig.init_weight_data(
        #    '1b1w.txt' if cfg is None else str(cfg.DATANAME_WEIGHT))
        # 各ノードの座標データ
        #self.node_list = DefaultConfig.init_node_data(
        #    '1b1n.txt' if cfg is None else str(cfg.DATANAME_NODE))
        # landscape（152×120画像）の各画素ごとの走行抵抗値（転がり摩擦係数）
        #self.resistence_list = DefaultConfig.init_resistance_data(
        #    'RRMap2.txt' if cfg is None else str(cfg.DATANAME_RRMAP), self.NUM_OF_GRID_X) 
        # Stationary beacon 4基の箱庭座標系における設置位置 (in studs)
        #self.beacon_address, self.beacon_position = DefaultConfig.init_beacon_data(
        #    '1b1b.txt' if cfg is None else str(cfg.DATANAME_BEACON))

    @staticmethod
    def init_weight_data(dataname):
        """
        ノード間Legの重み付けを読み込む
        引数：
            dataname ノード間Legの重み付けファイルへのパス
        戻り値：
            ノード間Legの重み付けデータ
        """
        f = open(dataname)
        lines = f.readlines()
        datalen = len(lines)
        f.close()
        data = np.zeros((datalen, datalen))
        for j in range (datalen):
            for i in range (datalen):
                sp = lines[i].split()
                data[i,j] = sp[j+1]
        return data

    # ノード座標データを読み込む
    @staticmethod
    def init_node_data(dataname):
        """
        ノード座標データを読み込む
        引数：
            dataname    ノード座標データファイル
        戻り値：
            ノード座標データ
        """
        f = open(dataname)
        lines = f.readlines()
        datalen = len(lines)
        f.close()
        data = np.zeros((datalen, 2))
        for j in range (2):
            for i in range (datalen):
                sp = lines[i].split()
                data[i,j] = sp[j+1]
        return data

    @staticmethod
    def init_resistance_data(dataname, num):
        """
        箱庭倉庫俯瞰図のピクセル毎の走行抵抗（転がり抵抗）を読み込む
        引数：
            dataname    箱庭倉庫俯瞰図のピクセル毎の走行抵抗（転がり抵抗）データファイルへのパス
            num         
        戻り値：
            箱庭倉庫俯瞰図のピクセル毎の走行抵抗（転がり抵抗）データ
        """
        f = open(dataname)
        lines = f.readlines()
        datalen = len(lines)
        f.close()
        data = np.zeros((datalen, num))
        for j in range (num):
            for i in range (datalen):
                sp = lines[i].split()
                data[i,j] = sp[j+1]
        return data

    @staticmethod
    def init_beacon_data(dataname):
        """
        箱庭倉庫に設置されたstationary beacon 4基の箱庭座標系での設置位置座標を読み込む
        引数：
            dataname    箱庭倉庫に設置されたstationary beacon 4基の箱庭座標系での
                        設置位置座標データファイルへのパス
        戻り値：
            箱庭倉庫に設置されたstationary beacon 4基の箱庭座標系での設置位置座標データ
        """
        f = open(dataname)
        lines = f.readlines()
        datalen = len(lines)
        f.close()
        data = np.zeros((datalen, 3))
        data0 = []
        for j in range (3):
            for i in range (datalen):
                sp = lines[i].split()
                if j==0:
                    data0.append(sp[0])
                data[i,j] = sp[j+1]
        return data0, data

class MapImageCreator:
    """
    Marvrelmind ビーコンから取得したセンサデータをもとに
    2D Map 画像配列（nd.array型(120, 160,3)形式）を生成する
    Donkeycar パーツクラス。
    """
    def __init__(self, cfg=None):
        """
        初期処理
        引数：
            なし
        戻り値：
            なし
        """
        # ToDo
        # スレッドを使う場合はここで開始処理をかく
        # スレッド処理でインスタンス変数 image にPIL Imageを書き込む
        self.cfg = DefaultConfig(cfg=cfg)
        self.head_hedge = MarvelmindHedge(
            tty=self.cfg.HEAD_HEDGE_TTY,
            recieveUltrasoundRawDataCallback=self.update_head_position)
        self.head_hedge.start()
        self.tail_hedge = MarvelmindHedge(
            tty=self.cfg.TAIL_HEDGE_TTY,
            recieveUltrasoundRawDataCallback=self.update_tail_position)
        self.tail_hedge.start()
        self.image = np.zeros((120, 160,3))
        sleep(self.cfg.WAIT_INTERVAL)

    def update_head_position(self):
        self.head_distances = self.head_hedge.distances()

    def update_tail_position(self):
        self.head_distances = self.tail_hedge.distances()

    def run_threaded(self):

        #self.image = next_vision_img
        return dk.utils.img_to_arr(self.image)


    def run(self):

        # ToDo
        # marvelmind 2台から最新のセンサデータを取得し
        # PIL Imageオブジェクトを作成、ローカル変数imageへ格納

        # image is PIL image
        return dk.utils.img_to_arr(self.image)
    
    def shutdown(self):
        """
        終了処理
        引数：
            なし
        戻り値：
            なし
        """
        # ToDo
        # スレッドを使っているなら終了処理をここにかく
        self.head_hedge.stop()
        self.tail_hedge.stop()
        sleep(self.cfg.WAIT_INTERVAL)


if __name__ == '__main__':
    V = dk.vehicle.Vehicle()

    V.add(MapImageCreator(), outputs=['cam/image_array'])

    class PrintImage:
        def run(self, image_array):
            print('image_array = {}'.format(str(image_array)))

    V.add(PrintImage(), inputs=['cam/image_array'])

    V.start(rate_hz=20, max_loop_count=10000)
