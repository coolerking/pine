# -*- coding: utf-8 -*-
"""
Marvelmind 2台のデータを読み込み、
nd.array型(120, 160, 3)形式のイメージ配列を取得する
Donkeycar パーツクラスを提供する。
"""
import donkeycar as dk
import numpy as np
import math
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
        self.beacon_address, self.beacon_position = DefaultConfig.init_beacon_data(
            '1b1b.txt' if cfg is None else str(cfg.DATANAME_BEACON))

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
    def __init__(self, cfg=None, debug=False):
        """
        初期処理
        引数：
            cfg     config.py/myconfig.pyモジュールオブジェクト
            debug   デバッグオプション
        戻り値：
            なし
        """
        # ToDo
        # スレッドを使う場合はここで開始処理をかく
        # スレッド処理でインスタンス変数 image にPIL Imageを書き込む
        self.cfg = DefaultConfig(cfg=cfg)
        self.debug = debug
        self.head_hedge = MarvelmindHedge(
            tty=self.cfg.HEAD_HEDGE_TTY,
            recieveUltrasoundRawDataCallback=self.update_head_position)
        self.head_hedge.start()
        self.tail_hedge = MarvelmindHedge(
            tty=self.cfg.TAIL_HEDGE_TTY,
            recieveUltrasoundRawDataCallback=self.update_tail_position)
        self.tail_hedge.start()
        self.image = np.zeros((120, 160,3))
        self.garden_axis = [0, 0, -1*41]
        self.head_address = None
        self.tail_address = None
        self.head_position = np.zeros((3, 1))
        self.tail_position = np.zeros((3, 1))
        self.head_distances = None
        self.tail_distances = None
        while(self.head_distances is None or self.tail_distances is None):
            if self.debug:
                print('[MapImageCreator] no distances yet, wait {} sec.'.format(str(self.cfg.WAIT_INTERVAL)))
            sleep(self.cfg.WAIT_INTERVAL)
        if self.debug:
            print('[MapImageCreator] init completed')

    def update_head_position(self):
        self.head_distances = self.head_hedge.distances()
        self.head_address, self.head_position = self.updatedMobileBeaconPosition(self.head_distances)
        if self.debug:
            print('[Head]')
            print(self.head_distances)

    def update_tail_position(self):
        self.tail_distances = self.tail_hedge.distances()
        self.tail_address, self.tail_position = self.updatedMobileBeaconPosition(self.tail_distances)
        if self.debug:
            print('[Tail]')
            print(self.tail_distances)

    def run(self):

        # ToDo
        # marvelmind 2台から最新のセンサデータを取得し
        # PIL Imageオブジェクトを作成、ローカル変数imageへ格納

        # image is PIL image
        print('[MapImageCreator] head adr:{}, pos:{}'.format(str(self.head_address), str(self.head_position)))
        print('[MapImageCreator] tail adr:{}, pos:{}'.format(str(self.tail_address), str(self.tail_position)))
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

    def updatedMobileBeaconPosition(self, distances):
        # Chose three of four distance data
        # for i in (1,len(beacon_address)):
        # distances[2*i] = distances[2*i]/8  # in studs
        shortest_distance = math.inf
        for i in range(1, len(self.cfg.beacon_address) + 1):
            if distances[2 * i] < shortest_distance:
                shortest_distance = distances[2 * i]
                excluded_address = distances[2 * i - 1]
        if self.debug:
            print("B{:d}:{:f} excluded".format(excluded_address, shortest_distance))

        # Rearrange the three distances of beacon addresses along clockwise
        tri_distances = []
        tri_beacon_address = []
        tri_beacon_position = []
        for j in range(len(self.cfg.beacon_address)):
            for i in range(1, len(self.cfg.beacon_address) + 1):
                if distances[2 * i - 1] != excluded_address:
                    if distances[2 * i - 1] == int(self.cfg.beacon_address[j]):
                        # print("beacon_address[{:d}] = {:s}".format(j, beacon_address[j]))
                        tri_distances.append(distances[2 * i - 1])
                        # tri_beacon_address.append(distances[2*i - 1])
                        tri_distances.append(distances[2 * i] * 1000 / 8)
                        tri_beacon_address.append(self.cfg.beacon_address[j])
                        # print(beacon_position[j])
                        tri_beacon_position.append([self.cfg.beacon_position[j, 0], self.cfg.beacon_position[j, 1], self.cfg.beacon_position[j, 2]])

        if self.debug:
            print("Multilateration: B{:d}:{:.3f}, B{:d}:{:.3f}, B{:d}:{:.3f}".format(tri_distances[0], tri_distances[1],
                                                                                 tri_distances[2], tri_distances[3],
                                                                                 tri_distances[4], tri_distances[5]))
            print(tri_beacon_address)
            print(tri_beacon_position)

        # Cal
        Leg_01 = math.sqrt((tri_beacon_position[1][0] - tri_beacon_position[0][0]) ** 2 + (
                    tri_beacon_position[1][1] - tri_beacon_position[0][1]) ** 2 + (
                                       tri_beacon_position[1][2] - tri_beacon_position[0][2]) ** 2)
        Leg_02 = math.sqrt((tri_beacon_position[2][0] - tri_beacon_position[0][0]) ** 2 + (
                    tri_beacon_position[2][1] - tri_beacon_position[0][1]) ** 2 + (
                                       tri_beacon_position[2][2] - tri_beacon_position[0][2]) ** 2)
        Leg_12 = math.sqrt((tri_beacon_position[2][0] - tri_beacon_position[1][0]) ** 2 + (
                    tri_beacon_position[2][1] - tri_beacon_position[1][1]) ** 2 + (
                                       tri_beacon_position[2][2] - tri_beacon_position[1][2]) ** 2)
        if self.debug:
            print("Leg_01:{:3f} Leg_02:{:3f} Leg_12:{:3f}".format(Leg_01, Leg_02, Leg_12))

        COS_C0 = (Leg_01 ** 2 + Leg_02 ** 2 - Leg_12 ** 2) / (2 * Leg_01 * Leg_02)
        U = Leg_01
        VX = Leg_02 * COS_C0
        VY = math.sqrt(Leg_02 ** 2 - VX ** 2)
        V = math.sqrt(VX ** 2 + VY ** 2)
        R0 = tri_distances[1]
        R1 = tri_distances[3]
        R2 = tri_distances[5]
        if self.debug:
            print("U:{:f} VX:{:f} VY:{:f} V:{:f} R0:{:f} R1:{:f} R2:{:f}".format(U, VX, VY, V, R0, R1, R2))
        P = []
        PX = (R0 ** 2 - R1 ** 2 + U ** 2) / (2 * U)
        PY = (R0 ** 2 - R2 ** 2 + V ** 2 - 2 * VX * PX) / (VY * 2)
        # PZ cannot calculate when the z-distance is too small. it needs more than 300mm.
        # PZ = math.sqrt(R0**2 - PX**2 - PY**2)
        if R0 ** 2 - PX ** 2 - PY ** 2 <= 0:
            PZ = 100
        else:
            PZ = math.sqrt(R0 ** 2 - PX ** 2 - PY ** 2)
        if self.debug:
            print("PX:{:3f} PY:{:3f} PZ:{:3f}".format(PX, PY, PZ))
        P = [PX, PY, PZ]
        if self.debug:
            print(P)

        # 座標変換
        def make_rot_mat(positions):
            # x = positions[1][0] - positions[0][0]
            # y = positions[1][1] - positions[0][1]

            x = positions[1][0] - positions[0][0]
            y = positions[1][1] - positions[0][1]
            leg = math.sqrt(x ** 2 + y ** 2)
            if self.debug:
                print("x:{:f} y:{:f} leg:{:f} sin(y/leg):{:f} cos(x/leg):{:f}".format(x, y, leg, np.sin(y / leg),
                                                                                  np.cos(x / leg)))
            # 採用されたベースライン（ベクトル）とビーコン座標系のｘ軸（ベクトル）とのなす角度をｚ軸に関して逆回転
            # rot_matrix = [[np.cos(x/leg), np.sin(y/leg), 0], [-1*np.sin(y/leg), np.cos(x/leg), 0], [0, 0, 1]]
            rot_matrix = [[x / leg, y / leg, 0], [-1 * y / leg, x / leg, 0], [0, 0, 1]]
            rot_matrix = np.array(rot_matrix)
            total_rot_matrix = rot_matrix

            return total_rot_matrix

        rotaion_matrix = make_rot_mat(tri_beacon_position)
        if self.debug:
            print(rotaion_matrix)
        baseline_point = np.zeros((3, 1))
        baseline_point[0, 0], baseline_point[1, 0], baseline_point[2, 0] = PX, PY, PZ  # in studs
        if self.debug:
            print(baseline_point)

        # 逆回転
        rotated_baseline_point = rotaion_matrix @ baseline_point
        if self.debug:
            print("逆回転後")
            print(rotated_baseline_point)

        x180_matrix = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
        x180_matrix = np.array(x180_matrix)
        if self.debug:
            print("180 now")
        rotated_baseline_point_180 = x180_matrix @ rotated_baseline_point
        if self.debug:
            print(rotated_baseline_point_180)

        # 平行移動距離:Baseline座標系原点(tri_beacon_position)と箱庭座標系原点()の距離
        Para = np.zeros((3, 1))
        # for i in range(3):
        #    Para[i,0] = garden_axis[i] - tri_beacon_position[0][i]
        # Para[2,0] = 0

        Para[0, 0], Para[1, 0], Para[2, 0] = self.garden_axis[0] - tri_beacon_position[0][0], self.garden_axis[1] - \
                                             tri_beacon_position[0][1], self.garden_axis[2] - tri_beacon_position[0][2]

        # Para[0,0], Para[1,0], Para[2,0] = garden_axis[0] - tri_beacon_position[0][0], tri_beacon_position[0][1] - garden_axis[1], 0
        if self.debug:
            print("平行移動距離")
            print(Para)
        if self.debug:
            print("平行移動後")
        updated_garden_point = np.zeros((3, 1))
        updated_garden_point = rotated_baseline_point_180 - Para
        if self.debug:
            print(updated_garden_point)
        # print("モバイルビーコン{:d}：x:{:f} y:{:f} z:{:f}".format(distances[0], updated_garden_point[0,0], updated_garden_point[1,0],updated_garden_point[2,0]))
        # updated_point_list=[]
        # for i in range(3):
        #    updated_point_list.append(garden_point[i,0])
        return distances[0], updated_garden_point

if __name__ == '__main__':
    V = dk.vehicle.Vehicle()

    V.add(MapImageCreator(), outputs=['cam/image_array'])

    class PrintImage:
        def run(self, image_array):
            print('image_array = {}'.format(str(image_array)))

    #V.add(PrintImage(), inputs=['cam/image_array'])

    V.start(rate_hz=20, max_loop_count=10000)
