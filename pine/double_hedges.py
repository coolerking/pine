# -*- coding: utf-8 -*-
"""
Marvelmind Mobile Beacon(V4.9)2台を搭載したロボットから、
Stationary Beacon4台との距離情報を取得し、
三角測量を用いてX-Y座標および方向を取得するDonkeycarパーツを
提供するモジュール。

X-Y座標系：
RATF倉庫の原点は倉庫自体にXマークを貼り付けた側左端、
長辺がX座標（原点から上方向が正）、短辺がY座標（原点から右方向が正）。
なお、使用していないが地面から地下への法線がZ軸正方向となる。
また座標系の単位stadは1stad=8mm（レゴのポッチ間）となっている。

方向：
単位は度、Y座標正方向が0度で右がプラス、左がマイナスとなっている。
"""
import donkeycar as dk
import numpy as np
import math
from time import sleep
try:
    from .marvelmind import MarvelmindHedge
except:
    raise
try:
    from .utils import init_beacon_data, init_garden_axis
except:
    raise

class PoseReader:
    """
    Marvelmind Mobile Beacon(V4.9)2台を搭載したロボットから、
    姿勢情報(X座標,Y座標,方向)を取得するDonkeycarパーツクラス。
    """
    def __init__(self, cfg, debug=False):
        # デバッグオプション
        self.debug = debug
        # config.py/myconfig.py オブジェクト（必須）
        self.cfg = cfg

        # 前方側Mobile Beacon
        self.head_address = None                # ID
        self.head_position = np.zeros((3, 1))   # 姿勢情報
        self.head_distances = None              # Stationary Beaconとの距離情報
        # 後方側Mobuile Beacon
        self.tail_address = None                # ID
        self.tail_position = np.zeros((3, 1))   # 姿勢情報
        self.tail_distances = None              # Stationary Beaconとの距離情報

        # run/run_threaded 戻り値
        self.pos_x, self.pos_y, self.angle = 0, 0, 0

        self.garden_axis = init_garden_axis()

        # 2台のMobile Beaconの初期化（スレッド開始）
        self.init_double_hedges()
    
    def init_double_hedges(self):
        """
        Mobile Beaconから姿勢情報を収集開始する。
        取得情報は各インスタンス変数に格納される。
        引数：
            なし
        戻り値：
            なし
        例外：
            ValueError      cfg設定と異なる場合
        """
        # Stationary Beacon情報の読み込み
        self.beacon_address, self.beacon_position = \
            init_beacon_data(self.cfg.BEACON_LIST_PATH)
        # 前方側Mobuile Beacon読み取り開始
        self.head_hedge = MarvelmindHedge(
            tty=self.cfg.HEAD_HEDGE_TTY,
            recieveUltrasoundRawDataCallback=self.update_head_position)
        self.head_hedge.start()
        # 後方側Mobuile Beaconオブジェクト読み取り開始
        self.tail_hedge = MarvelmindHedge(
            tty=self.cfg.TAIL_HEDGE_TTY,
            recieveUltrasoundRawDataCallback=self.update_tail_position)
        self.tail_hedge.start()

        # 両方のMobile Beaconからdistaces(raw)を取得するまで待機
        count = 0
        while(self.head_distances is None or self.tail_distances is None):
            if self.debug:
                print('[PoseReader] no distances yet, wait {} sec.'.format(
                    str(self.cfg.WAIT_INTERVAL)))
            sleep(self.cfg.WAIT_INTERVAL)
            count += 1
            if count > 20:
                raise TimeoutError('[PoseReader] firts distances data did not arraived, please set agent within marvelmind beacons')
        
        # cfg オブジェクトで指定したIDと異なる場合は例外発生
        if self.cfg.HEAD_HEDGE_ID == self.head_address and \
            self.cfg.TAIL_HEDGE_ID == self.tail_address:
            if self.debug:
                print('[PoseReader] hedge ids configuration match')
        else:
            if self.debug:
                print('[PoseReader] head conf id:{} actual:{}'.format(
                    str(self.cfg.HEAD_HEDGE_ID), str(self.head_address)))
                print('[PoseReader] tail conf id:{} actual:{}'.format(
                    str(self.cfg.TAIL_HEDGE_ID), str(self.tail_address)))
            raise ValueError('[PoseReader] hedge ids configuration unmatch')

    def update_head_position(self):
        """
        Marvelmindオブジェクトのコールバック関数。
        前方側Mobile BeaconからUltra Soundセンサのrawデータ(distances)を取得し、
        ID番号および位置情報を更新する。
        引数：
            なし
        戻り値：
            なし
        """
        self.head_distances = self.head_hedge.distances()
        self.head_address, self.head_position = \
            self.updatedMobileBeaconPosition(self.head_distances)
        self.head_position[0, 0] = round(self.head_position[0, 0], 1)
        self.head_position[1, 0] = round(self.head_position[1, 0], 1)
        self.head_position[2, 0] = round(self.head_position[2, 0], 1)
        if self.debug:
            print('[PoseReader] head distances:')
            print(self.head_distances)

    def update_tail_position(self):
        """
        Marvelmindオブジェクトのコールバック関数。
        後方側Mobile BeaconからUltra Soundセンサのrawデータ(distances)を取得し、
        ID番号および位置情報を更新する。
        引数：
            なし
        戻り値：
            なし
        """
        self.tail_distances = self.tail_hedge.distances()
        self.tail_address, self.tail_position = \
            self.updatedMobileBeaconPosition(self.tail_distances)
        self.tail_position[0, 0] = round(self.tail_position[0, 0], 1)
        self.tail_position[1, 0] = round(self.tail_position[1, 0], 1)
        self.tail_position[2, 0] = round(self.tail_position[2, 0], 1)
        if self.debug:
            print('[PoseReader] tail distances:')
            print(self.tail_distances)
    
    def updatedMobileBeaconPosition(self, distances):
        """
        Mobile Beaconから取得した Ultra Sound raw データ(引数：distances)を
        もとに三角測量をおこない位置情報(X,Y,Z)に変換する。
        引数：
            distances       Mobile Beaconから取得した Ultra Sound raw データ
        戻り値：
            hedge_address   Mobile Beacon ID
            hedge_position  三角測量によって算出したXYZ座標(nd.array型(3,1)形式)
        """
        # Chose three of four distance data
        # for i in (1,len(beacon_address)):
        # distances[2*i] = distances[2*i]/8  # in studs
        shortest_distance = math.inf
        for i in range(1, len(self.beacon_address) + 1):
            if distances[2 * i] < shortest_distance:
                shortest_distance = distances[2 * i]
                excluded_address = distances[2 * i - 1]
        if self.debug:
            print('[PoseReader] B{:d}:{:f} excluded'.format(
                excluded_address, shortest_distance))

        # Rearrange the three distances of beacon addresses along clockwise
        tri_distances = []
        tri_beacon_address = []
        tri_beacon_position = []
        for j in range(len(self.beacon_address)):
            for i in range(1, len(self.beacon_address) + 1):
                if distances[2 * i - 1] != excluded_address:
                    if distances[2 * i - 1] == int(self.beacon_address[j]):
                        # print("beacon_address[{:d}] = {:s}".format(j, beacon_address[j]))
                        tri_distances.append(distances[2 * i - 1])
                        # tri_beacon_address.append(distances[2*i - 1])
                        tri_distances.append(distances[2 * i] * 1000 / 8)
                        tri_beacon_address.append(self.beacon_address[j])
                        # print(beacon_position[j])
                        tri_beacon_position.append(
                            [
                                self.beacon_position[j, 0],
                                self.beacon_position[j, 1],
                                self.beacon_position[j, 2]
                            ])

        if self.debug:
            print('[PoseReader] Multilateration: B{:d}:{:.3f}, B{:d}:{:.3f}, B{:d}:{:.3f}'.format(
                tri_distances[0], tri_distances[1], tri_distances[2], tri_distances[3],
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
            print('[PoseReader] Leg_01:{:3f} Leg_02:{:3f} Leg_12:{:3f}'.format(Leg_01, Leg_02, Leg_12))

        COS_C0 = (Leg_01 ** 2 + Leg_02 ** 2 - Leg_12 ** 2) / (2 * Leg_01 * Leg_02)
        U = Leg_01
        VX = Leg_02 * COS_C0
        VY = math.sqrt(Leg_02 ** 2 - VX ** 2)
        V = math.sqrt(VX ** 2 + VY ** 2)
        R0 = tri_distances[1]
        R1 = tri_distances[3]
        R2 = tri_distances[5]
        if self.debug:
            print('[PoseReader] U:{:f} VX:{:f} VY:{:f} V:{:f} R0:{:f} R1:{:f} R2:{:f}'.format(
                U, VX, VY, V, R0, R1, R2))
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
            print('[PoseReader] PX:{:3f} PY:{:3f} PZ:{:3f}'.format(PX, PY, PZ))
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
                print('[PoseReader] x:{:f} y:{:f} leg:{:f} sin(y/leg):{:f} cos(x/leg):{:f}'.format(
                    x, y, leg, np.sin(y / leg), np.cos(x / leg)))
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
            print('[PoseReader] after back reverse:')
            print(rotated_baseline_point)

        x180_matrix = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
        x180_matrix = np.array(x180_matrix)
        if self.debug:
            print('[PoseReader] 180 now:')
        rotated_baseline_point_180 = x180_matrix @ rotated_baseline_point
        if self.debug:
            print(rotated_baseline_point_180)

        # 平行移動距離:Baseline座標系原点(tri_beacon_position)と箱庭座標系原点()の距離
        Para = np.zeros((3, 1))
        # for i in range(3):
        #    Para[i,0] = garden_axis[i] - tri_beacon_position[0][i]
        # Para[2,0] = 0

        Para[0, 0], Para[1, 0], Para[2, 0] = \
            self.garden_axis[0] - tri_beacon_position[0][0], \
            self.garden_axis[1] -  tri_beacon_position[0][1], \
            self.garden_axis[2] - tri_beacon_position[0][2]

        # Para[0,0], Para[1,0], Para[2,0] = garden_axis[0] - tri_beacon_position[0][0], tri_beacon_position[0][1] - garden_axis[1], 0
        if self.debug:
            print('[PoseReader] translation distance:')
            print(Para)
            print('[PoseReader] after translation:')
        updated_garden_point = np.zeros((3, 1))
        updated_garden_point = rotated_baseline_point_180 - Para
        if self.debug:
            print(updated_garden_point)
        # print("モバイルビーコン{:d}：x:{:f} y:{:f} z:{:f}".format(distances[0], updated_garden_point[0,0], updated_garden_point[1,0],updated_garden_point[2,0]))
        # updated_point_list=[]
        # for i in range(3):
        #    updated_point_list.append(garden_point[i,0])
        return distances[0], updated_garden_point

    def update_pose(self):
        """
        インスタンス変数に格納された両distances値から
        ロボットの座標(X, Y)および方向を取得し、インスタンス変数を更新する。
        引数：
            なし
        戻り値：
            なし
        """
        # angle
        y_length = self.head_position[1, 0] - self.tail_position[1, 0]
        x_length = self.head_position[0, 0] - self.tail_position[0, 0]
        angle = math.degrees(math.atan2(y_length, x_length))
        if self.debug:
            print('[PoseReader] atan2:(%f,%f,%f)' % (y_length, x_length, angle))

        if self.debug:
            print("[PoseReader] head adr:{:d}".format(self.head_address))
            print(self.head_position)
            print("[PoseReader] tail sdr:{:d}".format(self.tail_address))
            print(self.tail_position)

        pos_x = self.head_position[0, 0]
        pos_y = self.head_position[1, 0]
        angle = 90 - angle
        if self.debug:
            print('[PoseReader] update pos_x=%f, pos_y=%f, angle=%f' % (pos_x, pos_y, angle))
        self.pos_x, self.pos_y, self.angle = pos_x, pos_y, angle

    def run(self):
        """
        インスタンス変数に格納された両distances値から
        ロボットの座標(X, Y)および方向を取得・返却する。
        シングルスレッド実行指定時に使用される。
        引数：
            なし
        戻り値：
            pos_x   X座標値(単位:studs)
            pos_y   Y座標値(単位:studs)
            angle   方向(単位:度)
        """
        if self.debug:
            print('[MapImageCreator] head adr:{}, pos:({}, {}, {})'.format(
                str(self.head_address),
                str(self.head_position[0]), str(self.head_position[1]), str(self.head_position[2])))
            print('[MapImageCreator] tail adr:{}, pos:({}, {}, {})'.format(
                str(self.tail_address),
                str(self.tail_position[0]), str(self.tail_position[1]), str(self.tail_position[2])))
        self.update_pose()
        return self.run_threaded()

    def run_threaded(self):
        """
        インスタンス変数上に格納された最新の姿勢情報を取得する。
        マルチスレッド実行指定の場合、run()のかわりに使用される。
        引数：
            なし
        戻り値：
            pos_x   X座標値(単位:studs)
            pos_y   Y座標値(単位:studs)
            angle   方向(単位:度)
        """
        return self.pos_x, self.pos_y, self.angle

    def update(self):
        """
        マルチスレッド実行指定の場合、スレッド開始時に呼び出されるメソッド。
        1/DRIVE_LOOP_HZ 秒ごとに姿勢情報をインスタンス変数へ格納する。
        引数：
            なし
        戻り値：
            なし
        """
        while True:
            self.update_pose()
            sleep((1.0/float(self.cfg.DRIVE_LOOP_HZ)))

    def shutdown(self):
        """
        Marvelmindオブジェクトのスレッドを終了する。
        引数：
            なし
        戻り値：
            なし
        """
        self.head_hedge.stop()
        self.tail_hedge.stop()
        sleep(self.cfg.WAIT_INTERVAL)
