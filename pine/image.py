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
from tkinter import NW
from PIL import Image, ImageDraw, ImageTk
try:
    from .marvelmind import MarvelmindHedge
except:
    raise
try:
    from .SpoolMobile import SpoolMobile, Landscape, ResistanceMap
except:
    raise
try:
    from .utils import get_color_dict, get_course_node_data, init_beacon_data, \
        init_node_data, init_resistance_data, init_weight_data, init_garden_axis, course_line
except:
    raise

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
        self.HEAD_HEDGE_ID = int(self.set_def(cfg, 'HEAD_HEDGE_ID', 6))
        self.HEAD_HEDGE_TTY = str(self.set_def(cfg, 'HEAD_HEDGE_TTY', '/dev/ttyACM0'))

        # 後方モバイルビーコン
        self.TAIL_HEDGE_ID = int(self.set_def(cfg, 'TAIL_HEDGE_ID', 5))
        self.TAIL_HEDGE_TTY = str(self.set_def(cfg, 'TAIL_HEDGE_TTY', '/dev/ttyACM1'))

        # 箱庭倉庫俯瞰図サイズ
        self.NUM_OF_GRID_X = int(self.set_def(cfg, 'NUM_OF_GRID_X', 152))
        self.NUM_OF_GRID_Y = int(self.set_def(cfg, 'NUM_OF_GRID_Y', 120))

        # AIへの入力データサイズ
        self.VISION_SIZE_X = int(self.set_def(cfg, 'IMAGE_W', 160))
        self.VISION_SIZE_Y = int(self.set_def(cfg, 'IMAGE_H', 120))
        self.VISION_SIZE_Z = int(self.set_def(cfg, 'IMAGE_DEPTH', 3))
        self.GRID_SIZE = int(self.set_def(cfg, 'GRID_SIZE', 1))

        # フルサイズのビジョンイメージ
        self.BASE_MARGIN = 4  # 基礎エリアのオフセット
        self.LADDER_MARGIN = 8  # ラダーエリアのオフセット
        self.VISION_SCALE = 4  # 入力画面として作成する画像の拡大率（整数倍）。1倍で152×120。

        # テスト表示用のキャンバスの余白
        self.VISION_MARGIN_X = 10
        self.VISION_MARGIN_Y = 10



        self.WAIT_INTERVAL = float(self.set_def(cfg, 'WAIT_INTERVAL', 0.1))

        # 連結されたノード間の重み付け。移動コストによるコース選択判定に利用
        self.WEIGHT_LIST_PATH =  str(self.set_def(cfg, 'WEIGHT_LIST_PATH', '1b1w.txt'))

        # 各ノードの座標データ
        self.NODE_LIST_PATH = str(self.set_def(cfg, 'NODE_LIST_PATH', '1b1n.txt'))

        # landscape（152×120画像）の各画素ごとの走行抵抗値（転がり摩擦係数）
        self.RESISTANCE_LIST_PATH =  str(self.set_def(cfg, 'RESISTANCE_LIST_PATH', 'RRMap2.txt'))

        # Stationary beacon 4基の箱庭座標系における設置位置 (in studs)
        self.BEACON_LIST_PATH =  str(self.set_def(cfg, 'BEACON_LIST_PATH', '1b1b.txt'))

        # vision用の走行抵抗マップ（バックグラウンド）出力パス
        self.VISION_BACKGROUND_PATH =  str(self.set_def(cfg, 'VISION_BACKGROUND_PATH', 'vision_map.jpg'))

        # CourseUtilsクラスのget_course_node_data()の引数course_type値
        self.COURSE_TYPE =  str(self.set_def(cfg, 'COURSE_TYPE', 'INNER_CLOCKWISE'))

        if self.debug:
            print('[DefaultConfig] attr list: {}'.format(dir(self)))

    def set_def(self, cfg, attr, def_value=None):
        if cfg is None or attr is None:
            return def_value
        else:
            return getattr(cfg, attr) if hasattr(cfg, attr) else def_value

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
        self.cfg = DefaultConfig(cfg=cfg, debug=debug)
        self.debug = debug

        self.image = np.zeros((120, 160,3))
        self.garden_axis = init_garden_axis()
        self.head_address = None
        self.tail_address = None
        self.head_position = np.zeros((3, 1))
        self.tail_position = np.zeros((3, 1))
        self.head_distances = None
        self.tail_distances = None

        self.course_node_numbers, self.course_node_numbers_nodes = get_course_node_data(
            course_type=self.cfg.COURSE_TYPE)

        self.resistance_list = init_resistance_data(
            self.cfg.RESISTANCE_LIST_PATH, self.cfg.NUM_OF_GRID_X)
        self.weight_list = init_weight_data(self.cfg.WEIGHT_LIST_PATH)
        self.node_list = init_node_data(self.cfg.NODE_LIST_PATH)
        self.beacon_address, self.beacon_position = init_beacon_data(self.cfg.BEACON_LIST_PATH)

        self.color_list = get_color_dict()

        self.rrmap_vision_img =  ResistanceMap(
            self.cfg.RESISTANCE_LIST_PATH, self.color_list).generateImageFile(self.cfg.VISION_BACKGROUND_PATH)

        self.mobile = SpoolMobile(
            6, 28, 92, 180, 0.05, 0, 0, 0, 0, 0, 0, 10.625, 13.78, 1, 0, -6, 1000, 0, 8, 
            self.resistance_list, 1, 0, 0, 
            self.cfg.NUM_OF_GRID_X, self.cfg.NUM_OF_GRID_Y, self.cfg.GRID_SIZE,
            None, self.cfg.VISION_SCALE)
        self.vision_img_org = Landscape(
            self.rrmap_vision_img, 
            self.weight_list, self.node_list, 
            self.cfg.BASE_MARGIN, self.cfg.LADDER_MARGIN, 1).create_1x1_landscape_vision_img_wide4(
                self.cfg.VISION_MARGIN_X, self.cfg.VISION_MARGIN_Y,
                course_line(self.course_node_numbers_nodes, self.node_list), self.cfg.VISION_SCALE)
        #print('vision org array')
        #print(dk.utils.img_to_arr(self.vision_img_org))
        self.vision = self.vision_img_org.copy()
        #print('vision array')
        #print(dk.utils.img_to_arr(self.vision))
        self.next_vision_cropped_resized = None
        self.image_array = np.zeros((self.cfg.VISION_SIZE_Y, self.cfg.VISION_SIZE_X, self.cfg.VISION_SIZE_Z))
        self.head_hedge = MarvelmindHedge(
            tty=self.cfg.HEAD_HEDGE_TTY,
            recieveUltrasoundRawDataCallback=self.update_head_position)
        self.head_hedge.start()
        self.tail_hedge = MarvelmindHedge(
            tty=self.cfg.TAIL_HEDGE_TTY,
            recieveUltrasoundRawDataCallback=self.update_tail_position)
        self.tail_hedge.start()
        while(self.head_distances is None or self.tail_distances is None):
            if self.debug:
                print('[MapImageCreator] no distances yet, wait {} sec.'.format(str(self.cfg.WAIT_INTERVAL)))
            sleep(self.cfg.WAIT_INTERVAL)
        if self.cfg.HEAD_HEDGE_ID == self.head_address and self.cfg.TAIL_HEDGE_ID == self.tail_address:
            if self.debug:
                print('[MapImageCreator] hedge ids configuration match')
        else:
            if self.debug:
                print('[MapImageCreator] head conf id:{} actual:{}'.format(str(self.cfg.HEAD_HEDGE_ID), str(self.head_address)))
                print('[MapImageCreator] tail conf id:{} actual:{}'.format(str(self.cfg.TAIL_HEDGE_ID), str(self.tail_address)))
            raise ValueError('hedge ids configuration unmatch')
        self.update()
        print('[MapImageCreator] self.next_vision_cropped_resized is {}'.format(type(self.next_vision_cropped_resized)))
        print(dk.utils.img_to_arr(self.next_vision_cropped_resized))
        print('[MapImageCreator] init completed')

    def update_pose(self):
        # angle
        y_length = self.head_position[1, 0] - self.tail_position[1, 0]
        x_length = self.head_position[0, 0] - self.tail_position[0, 0]
        angle = math.degrees(math.atan2(y_length, x_length))
        if self.debug:
            print('[MapImageCreator] atan2:(%f,%f,%f)' % (y_length, x_length, angle))

        if self.debug:
            print("[MapImageCreator] head adr:{:d}".format(self.head_address))
            print(self.head_position)
            print("[MapImageCreator] tail sdr:{:d}".format(self.tail_address))
            print(self.tail_position)

        pos_x = self.head_position[0, 0]
        pos_y = self.head_position[1, 0]
        angle = 90 - angle
        if self.debug:
            print('[MapImageCreator] update pos_x=%f, pos_y=%f, angle=%f' % (pos_x, pos_y, angle))
        return pos_x, pos_y, angle

    def update(self):
        pos_x, pos_y, angle = self.update_pose()
        print('%f,%f,%f' % (pos_x, pos_y, angle))
        if pos_x != np.nan and pos_y != np.nan and angle != np.nan:
            # 切り出していないフルサイズのビジョン画面にエージェントを描画する
            next_vision = self.vision.copy()
            for b in self.mobile.loader:
                b.position[0, 0], b.position[1, 0] = pos_x, pos_y
                b.angle = angle
                b.draw(vision=next_vision)
            #print('next vision array')
            #print(dk.utils.img_to_arr(next_vision))
            # エージェントを描画したフルサイズのビジョン画面をそのときのエージェント位置に合わせて切り出し、入力画像サイズに成形する
            # これがローダーカメラ入力の画像
            self.next_vision_cropped_resized = self.get_torch_view(next_vision, 60, 
                self.mobile.loader[2].position[0, 0],
                self.mobile.loader[2].position[1, 0],
                self.mobile.loader[2].angle,
                self.cfg.VISION_SCALE, self.cfg.VISION_SIZE_X, self.cfg.VISION_SIZE_Y,
                self.cfg.VISION_MARGIN_X, self.cfg.VISION_MARGIN_Y)
            if self.debug:
                print('[MapImageCreator] next_vision_cropped_resized updated')
                print(self.next_vision_cropped_resized)
                print(type(self.next_vision_cropped_resized))
            #f = ByteIO()
            #self.next_vision_cropped_resized.save(f, format='jpeg')
            #bin_next_vision = f.getValue()
            #self.next_vision_cropped_resized = dk.utils.binary_to_img(dk.utils.img_to_binary(self.next_vision_cropped_resized))
            #self.next_vision_cropped_resized.save('next_vision.jpg', quality=100)
            #print('next_vision_cropped_resized')
            #print(self.next_vision_cropped_resized)
            #print(np.array(self.next_vision_cropped_resized)==dk.utils.img_to_arr(self.next_vision_cropped_resized))
            self.next_vision_cropped_resized.save('next_vision_before.jpg', quality=100)
            self.image_array = dk.utils.img_to_arr(self.next_vision_cropped_resized) #.convert('RGB'))
            dk.utils.arr_to_img(self.next_vision_cropped_resized).save('next_vision_after.jpg', quality=100)
            #if self.debug:
            #print('[MapImageCreator] image_array')
            #print(self.image_array.shape)
            #print(self.image_array.dtype)
            #print(dk.utils.arr_to_img(self.image_array)==self.next_vision_cropped_resized)
            #print(self.image_array == np.zeros(self.image_array.shape))
            #print(type(self.image_array))
            # テスト表示画面用に変換する
            #self.next_vision_img = ImageTk.PhotoImage(next_vision_cropped_resized)

            # テスト表示キャンバスに張替
            #w.delete("vision1")
            #w.create_image(self.cfg.VISION_MARGIN_X, self.cfg.VISION_MARGIN_Y, 
            #    image=next_vision_img, anchor=NW, tag="vision1")
            #master.update()
        else:
            if self.debug:
                print('[MapImageCreator] position or angle is nan!')
        

    def update_head_position(self):
        self.head_distances = self.head_hedge.distances()
        self.head_address, self.head_position = self.updatedMobileBeaconPosition(self.head_distances)
        self.head_position[0, 0] = round(self.head_position[0, 0], 1)
        self.head_position[1, 0] = round(self.head_position[1, 0], 1)
        self.head_position[2, 0] = round(self.head_position[2, 0], 1)
        if self.debug:
            print('[Head]')
            print(self.head_distances)

    def update_tail_position(self):
        self.tail_distances = self.tail_hedge.distances()
        self.tail_address, self.tail_position = self.updatedMobileBeaconPosition(self.tail_distances)
        self.tail_position[0, 0] = round(self.tail_position[0, 0], 1)
        self.tail_position[1, 0] = round(self.tail_position[1, 0], 1)
        self.tail_position[2, 0] = round(self.tail_position[2, 0], 1)
        if self.debug:
            print('[Tail]')
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

    def run(self):

        # ToDo
        # marvelmind 2台から最新のセンサデータを取得し
        # PIL Imageオブジェクトを作成、ローカル変数imageへ格納

        # image is PIL image
        if self.debug:
            print('[MapImageCreator] head adr:{}, pos:({}, {}, {})'.format(
                str(self.head_address),
                str(self.head_position[0]), str(self.head_position[1]), str(self.head_position[2])))
            print('[MapImageCreator] tail adr:{}, pos:({}, {}, {})'.format(
                str(self.tail_address),
                str(self.tail_position[0]), str(self.tail_position[1]), str(self.tail_position[2])))
        self.update()
        return self.image_array
    
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



    def get_torch_view(self, overall_view, torch_radius, pos_x, pos_y, rotate_angle, v_scale, view_width, view_height, offset_x, offset_y):
        # mobileの位置を中心として、半径rのアルファチャンネルを作る
        im_a = Image.new("L", overall_view.size, 0)
        draw = ImageDraw.Draw(im_a)
        pos = [
                (pos_x + offset_x) * v_scale - torch_radius, (pos_y + offset_y) * v_scale - torch_radius, 
                (pos_x + offset_x) * v_scale + torch_radius, (pos_y + offset_y) * v_scale + torch_radius
            ]
        # print(pos)
        draw.ellipse(pos, fill=255)
        im_rgba = overall_view.copy()
        im_rgba.putalpha(im_a)
        im_rgba_crop = im_rgba.crop(pos)  # R120
        #　mobileのangleで回転
        rotate_angle = (360 - rotate_angle) % 360 - 180
        img = im_rgba_crop.rotate(rotate_angle)

        # 入力画像サイズ(160, 120)のバックグラウンドに張り付ける
        torch_view = Image.new("RGB", (view_width, view_height), 0)
        torch_view.paste(img, 
            (
                int((view_width - torch_radius * 2) / 2), 
                int((view_height - torch_radius * 2) / 2)
            )
        )

        return torch_view
