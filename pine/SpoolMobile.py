# -*- coding: utf-8 -*-
"""
2D Map Pillwo(PIL) オブジェクトを取得するためのモジュールを提供する。
また Tkinter canvasオブジェクトを指定した場合は、canvasを更新する。
外部から利用する場合は、以下の情報が必要となる。
* ベースイメージファイル(JPEG)
* エージェントの座標(X, Y)および方向

外部モジュールから使用する場合は、 `SpoolMobileVision` クラスのみをcallすればよい。
"""
#from tkinter import *
from PIL import Image, ImageDraw
import numpy as np
import math
color_list = {0.001: '#E5E5E5', 0.3: '#00CB65', 0.002: '#00984C'}   # to be refactored


class Mobile:
    """
    駆動車の抽象クラス。
    描画実装は円を描く処理となっている。
    """
    def __init__(self, id, x, y, angle, dt, max_voltage, tn_gradient, rv_gradient, rv_intercept, gear_ratio,
                 transmission_efficiency, tread, tread_to_tail, trail, gc_x, gc_y, weight, offset_x, offset_y,
                 rrmap, scale, margin_x, margin_y, num_of_gridx, num_of_gridy, grid_size, landscape, vision_scale):
        """
        2D マップ上のオブジェクトの初期情報をインスタンス変数へ格納する。
        引数：
            id                      Marvelmind モバイルビーコンID
            x                       対象オブジェクトの開始時点のX座標
            y                       対象オブジェクトの開始時点のY座標
            angle                   画像左下を原点右をX軸、上をY軸としたときの角度(単位：度)
            dt                      0.05 Hz
            max_voltage             印加電圧の最大値(V)
            tn_gradient             モーターの公表されているT-N特性の勾配（印加電圧最大時）
            rv_gradient             ギヤ駆動時の勾配：回転数/印加電圧
            rv_intercept            ギヤ駆動時のy切片
            gear_ratio              駆動輪とモーター軸のギア比
            transmission_efficiency ギアの伝達効率
            tread                   左右両輪間の距離
            thread_to_trail         前輪間中心座標から降臨中心までの距離
            gc_x                    重心X座標
            gc_y                    重心Y座標
            weight                  空荷時重量
            offset_x                前輪間中心座標からセンサ計測値のオフセットX座標
            offset_y                前輪間中心座標からセンサ計測値のオフセットY座標
            rrmap                   マップ画像背景
            scale                   画像をスケールアップする場合の倍率
            margin_x                画像左下を原点としたときの上下X座標
            margin_y                画像左下を原点としたときの右左Y座標
            num_of_gridx            グリッド数X軸
            num_of_gridy            グリッド数Y軸
            grid_size               グリッドサイズ
            landscape               ベースイメージ(JPEG)
            vision_scale            画像スケール
        戻り値：
            なし
        """

        # 移動体の識別id
        self.id = id
        # 箱庭座標系における位置とアングル
        self.position = np.zeros((2, 1))
        self.x = x
        self.y = y
        self.position[0,0], self.position[1,0] = x, y # 箱庭座標系における座標
        self.angle = angle # 時計回りに0≦angle＜360
        # 移動体のxy平面上の重心、空荷時の車両重量[gf]
        self.gravity_center = np.zeros((2, 1))
        self.gc_x = gc_x
        self.gc_y = gc_y
        self.gravity_center[0, 0] = gc_x
        self.gravity_center[1, 0] = gc_y
        self.weight = weight # gf
        # print(self.gravity_center)
        self.dt = dt # 経過時間 sec
        # 駆動系パラメーター
        self.max_voltage = max_voltage # 印加電圧の最大値
        self.tn_gradient = tn_gradient # モーターの公表されているT-N特性の勾配（印加電圧最大時）
        self.rv_gradient = rv_gradient # ギヤ駆動時の勾配：回転数/印加電圧
        self.rv_intercept = rv_intercept # ギヤ駆動時のy切片
        self.gear_ratio = gear_ratio # 駆動輪とモーター軸のギア比
        self.transmission_efficiency = transmission_efficiency # ギアの伝達効率
        self.load = 0
        self.voltage = 0
        # タイヤジオメトリー
        self.grounding_point = np.zeros((2, 1))  # 箱庭座標系における接地点座標。前後輪の摩擦係数取得に利用。
        self.grounding_point[0, 0], self.grounding_point[1, 0] = 0, 0  # ディフォルトは箱庭座標系の原点
        # self.polygon_center = np.zeros((2, 1))  # モバイル座標系における図形としての中心
        # self.polygon_center[0, 0], self.polygon_center[1, 0] = 0, 0
        self.rolling_resistance_moment = []  # 荷重1kgf当たりで発生する転がり抵抗による負荷トルク　mNm/kgf
        self.starting_resistance_moment = []  # 荷重1kgf当たりで発生する始動抵抗による負荷トルク　mNm/kgf
        self.tread = tread  # トレッド幅　フォークリフトなら10.5studs(84mmm)
        self.tread_to_tail = tread_to_tail  # 駆動輪中間点からキャスター軸までの距離　13.75
        self.trail = trail  # トレール
        # 初期値
        self.caster_angle = 0 # v_left/v_rightから導出される尾輪の方向
        self.tail_wheel_angle = 0 # 実際の尾輪の方向（実際の方向は一般的にcaster_angleとは異なる）
        self.tail_mounting_position = np.zeros((2, 1))
        self.tail_mounting_position[0, 0], self.tail_mounting_position[1, 0] = 0, -13.78  # キャスター回転中心の座標
        self.tail_mount_distance = np.linalg.norm(self.tail_mounting_position)
        self.rc = math.inf # 前輪（駆動輪）の中間点の旋回半径
        self.v = 0 # 前輪（駆動輪）の中間点の速度（車台速度）
        self.avw = 0 # 前輪（駆動輪）の中間点の回転角速度
        self.direction = 0 # 0:forward 1:backward

        # 背景および表示スケール等描画属性
        self.rrmap = rrmap
        self.scale = scale
        self.margin_x = margin_x
        self.margin_y = margin_y
        self.num_of_gridx = num_of_gridx
        self.num_of_gridy = num_of_gridy
        self.grid_size = grid_size
        self.landscape = landscape
        self.vision_scale = vision_scale

        # ローダーの位置座標（センサー値）とローダー図形（前車軸中心）のオフセット
        self.center_offset = np.zeros((2, 1))
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.center_offset[0, 0], self.center_offset[1, 0] = self.offset_x, self.offset_y  # in studs

    def init_for_replay(self, x, y, angle, gc_x, gc_y):
        """
        再現時の初期位置、初期状態にインスタンス変数を変更する。
        引数：
            x       X座標
            y       Y座標
            angle   方向
            gc_x    
            gc_y
        戻り値：
            なし
        """
        # 箱庭座標系における位置とアングル
        self.position = np.zeros((2, 1))
        self.x = x
        self.y = y
        self.position[0, 0], self.position[1, 0] = x, y  # 箱庭座標系における座標
        self.angle = angle  # 時計回りに0≦angle＜360
        # 移動体のxy平面上の重心、空荷時の車両重量[gf]
        self.gravity_center = np.zeros((2, 1))
        self.gc_x = gc_x
        self.gc_y = gc_y
        self.gravity_center[0, 0] = gc_x
        self.gravity_center[1, 0] = gc_y
        # 駆動系パラメーター
        self.load = 0
        self.voltage = 0
        # タイヤジオメトリー
        self.grounding_point[0, 0], self.grounding_point[1, 0] = 0, 0  # ディフォルトは箱庭座標系の原点
        # 初期値
        self.caster_angle = 0  # v_left/v_rightから導出される尾輪の方向
        self.tail_wheel_angle = 0  # 実際の尾輪の方向（実際の方向は一般的にcaster_angleとは異なる）
        self.tail_mounting_position = np.zeros((2, 1))
        self.tail_mounting_position[0, 0], self.tail_mounting_position[1, 0] = 0, -13.78  # キャスター回転中心の座標
        self.tail_mount_distance = np.linalg.norm(self.tail_mounting_position)
        self.rc = math.inf  # 前輪（駆動輪）の中間点の旋回半径
        self.v = 0  # 前輪（駆動輪）の中間点の速度（車台速度）
        self.avw = 0  # 前輪（駆動輪）の中間点の回転角速度
        self.direction = 0  # 0:forward 1:backward

    def move(self, v_left, v_right, canvas=None, vision=None): # 左右駆動輪の速度を入力とする
        if canvas is not None or vision is not None:
            # 回転中心からの半径rc、その角速度avw、台車中心の速度v
            self.avw = (v_right - v_left) / self.tread # 左旋回が正
            self.v = (v_right + v_left) / 2

            if self.avw==0:
                self.rc = math.inf
            else:
                self.rc = (v_right + v_left) * (self.tread / 2) / (v_right - v_left) # 左旋回が正

            # dt後のangle値　左旋回（反時計回り）が増加
            # self.angle = self.angle + math.degrees(self.avw * self.dt)
            # self.angle = np.mod(self.angle + math.degrees(self.avw * self.dt), 360)
            self.angle = (self.angle + math.degrees(self.avw * self.dt)) % 360
            # print('Mobile v = %f v_left = %f v_right = %f angle = %f' % (self.v, v_left, v_right, self.angle))

            # dt後の移動距離 movement(dx, dy)
            movement = np.zeros((2, 1))
            movement[0, 0], movement[1, 0] = self.v * self.dt * math.sin(math.radians(self.avw * self.dt)), self.v * self.dt * math.cos(math.radians(self.avw * self.dt))

            # Mobileの向きangle方向への移動を箱庭座標系での移動距離に変換する
            # print('angle = %d' % self.angle)
            rot_matrix = [[math.cos(math.radians(self.angle)), math.sin(math.radians(self.angle))],
                          [-1 * math.sin(math.radians(self.angle)), math.cos(math.radians(self.angle))]]
            rot_matrix = np.array(rot_matrix)
            rotated_movement = rot_matrix@movement
            # 移動体を次の位置に平行移動
            self.position = self.position + rotated_movement
            # print('Mobile moved to (%f, %f)' % (self.position[0, 0], self.position[1, 0]))
            self.draw(canvas, vision)

    def draw(self, canvas=None, vision=None):
        # 回転中心の円とID
        # シミュレータ画面、ビジョン画面共通で描画する
        radius = 2
        # radius_outer_limit = 16.78
        radius_outer_limit = 7

        # ビジョン画面用の進行方向矢線ポリゴンの座標
        # エージェントに見せるビジョン画像におけるエージェントの振る舞いを誇張するために追加
        #polygon = np.zeros((2, 11))
        #polygon[0, 0], polygon[1, 0] = 0, 16.78
        #polygon[0, 1], polygon[1, 1] = 2.0, 14.78
        #polygon[0, 2], polygon[1, 2] = 1.0, 14.78
        #polygon[0, 3], polygon[1, 3] = 1.0, 1.0
        #polygon[0, 4], polygon[1, 4] = 16.78, 1.0
        #polygon[0, 5], polygon[1, 5] = 16.78, -1.0
        #polygon[0, 6], polygon[1, 6] = -16.78, -1.0
        #polygon[0, 7], polygon[1, 7] = -16.78, 1.0
        #polygon[0, 8], polygon[1, 8] = -1.0, 1.0
        #polygon[0, 9], polygon[1, 9] = -1.0, 14.78
        #polygon[0, 10], polygon[1, 10] = -2.0, 14.78

        polygon = np.zeros((2, 7))
        polygon[0, 0], polygon[1, 0] = 0, 11.78
        polygon[0, 1], polygon[1, 1] = 2.0, 9.78
        polygon[0, 2], polygon[1, 2] = 1.0, 9.78
        polygon[0, 3], polygon[1, 3] = 1.0, 0.0
        polygon[0, 4], polygon[1, 4] = -1.0, 0.0
        polygon[0, 5], polygon[1, 5] = -1.0, 9.78
        polygon[0, 6], polygon[1, 6] = -2.0, 9.78

        polygon = polygon + self.center_offset

        #polygon = np.zeros((2, 11))
        #polygon[0, 0], polygon[1, 0] = 0, 16.78
        #polygon[0, 1], polygon[1, 1] = 2.0, 14.78
        #polygon[0, 2], polygon[1, 2] = 1.0, 14.78
        #polygon[0, 3], polygon[1, 3] = 1.0, 1.0
        #polygon[0, 4], polygon[1, 4] = 5.0, 1.0
        #polygon[0, 5], polygon[1, 5] = 5.0, -1.0
        #polygon[0, 6], polygon[1, 6] = -5.0, -1.0
        #polygon[0, 7], polygon[1, 7] = -5.0, 1.0
        #polygon[0, 8], polygon[1, 8] = -1.0, 1.0
        #polygon[0, 9], polygon[1, 9] = -1.0, 14.78
        #polygon[0, 10], polygon[1, 10] = -2.0, 14.78

        # print('ビジョンサイズ（%d, %d）' % (vision.size[0], vision.size[1]))
        # -angleだけ回転する
        rot_matrix = [[math.cos(math.radians(self.angle)), math.sin(math.radians(self.angle))],
                      [-1 * math.sin(math.radians(self.angle)), math.cos(math.radians(self.angle))]]
        rot_matrix = np.array(rot_matrix)
        # rotated_chassis = np.zeros((2, 4))
        rotated_polygon = rot_matrix @ polygon
        # self.position[]だけ平行移動
        rotated_polygon = rotated_polygon + self.position

        center = np.zeros((2, 1))
        center = center + self.center_offset
        rotated_center = rot_matrix @ center
        rotated_center = rotated_center + self.position

        if canvas is not None:
            mobile_radius = radius * self.scale
            canvas.delete(str(self.id) + "oval")
            #canvas.create_oval(self.position[0, 0] * self.scale + self.margin_x - mobile_radius,
            #                   self.position[1, 0] * self.scale + self.margin_y - mobile_radius,
            #                   self.position[0, 0] * self.scale + self.margin_x + mobile_radius,
            #                   self.position[1, 0] * self.scale + self.margin_y + mobile_radius,
            #                   outline="#323232", fill="#00FFFF", tag=str(self.id) + "oval")

            canvas.create_oval(rotated_center[0, 0] * self.scale + self.margin_x - mobile_radius,
                               rotated_center[1, 0] * self.scale + self.margin_y - mobile_radius,
                               rotated_center[0, 0] * self.scale + self.margin_x + mobile_radius,
                               rotated_center[1, 0] * self.scale + self.margin_y + mobile_radius,
                               outline="#323232", fill="#00FFFF", tag=str(self.id) + "oval")

        if vision is not None:
            mobile_radius = radius * self.vision_scale
            draw = ImageDraw.Draw(vision)
            frame_offset_x = (vision.size[0] - self.num_of_gridx * self.vision_scale) / 2
            frame_offset_y = (vision.size[1] - self.num_of_gridy * self.vision_scale) / 2
            # print('frame_offset(%f,%f)' % (frame_offset_x, frame_offset_y))
            #draw.ellipse((self.position[0, 0] * self.vision_scale + self.margin_x + frame_offset_x - mobile_radius,
            #              self.position[1, 0] * self.vision_scale + self.margin_y + frame_offset_y - mobile_radius,
            #              self.position[0, 0] * self.vision_scale + self.margin_x + frame_offset_x + mobile_radius,
            #              self.position[1, 0] * self.vision_scale + self.margin_y + frame_offset_y + mobile_radius),
            #             fill=(0, 255, 255), outline=(50, 50, 50))
            draw.ellipse((rotated_center[0, 0] * self.vision_scale + self.margin_x + frame_offset_x - mobile_radius,
                          rotated_center[1, 0] * self.vision_scale + self.margin_y + frame_offset_y - mobile_radius,
                          rotated_center[0, 0] * self.vision_scale + self.margin_x + frame_offset_x + mobile_radius,
                          rotated_center[1, 0] * self.vision_scale + self.margin_y + frame_offset_y + mobile_radius),
                         fill=(0, 255, 255), outline=(50, 50, 50))
            # 赤い円
            mobile_radius = radius_outer_limit * self.vision_scale
            #draw.ellipse((self.position[0, 0] * self.vision_scale + self.margin_x + frame_offset_x - mobile_radius,
            #              self.position[1, 0] * self.vision_scale + self.margin_y + frame_offset_y - mobile_radius,
            #              self.position[0, 0] * self.vision_scale + self.margin_x + frame_offset_x + mobile_radius,
            #              self.position[1, 0] * self.vision_scale + self.margin_y + frame_offset_y + mobile_radius),
            #             outline=(255, 0, 0))
            draw.ellipse((rotated_center[0, 0] * self.vision_scale + self.margin_x + frame_offset_x - mobile_radius,
                          rotated_center[1, 0] * self.vision_scale + self.margin_y + frame_offset_y - mobile_radius,
                          rotated_center[0, 0] * self.vision_scale + self.margin_x + frame_offset_x + mobile_radius,
                          rotated_center[1, 0] * self.vision_scale + self.margin_y + frame_offset_y + mobile_radius),
                         outline=(255, 0, 0))

            # 短い矢印
            draw.polygon(((rotated_polygon[0, 0] * self.vision_scale + self.margin_x + frame_offset_x,
                           rotated_polygon[1, 0] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 1] * self.vision_scale + self.margin_x + frame_offset_x,
                           rotated_polygon[1, 1] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 2] * self.vision_scale + self.margin_x + frame_offset_x,
                           rotated_polygon[1, 2] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 3] * self.vision_scale + self.margin_x + frame_offset_x,
                           rotated_polygon[1, 3] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 4] * self.vision_scale + self.margin_x + frame_offset_x,
                           rotated_polygon[1, 4] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 5] * self.vision_scale + self.margin_x + frame_offset_x,
                           rotated_polygon[1, 5] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 6] * self.vision_scale + self.margin_x + frame_offset_x,
                           rotated_polygon[1, 6] * self.vision_scale + self.margin_y + frame_offset_y)),
                         fill=(255, 0, 0), outline=(76, 76, 76))

    def getRegistance(self, load):
        # 接地部品ではオーバーライドされ、非接地部品では呼ばれない前提
        # print('super.getRegistance now')
        self.load = load
        registance = 0
        return registance

    def getCenter(self):
        # 接地部品ではオーバーライドされ、非接地部品では呼ばれない前提
        print('super.getCenter called!!')
        center = self.position
        return center

    def getVelocity(self, rate, load):
        # 接地部品ではオーバーライドされ、非接地部品では呼ばれない前提
        # print('super.getVelocity now')
        #self.voltage = voltage
        #self.load = load
        velocity = 0
        return velocity


class Chassis(Mobile):
    def __init__(self, id, x, y, angle, dt, max_voltage, tn_gradient, rv_gradient, rv_intercept, gear_ratio, transmission_efficiency, tread, tread_to_tail, trail, gc_x, gc_y, weight,  offset_x, offset_y, rrmap, scale, margin_x, margin_y, num_of_gridx, num_of_gridy, grid_size, landscape, vision_scale):
        super().__init__(id, x, y, angle, dt, max_voltage, tn_gradient, rv_gradient, rv_intercept, gear_ratio, transmission_efficiency, tread, tread_to_tail, trail, gc_x, gc_y, weight,  offset_x, offset_y, rrmap, scale, margin_x, margin_y, num_of_gridx, num_of_gridy, grid_size, landscape, vision_scale)
        # シャーシの座標。箱庭座標系においてangle0度のときの重心（self.position[]）を原点とする座標
        self.polygon = np.zeros((2, 6))
        self.polygon[0, 0], self.polygon[1, 0] = 5.0, 2.5
        self.polygon[0, 1], self.polygon[1, 1] = 5.0, -13.03
        self.polygon[0, 2], self.polygon[1, 2] = 2.0, -15.03
        self.polygon[0, 3], self.polygon[1, 3] = -2.0, -15.03
        self.polygon[0, 4], self.polygon[1, 4] = -5.0, -13.03
        self.polygon[0, 5], self.polygon[1, 5] = -5.0, 2.5

        self.polygon = self.polygon + self.center_offset

    def draw(self, canvas=None, vision=None):
        # print('ビジョンサイズ（%d, %d）' % (vision.size[0], vision.size[1]))
        # -angleだけ回転する
        rot_matrix = [[math.cos(math.radians(self.angle)), math.sin(math.radians(self.angle))],
                      [-1 * math.sin(math.radians(self.angle)), math.cos(math.radians(self.angle))]]
        rot_matrix = np.array(rot_matrix)
        # rotated_chassis = np.zeros((2, 4))
        rotated_polygon = rot_matrix@self.polygon
        # self.position[]だけ平行移動
        rotated_polygon = rotated_polygon + self.position

        if canvas is not None:
            canvas.delete(str(self.id) + "chassis")
            canvas.create_polygon(rotated_polygon[0, 0] * self.scale + self.margin_x, rotated_polygon[1, 0] * self.scale + self.margin_y,
                                  rotated_polygon[0, 1] * self.scale + self.margin_x, rotated_polygon[1, 1] * self.scale + self.margin_y,
                                  rotated_polygon[0, 2] * self.scale + self.margin_x, rotated_polygon[1, 2] * self.scale + self.margin_y,
                                  rotated_polygon[0, 3] * self.scale + self.margin_x, rotated_polygon[1, 3] * self.scale + self.margin_y,
                                  rotated_polygon[0, 4] * self.scale + self.margin_x, rotated_polygon[1, 4] * self.scale + self.margin_y,
                                  rotated_polygon[0, 5] * self.scale + self.margin_x, rotated_polygon[1, 5] * self.scale + self.margin_y,
                                  outline="#4C4C4C", fill="#FFBF00", tag=str(self.id) + "chassis")
        if vision is not None:
            draw = ImageDraw.Draw(vision)
            frame_offset_x = (vision.size[0] - self.num_of_gridx * self.vision_scale) / 2
            frame_offset_y = (vision.size[1] - self.num_of_gridy * self.vision_scale) / 2
            draw.polygon(((rotated_polygon[0, 0] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 0] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 1] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 1] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 2] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 2] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 3] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 3] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 4] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 4] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 5] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 5] * self.vision_scale + self.margin_y + frame_offset_y)),
                         fill=(255, 191, 0), outline=(76, 76, 76))


class Folk(Mobile):
    def __init__(self, id, x, y, angle, dt, max_voltage, tn_gradient, rv_gradient, rv_intercept, gear_ratio,
                 transmission_efficiency, tread, tread_to_tail, trail, gc_x, gc_y, weight, offset_x, offset_y, rrmap, scale, margin_x,
                 margin_y, num_of_gridx, num_of_gridy, grid_size, landscape, vision_scale):
        super().__init__(id, x, y, angle, dt, max_voltage, tn_gradient, rv_gradient, rv_intercept, gear_ratio,
                         transmission_efficiency, tread, tread_to_tail, trail, gc_x, gc_y, weight, offset_x, offset_y, rrmap, scale,
                         margin_x, margin_y, num_of_gridx, num_of_gridy, grid_size, landscape, vision_scale)
        # フォークの座標。箱庭座標系においてangle0度のときの重心（self.position[]）を原点とする座標
        self.polygon = np.zeros((2, 12))
        self.polygon[0, 0], self.polygon[1, 0] = -5.625, 3.5
        self.polygon[0, 1], self.polygon[1, 1] = -5.625, 3.875
        self.polygon[0, 2], self.polygon[1, 2] = -5.25, 3.875
        self.polygon[0, 3], self.polygon[1, 3] = -5.0, 11.25
        self.polygon[0, 4], self.polygon[1, 4] = -3.75, 11.25
        self.polygon[0, 5], self.polygon[1, 5] = -3.75, 3.875
        self.polygon[0, 6], self.polygon[1, 6] = 3.75, 3.875
        self.polygon[0, 7], self.polygon[1, 7] = 3.75, 11.25
        self.polygon[0, 8], self.polygon[1, 8] = 5.0, 11.25
        self.polygon[0, 9], self.polygon[1, 9] = 5.25, 3.875
        self.polygon[0, 10], self.polygon[1, 10] = 5.625, 3.875
        self.polygon[0, 11], self.polygon[1, 11] = 5.625, 3.5

        self.polygon = self.polygon + self.center_offset

    def draw(self, canvas=None, vision=None):
        # -angleだけ回転する
        rot_matrix = [[math.cos(math.radians(self.angle)), math.sin(math.radians(self.angle))],
                      [-1 * math.sin(math.radians(self.angle)), math.cos(math.radians(self.angle))]]
        rot_matrix = np.array(rot_matrix)
        rotated_polygon = rot_matrix@self.polygon
        # self.position[]だけ平行移動
        rotated_polygon = rotated_polygon + self.position

        if canvas is not None:
            canvas.delete(str(self.id) + "folk")
            canvas.create_polygon(rotated_polygon[0, 0] * self.scale + self.margin_x, rotated_polygon[1, 0] * self.scale + self.margin_y,
                                  rotated_polygon[0, 1] * self.scale + self.margin_x, rotated_polygon[1, 1] * self.scale + self.margin_y,
                                  rotated_polygon[0, 2] * self.scale + self.margin_x, rotated_polygon[1, 2] * self.scale + self.margin_y,
                                  rotated_polygon[0, 3] * self.scale + self.margin_x, rotated_polygon[1, 3] * self.scale + self.margin_y,
                                  rotated_polygon[0, 4] * self.scale + self.margin_x, rotated_polygon[1, 4] * self.scale + self.margin_y,
                                  rotated_polygon[0, 5] * self.scale + self.margin_x, rotated_polygon[1, 5] * self.scale + self.margin_y,
                                  rotated_polygon[0, 6] * self.scale + self.margin_x, rotated_polygon[1, 6] * self.scale + self.margin_y,
                                  rotated_polygon[0, 7] * self.scale + self.margin_x, rotated_polygon[1, 7] * self.scale + self.margin_y,
                                  rotated_polygon[0, 8] * self.scale + self.margin_x, rotated_polygon[1, 8] * self.scale + self.margin_y,
                                  rotated_polygon[0, 9] * self.scale + self.margin_x, rotated_polygon[1, 9] * self.scale + self.margin_y,
                                  rotated_polygon[0, 10] * self.scale + self.margin_x, rotated_polygon[1, 10] * self.scale + self.margin_y,
                                  rotated_polygon[0, 11] * self.scale + self.margin_x, rotated_polygon[1, 11] * self.scale + self.margin_y,
                                  outline="#4C4C4C", fill="#CCFFFF", tag=str(self.id) + "folk")
        if vision is not None:
            draw = ImageDraw.Draw(vision)
            frame_offset_x = (vision.size[0] - self.num_of_gridx * self.vision_scale) / 2
            frame_offset_y = (vision.size[1] - self.num_of_gridy * self.vision_scale) / 2
            draw.polygon(((rotated_polygon[0, 0] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 0] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 1] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 1] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 2] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 2] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 3] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 3] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 4] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 4] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 5] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 5] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 6] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 6] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 7] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 7] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 8] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 8] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 9] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 9] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 10] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 10] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 11] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 11] * self.vision_scale + self.margin_y + frame_offset_y)),
                         fill=(204, 255, 255), outline=(76, 76, 76))


class Left_wheel(Mobile):
    def __init__(self, id, x, y, angle, dt, max_voltage, tn_gradient, rv_gradient, rv_intercept, gear_ratio, transmission_efficiency, tread, tread_to_tail, trail, gc_x, gc_y, weight,  offset_x, offset_y, rrmap, scale, margin_x, margin_y, num_of_gridx, num_of_gridy, grid_size, landscape, vision_scale):
        super().__init__(id, x, y, angle, dt, max_voltage, tn_gradient, rv_gradient, rv_intercept, gear_ratio, transmission_efficiency, tread, tread_to_tail, trail, gc_x, gc_y, weight,  offset_x, offset_y, rrmap, scale, margin_x, margin_y, num_of_gridx, num_of_gridy, grid_size, landscape, vision_scale)
        # 左駆動輪の座標。箱庭座標系においてangle0度のときの重心（self.position[]）を原点とする座標
        self.polygon = np.zeros((2, 4))
        self.polygon[0, 0], self.polygon[1, 0] = 6.3125, 2.5
        self.polygon[0, 1], self.polygon[1, 1] = 4.3125, 2.5
        self.polygon[0, 2], self.polygon[1, 2] = 4.3125, -2.5
        self.polygon[0, 3], self.polygon[1, 3] = 6.3125, -2.5
        self.polygon = self.polygon + self.center_offset
        # 左駆動輪の図形としての中心。接地点として負荷の分配計算に使用
        self.center = np.zeros((2, 1))
        self.center[0, 0] = (self.polygon[0, 0] + self.polygon[0, 1] + self.polygon[0, 2] + self.polygon[0, 3]) / 4
        self.center[1, 0] = (self.polygon[1, 0] + self.polygon[1, 1] + self.polygon[1, 2] + self.polygon[1, 3]) / 4
        self.polygon = np.append(self.polygon, self.center, axis=1)
        self.radius = (self.polygon[1, 1] - self.polygon[1, 2]) / 2
        # 箱庭座標系での初期状態（ポリゴンの座標と接地点の座標）
        self.grounding_point = np.zeros((2, 1))
        # -angleだけ回転する
        rot_matrix = [[math.cos(math.radians(self.angle)), math.sin(math.radians(self.angle))],
                      [-1 * math.sin(math.radians(self.angle)), math.cos(math.radians(self.angle))]]
        rot_matrix = np.array(rot_matrix)
        rotated_polygon = rot_matrix @ self.polygon
        # self.position[]だけ平行移動
        rotated_polygon = rotated_polygon + self.position
        self.grounding_point[0, 0], self.grounding_point[1, 0] = rotated_polygon[0, -1], rotated_polygon[1, -1]

    def getRegistance(self, load):
        # 路面と接して車両の荷重(load [gf])を受けるポイント（基本的に平面図の重心座標）のトルク負荷[mN・m]
        # 現状では駆動輪と従動輪のみが路面に接して発生する負荷トルク＝走行抵抗（転がり抵抗）×タイヤ半径のみを返す
        if math.isnan(self.grounding_point[0, 0]) or math.isnan(self.grounding_point[1, 0]):
            print('nan detected')
            mu_x, mu_y = 0, 0
        else:
            mu_x = int(self.grounding_point[0, 0])
            mu_y = int(self.grounding_point[1, 0])

        if mu_x < 0:
            mu_x = 0
        if mu_x > self.num_of_gridx - 1:
            mu_x = self.num_of_gridx - 1
        if mu_y < 0:
            mu_y = 0
        if mu_y > self.num_of_gridy - 1:
            mu_y = self.num_of_gridy - 1
        mu = self.rrmap[mu_y, mu_x]

        resistance = mu * load * 9.8 / 1000  # mN
        return resistance

    def getVelocity(self, rate, load):
        # 駆動輪を駆動するモーターの性能特性から入力された印加電圧とそのときの負荷トルクから
        # 速度（回転数×タイヤ半径）[stud/sec]を求める
        velocity = 2 * 3.14 * self.radius * (self.rv_gradient * abs(rate * self.max_voltage) - self.rv_intercept - self.tn_gradient * load * self.radius / (self.gear_ratio * self.transmission_efficiency)) / (self.gear_ratio * 60)
        # print('Left wheel velocity = %f' % velocity)
        if velocity < 0:
            velocity = 0
        return velocity

    def draw(self, canvas=None, vision=None):
        # -angleだけ回転する
        rot_matrix = [[math.cos(math.radians(self.angle)), math.sin(math.radians(self.angle))],
                      [-1 * math.sin(math.radians(self.angle)), math.cos(math.radians(self.angle))]]
        rot_matrix = np.array(rot_matrix)
        # rotated_chassis = np.zeros((2, 4))
        rotated_polygon = rot_matrix@self.polygon
        # self.position[]だけ平行移動
        rotated_polygon = rotated_polygon + self.position
        self.grounding_point[0, 0], self.grounding_point[1, 0] = rotated_polygon[0, -1], rotated_polygon[1, -1]

        if canvas is not None:
            canvas.delete(str(self.id) + "left_wheel")
            canvas.create_polygon(rotated_polygon[0, 0] * self.scale + self.margin_x, rotated_polygon[1, 0] * self.scale + self.margin_y,
                                  rotated_polygon[0, 1] * self.scale + self.margin_x, rotated_polygon[1, 1] * self.scale + self.margin_y,
                                  rotated_polygon[0, 2] * self.scale + self.margin_x, rotated_polygon[1, 2] * self.scale + self.margin_y,
                                  rotated_polygon[0, 3] * self.scale + self.margin_x, rotated_polygon[1, 3] * self.scale + self.margin_y,
                                  outline="#4C4C4C", fill="#656565", tag=str(self.id) + "left_wheel")

        if vision is not None:
            draw = ImageDraw.Draw(vision)
            frame_offset_x = (vision.size[0] - self.num_of_gridx * self.vision_scale) / 2
            frame_offset_y = (vision.size[1] - self.num_of_gridy * self.vision_scale) / 2
            draw.polygon(((rotated_polygon[0, 0] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 0] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 1] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 1] * self.vision_scale + self.margin_y + frame_offset_y),
                         (rotated_polygon[0, 2] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 2] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 3] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 3] * self.vision_scale + self.margin_y + frame_offset_y)),
                         fill=(101, 101, 101), outline=(76, 76, 76))


class Right_wheel(Mobile):
    def __init__(self, id, x, y, angle, dt, max_voltage, tn_gradient, rv_gradient, rv_intercept, gear_ratio, transmission_efficiency, tread, tread_to_tail, trail, gc_x, gc_y, weight,  offset_x, offset_y, rrmap, scale, margin_x, margin_y, num_of_gridx, num_of_gridy, grid_size, landscape, vision_scale):
        super().__init__(id, x, y, angle, dt, max_voltage, tn_gradient, rv_gradient, rv_intercept, gear_ratio, transmission_efficiency, tread, tread_to_tail, trail, gc_x, gc_y, weight,  offset_x, offset_y, rrmap, scale, margin_x, margin_y, num_of_gridx, num_of_gridy, grid_size, landscape, vision_scale)
        # 右駆動輪の座標。箱庭座標系においてangle0度のときの重心（self.position[]）を原点とする座標
        self.polygon = np.zeros((2, 4))
        self.polygon[0, 0], self.polygon[1, 0] = -4.3125, 2.5
        self.polygon[0, 1], self.polygon[1, 1] = -6.3125, 2.5
        self.polygon[0, 2], self.polygon[1, 2] = -6.3125, -2.5
        self.polygon[0, 3], self.polygon[1, 3] = -4.3125, -2.5
        self.polygon = self.polygon + self.center_offset
        # 右駆動輪の図形としての中心。接地点として負荷の分配計算に使用
        self.center = np.zeros((2, 1))
        self.center[0, 0] = (self.polygon[0, 0] + self.polygon[0, 1] + self.polygon[0, 2] + self.polygon[0, 3]) / 4
        self.center[1, 0] = (self.polygon[1, 0] + self.polygon[1, 1] + self.polygon[1, 2] + self.polygon[1, 3]) / 4
        self.polygon = np.append(self.polygon, self.center, axis=1)
        self.radius = (self.polygon[1, 1] - self.polygon[1, 2]) / 2
        # 箱庭座標系での初期状態（ポリゴンの座標と接地点の座標）
        self.grounding_point = np.zeros((2, 1))
        # -angleだけ回転する
        rot_matrix = [[math.cos(math.radians(self.angle)), math.sin(math.radians(self.angle))],
                      [-1 * math.sin(math.radians(self.angle)), math.cos(math.radians(self.angle))]]
        rot_matrix = np.array(rot_matrix)
        rotated_polygon = rot_matrix @ self.polygon
        # self.position[]だけ平行移動
        rotated_polygon = rotated_polygon + self.position
        self.grounding_point[0, 0], self.grounding_point[1, 0] = rotated_polygon[0, -1], rotated_polygon[1, -1]

    def getRegistance(self, load):
        # 路面と接して車両の荷重(load [gf])を受けるポイント（基本的に平面図の重心座標）のトルク負荷[mN・m]
        # 現状では駆動輪と従動輪のみが路面に接して発生する負荷トルク＝走行抵抗（転がり抵抗）×タイヤ半径のみを返す
        # print('grounding point Right (%f, %f)' % (self.grounding_point[0, 0], self.grounding_point[1, 0]))
        if math.isnan(self.grounding_point[0, 0]) or math.isnan(self.grounding_point[1, 0]):
            print('nan detected')
            mu_x, mu_y = 0, 0
        else:
            mu_x = int(self.grounding_point[0, 0])
            mu_y = int(self.grounding_point[1, 0])

        if mu_x < 0:
            mu_x = 0
        if mu_x > self.num_of_gridx - 1:
            mu_x = self.num_of_gridx - 1
        if mu_y < 0:
            mu_y = 0
        if mu_y > self.num_of_gridy - 1:
            mu_y = self.num_of_gridy - 1
        mu = self.rrmap[mu_y, mu_x]

        resistance = mu * load * 9.8 / 1000  # mN
        return resistance

    def getVelocity(self, rate, load):
        # 駆動輪を駆動するモーターの性能特性から入力された印加電圧とそのときの負荷トルクから
        # 速度（回転数×タイヤ半径）[stud/sec]を求める
        torque = load * self.radius / (self.gear_ratio * self.transmission_efficiency)  # mNm
        rpm = self.rv_gradient * abs(rate * self.max_voltage) - self.rv_intercept
        velocity = 2 * 3.14 * self.radius * (rpm - self.tn_gradient * torque) / (self.gear_ratio * 60)
        if velocity < 0:
            velocity = 0
        return velocity

    def draw(self, canvas=None, vision=None):
        # -angleだけ回転する
        rot_matrix = [[math.cos(math.radians(self.angle)), math.sin(math.radians(self.angle))],
                      [-1 * math.sin(math.radians(self.angle)), math.cos(math.radians(self.angle))]]
        rot_matrix = np.array(rot_matrix)
        rotated_polygon = rot_matrix @ self.polygon
        # self.position[]だけ平行移動
        rotated_polygon = rotated_polygon + self.position
        self.grounding_point[0, 0], self.grounding_point[1, 0] = rotated_polygon[0, -1], rotated_polygon[1, -1]

        if canvas is not None:
            canvas.delete(str(self.id) + "right_wheel")
            canvas.create_polygon(rotated_polygon[0, 0] * self.scale + self.margin_x, rotated_polygon[1, 0] * self.scale + self.margin_y,
                                  rotated_polygon[0, 1] * self.scale + self.margin_x, rotated_polygon[1, 1] * self.scale + self.margin_y,
                                  rotated_polygon[0, 2] * self.scale + self.margin_x, rotated_polygon[1, 2] * self.scale + self.margin_y,
                                  rotated_polygon[0, 3] * self.scale + self.margin_x, rotated_polygon[1, 3] * self.scale + self.margin_y,
                                  outline="#4C4C4C", fill="#656565", tag=str(self.id) + "right_wheel")

        if vision is not None:
            draw = ImageDraw.Draw(vision)
            frame_offset_x = (vision.size[0] - self.num_of_gridx * self.vision_scale) / 2
            frame_offset_y = (vision.size[1] - self.num_of_gridy * self.vision_scale) / 2
            draw.polygon(((rotated_polygon[0, 0] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 0] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 1] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 1] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 2] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 2] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 3] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 3] * self.vision_scale + self.margin_y + frame_offset_y)),
                         fill=(101, 101, 101), outline=(76, 76, 76))

class Tail_wheel(Mobile):
    def __init__(self, id, x, y, angle, dt, max_voltage, tn_gradient, rv_gradient, rv_intercept, gear_ratio, transmission_efficiency, tread, tread_to_tail, trail, gc_x, gc_y, weight,  offset_x, offset_y, rrmap, scale, margin_x, margin_y, num_of_gridx, num_of_gridy, grid_size, landscape, vision_scale):
        super().__init__(id, x, y, angle, dt, max_voltage, tn_gradient, rv_gradient, rv_intercept, gear_ratio, transmission_efficiency, tread, tread_to_tail, trail, gc_x, gc_y, weight,  offset_x, offset_y, rrmap, scale, margin_x, margin_y, num_of_gridx, num_of_gridy, grid_size, landscape, vision_scale)
        # 尾輪（従動輪）の座標。箱庭座標系においてangle0度のときの重心（self.position[]）を原点とする座標
        self.polygon = np.zeros((2, 4))
        self.polygon[0, 0], self.polygon[1, 0] = 1.0, -12.78
        self.polygon[0, 1], self.polygon[1, 1] = 1.0, -16.78
        self.polygon[0, 2], self.polygon[1, 2] = -1.0, -16.78
        self.polygon[0, 3], self.polygon[1, 3] = -1.0, -12.78
        self.polygon = self.polygon + self.center_offset
        # 尾輪（従動輪）の図形としての中心。接地点として負荷の分配計算に使用
        self.center = np.zeros((2, 1))
        self.center[0, 0] = (self.polygon[0, 0] + self.polygon[0, 1] + self.polygon[0, 2] + self.polygon[0, 3]) / 4
        self.center[1, 0] = (self.polygon[1, 0] + self.polygon[1, 1] + self.polygon[1, 2] + self.polygon[1, 3]) / 4
        self.polygon = np.append(self.polygon, self.center, axis=1)
        self.radius = (self.polygon[1, 0] - self.polygon[1, 1]) / 2
        # 箱庭座標系での初期状態（ポリゴンの座標と接地点の座標）
        self.grounding_point = np.zeros((2, 1))
        # 初期キャスターアングルの回転
        rot_matrix_t = [
            [math.cos(math.radians(self.tail_wheel_angle)), -1 * math.sin(math.radians(self.tail_wheel_angle))],
            [math.sin(math.radians(self.tail_wheel_angle)), math.cos(math.radians(self.tail_wheel_angle))]]
        rot_matrix_t = np.array(rot_matrix_t)
        rotated_polygon_t = rot_matrix_t @ (self.polygon - self.tail_mounting_position) + self.tail_mounting_position
        # 初期状態での尾輪の図形としての中心座標
        self.center[0, 0], self.center[1, 0] = rotated_polygon_t[0, -1], rotated_polygon_t[1, -1]
        # さらに回転・移動して箱庭座標系での初期状態（ポリゴンの座標と接地点の座標）とする
        # -angleだけ回転する
        rot_matrix = [[math.cos(math.radians(self.angle)), math.sin(math.radians(self.angle))],
                      [-1 * math.sin(math.radians(self.angle)), math.cos(math.radians(self.angle))]]
        rot_matrix = np.array(rot_matrix)
        rotated_polygon = rot_matrix @ rotated_polygon_t
        # self.position[]だけ平行移動
        rotated_polygon = rotated_polygon + self.position
        self.grounding_point[0, 0], self.grounding_point[1, 0] = rotated_polygon[0, -1], rotated_polygon[1, -1]

    def getRegistance(self, load):
        # 路面と接して車両の荷重(load [gf])を受けるポイント（基本的に平面図の重心座標）のトルク負荷[mN・m]
        # 現状では駆動輪と従動輪のみが路面に接して発生する負荷トルク＝走行抵抗（転がり抵抗）×タイヤ半径のみを返す

        # comment out for test
        if math.isnan(self.grounding_point[0, 0]) or math.isnan(self.grounding_point[1, 0]):
            print('nan detected')
            mu_x, mu_y = 0, 0
        else:
            mu_x = int(self.grounding_point[0, 0])
            mu_y = int(self.grounding_point[1, 0])

        if mu_x < 0:
            mu_x = 0
        if mu_x > self.num_of_gridx - 1:
            mu_x = self.num_of_gridx - 1
        if mu_y < 0:
            mu_y = 0
        if mu_y > self.num_of_gridy - 1:
            mu_y = self.num_of_gridy - 1
        mu = self.rrmap[mu_y, mu_x]

        resistance = mu * load * 9.8 / 1000  # mN
        return resistance

    def draw(self, canvas=None, vision=None):
        # tailのみ自分の中心に関して回転
        # 自由に首振りをするtail wheelを想定する。キャスター無しの1軸

        rot_matrix_t = [[math.cos(math.radians(self.tail_wheel_angle)), -1 * math.sin(math.radians(self.tail_wheel_angle))],
                     [math.sin(math.radians(self.tail_wheel_angle)), math.cos(math.radians(self.tail_wheel_angle))]]
        rot_matrix_t = np.array(rot_matrix_t)
        rotated_polygon_t = rot_matrix_t@(self.polygon - self.tail_mounting_position) + self.tail_mounting_position

        # -angleだけ回転する
        rot_matrix = [[math.cos(math.radians(self.angle)), math.sin(math.radians(self.angle))],
                      [-1 * math.sin(math.radians(self.angle)), math.cos(math.radians(self.angle))]]
        rot_matrix = np.array(rot_matrix)
        rotated_polygon = rot_matrix @ rotated_polygon_t

        # self.position[]だけ平行移動
        rotated_polygon = rotated_polygon + self.position
        self.grounding_point[0, 0], self.grounding_point[1, 0] = rotated_polygon[0, -1], rotated_polygon[1, -1]
        # print('Tail wheel grounding at (%f, %f)' % (self.grounding_point[0, 0], self.grounding_point[1, 0]))

        if canvas is not None:
            canvas.delete(str(self.id) + "tail_wheel")
            canvas.create_polygon(rotated_polygon[0, 0] * self.scale + self.margin_x, rotated_polygon[1, 0] * self.scale + self.margin_y,
                                  rotated_polygon[0, 1] * self.scale + self.margin_x, rotated_polygon[1, 1] * self.scale + self.margin_y,
                                  rotated_polygon[0, 2] * self.scale + self.margin_x, rotated_polygon[1, 2] * self.scale + self.margin_y,
                                  rotated_polygon[0, 3] * self.scale + self.margin_x, rotated_polygon[1, 3] * self.scale + self.margin_y,
                                  outline="#4C4C4C", fill="#656565", tag=str(self.id) + "tail_wheel")

        if vision is not None:
            draw = ImageDraw.Draw(vision)
            frame_offset_x = (vision.size[0] - self.num_of_gridx * self.vision_scale) / 2
            frame_offset_y = (vision.size[1] - self.num_of_gridy * self.vision_scale) / 2
            draw.polygon(((rotated_polygon[0, 0] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 0] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 1] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 1] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 2] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 2] * self.vision_scale + self.margin_y + frame_offset_y),
                          (rotated_polygon[0, 3] * self.vision_scale + self.margin_x + frame_offset_x, rotated_polygon[1, 3] * self.vision_scale + self.margin_y + frame_offset_y)),
                         fill=(101, 101, 101), outline=(76, 76, 76))


class ResistanceMap:

    def __init__(self, inputMapFile, colorMapList):
        self.inputMapFile = inputMapFile
        self.colorMapList = colorMapList

    def init_resistance_data(self):
        f = open(self.inputMapFile)
        lines = f.readlines()
        datalen = len(lines)
        datanum = len(lines[0].split()) - 1
        f.close()
        data = np.zeros((datalen, datanum, 3))
        for j in range (datanum):
            for i in range (datalen):
                sp = lines[i].split()
                # c = color_list.get(float(sp[j+1]))
                c = self.colorMapList.get(float(sp[j+1]))
                data[i, j, 0] = int(c[1:3], 16)
                data[i, j, 1] = int(c[3:5], 16)
                data[i, j, 2] = int(c[5:7], 16)
        return data

    def generateImageFile(self, outfilepath):
        resistance_list = ResistanceMap.init_resistance_data(self)
        pil_img = Image.fromarray(np.uint8(resistance_list))
        pil_img.save(outfilepath)
        return pil_img


class Landscape:
    def __init__(self, rrmap_img, weight_list, node_list, base_margin, ladder_margin, scale):
        self.rrmap_img = rrmap_img # 走行抵抗イメージ
        self.weight_list = weight_list
        self.node_list = node_list
        self.base_margin = base_margin
        self.ladder_margin = ladder_margin
        self.scale = scale

    def create_1x1_landscape_img(self, margin_x, margin_y):
        print('I an SpoolMobile.')
        if margin_x != 0 or margin_y != 0:
            result_img = Image.new(self.rrmap_img.mode, (self.rrmap_img.size[0] + margin_x * 2, self.rrmap_img.size[1] + margin_y * 2), (255, 255, 255))
            result_img.paste(self.rrmap_img, (margin_x, margin_y))
            self.rrmap_img = result_img

        if self.scale != 1:
            self.rrmap_img = self.rrmap_img.resize((self.rrmap_img.size[0] * self.scale, self.rrmap_img.size[1] * self.scale), Image.LANCZOS)

        draw = ImageDraw.Draw(self.rrmap_img)

        # draw nodes and legs on the resistance map
        node_circle_radius = 2 * self.scale
        display_offset_x = (margin_x + self.base_margin + self.ladder_margin) * self.scale
        display_offset_y = (margin_y + self.base_margin + self.ladder_margin) * self.scale

        node_list_d = []
        for i in range(len(self.node_list)):
            node_list_d.append([self.node_list[i, 0] * self.scale + display_offset_x, self.node_list[i, 1] * self.scale + display_offset_y])

        for j in range(len(self.weight_list)):
            for i in range(len(self.weight_list)):
                if self.weight_list[i, j] != 0:
                    draw.line(((node_list_d[i][0], node_list_d[i][1]), (node_list_d[j][0], node_list_d[j][1])),
                              fill=(0, 0, 203), width=1 * self.scale)

        for i in range(len(node_list_d)):
            draw.ellipse((node_list_d[i][0] - node_circle_radius, node_list_d[i][1] - node_circle_radius,
                          node_list_d[i][0] + node_circle_radius, node_list_d[i][1] + node_circle_radius),
                         fill=(0, 0, 203), outline=(0, 0, 203))

        return self.rrmap_img

    def create_1x1_landscape_1_img(self, margin_x, margin_y, scale):
        if margin_x != 0 or margin_y != 0:
            result_img = Image.new(self.rrmap_img.mode, (self.rrmap_img.size[0] + margin_x * 2, self.rrmap_img.size[1] + margin_y * 2), (255, 255, 255))
            result_img.paste(self.rrmap_img, (margin_x, margin_y))
            self.rrmap_img = result_img

        if scale != 1:
            self.rrmap_img = self.rrmap_img.resize((self.rrmap_img.size[0] * scale, self.rrmap_img.size[1] * scale), Image.LANCZOS)

        draw = ImageDraw.Draw(self.rrmap_img)

        # draw nodes and legs on the resistance map
        node_circle_radius = 2 * scale
        display_offset_x = (margin_x + self.base_margin + self.ladder_margin) * scale
        display_offset_y = (margin_y + self.base_margin + self.ladder_margin) * scale

        # node_list_d = []
        node_list = np.array([[28,92],[28,28],[124,28],[124,92],[28,92]])
        magic_number = 12
        node_list_d = []
        for i in range(len(node_list)):
            node_list_d.append([(node_list[i, 0] - magic_number) * scale + display_offset_x, (node_list[i, 1] - magic_number) * scale + display_offset_y])

        for i in range(len(node_list_d) - 1):
            draw.line(((node_list_d[i][0], node_list_d[i][1]), (node_list_d[i + 1][0], node_list_d[i + 1][1])), fill=(0, 0, 203), width=1 * scale)

        for i in range(len(node_list_d) - 1):
            draw.ellipse((node_list_d[i][0] - node_circle_radius, node_list_d[i][1] - node_circle_radius,
                          node_list_d[i][0] + node_circle_radius, node_list_d[i][1] + node_circle_radius),
                         fill=(0, 0, 203), outline=(0, 0, 203))

        return self.rrmap_img

    def create_1x1_landscape_2_img(self, margin_x, margin_y, scale):
        if margin_x != 0 or margin_y != 0:
            result_img = Image.new(self.rrmap_img.mode, (self.rrmap_img.size[0] + margin_x * 2, self.rrmap_img.size[1] + margin_y * 2), (255, 255, 255))
            result_img.paste(self.rrmap_img, (margin_x, margin_y))
            self.rrmap_img = result_img

        if scale != 1:
            self.rrmap_img = self.rrmap_img.resize((self.rrmap_img.size[0] * scale, self.rrmap_img.size[1] * scale), Image.LANCZOS)

        draw = ImageDraw.Draw(self.rrmap_img)

        # draw nodes and legs on the resistance map
        node_circle_radius = 2 * scale
        display_offset_x = (margin_x + self.base_margin + self.ladder_margin) * scale
        display_offset_y = (margin_y + self.base_margin + self.ladder_margin) * scale

        return self.rrmap_img

    def create_1x1_landscape_3_img(self, margin_x, margin_y, scale):
        if margin_x != 0 or margin_y != 0:
            result_img = Image.new(self.rrmap_img.mode, (self.rrmap_img.size[0] + margin_x * 2, self.rrmap_img.size[1] + margin_y * 2), (255, 255, 255))
            result_img.paste(self.rrmap_img, (margin_x, margin_y))
            self.rrmap_img = result_img

        if scale != 1:
            self.rrmap_img = self.rrmap_img.resize((self.rrmap_img.size[0] * scale, self.rrmap_img.size[1] * scale), Image.LANCZOS)

        draw = ImageDraw.Draw(self.rrmap_img)

        # draw nodes and legs on the resistance map
        node_circle_radius = 2 * scale
        display_offset_x = (margin_x + self.base_margin + self.ladder_margin) * scale
        display_offset_y = (margin_y + self.base_margin + self.ladder_margin) * scale

        # node_list_d = []
        node_list = np.array([[20, 100], [20, 20], [132, 20], [132, 100], [20, 100]])
        magic_number = 12
        node_list_d = []
        for i in range(len(node_list)):
            node_list_d.append([(node_list[i, 0] - magic_number) * scale + display_offset_x, (node_list[i, 1] - magic_number) * scale + display_offset_y])

        for i in range(len(node_list_d) - 1):
            draw.line(((node_list_d[i][0], node_list_d[i][1]), (node_list_d[i + 1][0], node_list_d[i + 1][1])), fill=(0, 0, 203), width=1 * scale)

        for i in range(len(node_list_d) - 1):
            draw.ellipse((node_list_d[i][0] - node_circle_radius, node_list_d[i][1] - node_circle_radius,
                          node_list_d[i][0] + node_circle_radius, node_list_d[i][1] + node_circle_radius),
                         fill=(0, 0, 203), outline=(0, 0, 203))

        return self.rrmap_img

    def create_1x1_landscape_4_img(self, margin_x, margin_y, scale):
        if margin_x != 0 or margin_y != 0:
            result_img = Image.new(self.rrmap_img.mode, (self.rrmap_img.size[0] + margin_x * 2, self.rrmap_img.size[1] + margin_y * 2), (255, 255, 255))
            result_img.paste(self.rrmap_img, (margin_x, margin_y))
            self.rrmap_img = result_img

        if scale != 1:
            self.rrmap_img = self.rrmap_img.resize((self.rrmap_img.size[0] * scale, self.rrmap_img.size[1] * scale), Image.LANCZOS)

        draw = ImageDraw.Draw(self.rrmap_img)

        # draw nodes and legs on the resistance map
        node_circle_radius = 2 * scale
        display_offset_x = (margin_x + self.base_margin + self.ladder_margin) * scale
        display_offset_y = (margin_y + self.base_margin + self.ladder_margin) * scale

        # node_list_d = []
        node_list = np.array([[12, 108], [12, 12], [140, 12], [140, 108], [12, 108]])
        magic_number = 12
        node_list_d = []
        for i in range(len(node_list)):
            node_list_d.append([(node_list[i, 0] - magic_number) * scale + display_offset_x, (node_list[i, 1] - magic_number) * scale + display_offset_y])

        for i in range(len(node_list_d) - 1):
            draw.line(((node_list_d[i][0], node_list_d[i][1]), (node_list_d[i + 1][0], node_list_d[i + 1][1])), fill=(0, 0, 203), width=1 * scale)

        for i in range(len(node_list_d) - 1):
            draw.ellipse((node_list_d[i][0] - node_circle_radius, node_list_d[i][1] - node_circle_radius,
                          node_list_d[i][0] + node_circle_radius, node_list_d[i][1] + node_circle_radius),
                         fill=(0, 0, 203), outline=(0, 0, 203))

        return self.rrmap_img

    def create_1x1_landscape_vision_img(self, margin_x, margin_y, node_list=None, scale=1):
        print('I am SpoolMobile_v.')
        if margin_x != 0 or margin_y != 0:
            result_img = Image.new(self.rrmap_img.mode, (self.rrmap_img.size[0] + margin_x * 2, self.rrmap_img.size[1] + margin_y * 2), (255, 255, 255))
            result_img.paste(self.rrmap_img, (margin_x, margin_y))
            self.rrmap_img = result_img

        if scale != 1:
            self.rrmap_img = self.rrmap_img.resize((self.rrmap_img.size[0] * scale, self.rrmap_img.size[1] * scale), Image.LANCZOS)

        draw = ImageDraw.Draw(self.rrmap_img)

        #if np.isnan(node_list) is True:
            # draw nodes and legs on the resistance map
        node_circle_radius = 2 * scale
        display_offset_x = (margin_x + self.base_margin + self.ladder_margin) * scale
        display_offset_y = (margin_y + self.base_margin + self.ladder_margin) * scale

        # print(np.isnan(node_list))
        if node_list is not None:
            print('course is given')
            # node_list_d = []
            # node_list = np.array([[12, 108], [12, 12], [140, 12], [140, 108], [12, 108]])
            magic_number = 12
            node_list_d = []
            for i in range(len(node_list)):
                node_list_d.append([(node_list[i, 0] - magic_number) * scale + display_offset_x, (node_list[i, 1] - magic_number) * scale + display_offset_y])

            for i in range(len(node_list_d) - 1):
                draw.line(((node_list_d[i][0], node_list_d[i][1]), (node_list_d[i + 1][0], node_list_d[i + 1][1])), fill=(0, 0, 203), width= 1 * scale)

            for i in range(len(node_list_d) - 1):
                draw.ellipse((node_list_d[i][0] - node_circle_radius, node_list_d[i][1] - node_circle_radius,
                              node_list_d[i][0] + node_circle_radius, node_list_d[i][1] + node_circle_radius),
                             fill=(0, 0, 203), outline=(0, 0, 203))

        return self.rrmap_img

    def create_1x1_landscape_vision_img_wide(self, margin_x, margin_y, node_list=None, scale=1):
        if margin_x != 0 or margin_y != 0:
            result_img = Image.new(self.rrmap_img.mode, (self.rrmap_img.size[0] + margin_x * 2, self.rrmap_img.size[1] + margin_y * 2), (255, 255, 255))
            result_img.paste(self.rrmap_img, (margin_x, margin_y))
            self.rrmap_img = result_img

        if scale != 1:
            self.rrmap_img = self.rrmap_img.resize((self.rrmap_img.size[0] * scale, self.rrmap_img.size[1] * scale), Image.LANCZOS)

        draw = ImageDraw.Draw(self.rrmap_img)

        #if np.isnan(node_list) is True:
            # draw nodes and legs on the resistance map
        node_circle_radius = (14 / 2) * scale
        display_offset_x = (margin_x + self.base_margin + self.ladder_margin) * scale
        display_offset_y = (margin_y + self.base_margin + self.ladder_margin) * scale

        # print(np.isnan(node_list))
        if node_list is not None:
            print('course is given')
            # node_list_d = []
            # node_list = np.array([[12, 108], [12, 12], [140, 12], [140, 108], [12, 108]])
            magic_number = 12
            node_list_d = []
            for i in range(len(node_list)):
                node_list_d.append([(node_list[i, 0] - magic_number) * scale + display_offset_x, (node_list[i, 1] - magic_number) * scale + display_offset_y])

            for i in range(len(node_list_d) - 1):
                draw.line(((node_list_d[i][0], node_list_d[i][1]), (node_list_d[i + 1][0], node_list_d[i + 1][1])), fill=(0, 0, 203), width= 14 * scale)

            for i in range(len(node_list_d) - 1):
                draw.ellipse((node_list_d[i][0] - node_circle_radius, node_list_d[i][1] - node_circle_radius,
                              node_list_d[i][0] + node_circle_radius, node_list_d[i][1] + node_circle_radius),
                             fill=(0, 0, 203), outline=(0, 0, 203))

        return self.rrmap_img

    def create_1x1_landscape_vision_img_wide2(self, margin_x, margin_y, node_list=None, scale=1):
        if margin_x != 0 or margin_y != 0:
            result_img = Image.new(self.rrmap_img.mode, (self.rrmap_img.size[0] + margin_x * 2, self.rrmap_img.size[1] + margin_y * 2), (255, 255, 255))
            result_img.paste(self.rrmap_img, (margin_x, margin_y))
            self.rrmap_img = result_img

        if scale != 1:
            self.rrmap_img = self.rrmap_img.resize((self.rrmap_img.size[0] * scale, self.rrmap_img.size[1] * scale), Image.LANCZOS)

        draw = ImageDraw.Draw(self.rrmap_img)

        #if np.isnan(node_list) is True:
            # draw nodes and legs on the resistance map
        node_circle_radius = (14 / 2) * scale
        display_offset_x = (margin_x + self.base_margin + self.ladder_margin) * scale
        display_offset_y = (margin_y + self.base_margin + self.ladder_margin) * scale

        # print(np.isnan(node_list))
        if node_list is not None:
            print('course is given')
            # node_list_d = []
            # node_list = np.array([[12, 108], [12, 12], [140, 12], [140, 108], [12, 108]])
            magic_number = 12
            node_list_d = []
            for i in range(len(node_list)):
                node_list_d.append([(node_list[i, 0] - magic_number) * scale + display_offset_x, (node_list[i, 1] - magic_number) * scale + display_offset_y])

            for i in range(len(node_list_d) - 1):
                draw.line(((node_list_d[i][0], node_list_d[i][1]), (node_list_d[i + 1][0], node_list_d[i + 1][1])), fill=(204,255,255), width= 14 * scale)

            for i in range(len(node_list_d) - 1):
                draw.ellipse((node_list_d[i][0] - node_circle_radius, node_list_d[i][1] - node_circle_radius,
                              node_list_d[i][0] + node_circle_radius, node_list_d[i][1] + node_circle_radius),
                             fill=(204,255,255), outline=(204,255,255))

        return self.rrmap_img

    def create_1x1_landscape_vision_img_wide3(self, margin_x, margin_y, node_list=None, scale=1):  #Road with center line
        if margin_x != 0 or margin_y != 0:
            result_img = Image.new(self.rrmap_img.mode, (self.rrmap_img.size[0] + margin_x * 2, self.rrmap_img.size[1] + margin_y * 2), (255, 255, 255))
            result_img.paste(self.rrmap_img, (margin_x, margin_y))
            self.rrmap_img = result_img

        if scale != 1:
            self.rrmap_img = self.rrmap_img.resize((self.rrmap_img.size[0] * scale, self.rrmap_img.size[1] * scale), Image.LANCZOS)

        draw = ImageDraw.Draw(self.rrmap_img)

        #if np.isnan(node_list) is True:
            # draw nodes and legs on the resistance map
        node_circle_radius = (14 / 2) * scale
        display_offset_x = (margin_x + self.base_margin + self.ladder_margin) * scale
        display_offset_y = (margin_y + self.base_margin + self.ladder_margin) * scale

        # print(np.isnan(node_list))
        if node_list is not None:
            print('course is given')
            # node_list_d = []
            # node_list = np.array([[12, 108], [12, 12], [140, 12], [140, 108], [12, 108]])
            magic_number = 12
            node_list_d = []
            for i in range(len(node_list)):
                node_list_d.append([(node_list[i, 0] - magic_number) * scale + display_offset_x, (node_list[i, 1] - magic_number) * scale + display_offset_y])

            for i in range(len(node_list_d) - 1):
                draw.line(((node_list_d[i][0], node_list_d[i][1]), (node_list_d[i + 1][0], node_list_d[i + 1][1])), fill=(204,255,255), width= 14 * scale)

            for i in range(len(node_list_d) - 1):
                draw.ellipse((node_list_d[i][0] - node_circle_radius, node_list_d[i][1] - node_circle_radius,
                              node_list_d[i][0] + node_circle_radius, node_list_d[i][1] + node_circle_radius),
                             fill=(204,255,255), outline=(204,255,255))

            for i in range(len(node_list_d) - 1):
                draw.line(((node_list_d[i][0], node_list_d[i][1]), (node_list_d[i + 1][0], node_list_d[i + 1][1])), fill=(0, 0, 203), width= 1 * scale)

            node_circle_radius = 2 * scale
            for i in range(len(node_list_d) - 1):
                draw.ellipse((node_list_d[i][0] - node_circle_radius, node_list_d[i][1] - node_circle_radius,
                              node_list_d[i][0] + node_circle_radius, node_list_d[i][1] + node_circle_radius),
                             fill=(0, 0, 203), outline=(0, 0, 203))

        return self.rrmap_img

    def create_1x1_landscape_vision_img_wide4(self, margin_x, margin_y, node_list=None, scale=1):  #Road with center line
        if margin_x != 0 or margin_y != 0:
            result_img = Image.new(self.rrmap_img.mode,
                                   (self.rrmap_img.size[0] + margin_x * 2, self.rrmap_img.size[1] + margin_y * 2),
                                   (0, 152, 76))
            # result_img = Image.new(self.rrmap_img.mode, (self.rrmap_img.size[0] + margin_x * 2, self.rrmap_img.size[1] + margin_y * 2), (255, 255, 255))
            result_img.paste(self.rrmap_img, (margin_x, margin_y))
            self.rrmap_img = result_img

        if scale != 1:
            self.rrmap_img = self.rrmap_img.resize((self.rrmap_img.size[0] * scale, self.rrmap_img.size[1] * scale), Image.LANCZOS)

        draw = ImageDraw.Draw(self.rrmap_img)

        #if np.isnan(node_list) is True:
            # draw nodes and legs on the resistance map
        node_circle_radius = (14 / 2) * scale
        display_offset_x = (margin_x + self.base_margin + self.ladder_margin) * scale
        display_offset_y = (margin_y + self.base_margin + self.ladder_margin) * scale

        # print(np.isnan(node_list))
        if node_list is not None:
            print('course is given')
            # node_list_d = []
            # node_list = np.array([[12, 108], [12, 12], [140, 12], [140, 108], [12, 108]])
            magic_number = 12
            node_list_d = []
            for i in range(len(node_list)):
                node_list_d.append([(node_list[i, 0] - magic_number) * scale + display_offset_x, (node_list[i, 1] - magic_number) * scale + display_offset_y])

            for i in range(len(node_list_d) - 1):
                draw.line(((node_list_d[i][0], node_list_d[i][1]), (node_list_d[i + 1][0], node_list_d[i + 1][1])), fill=(204,255,255), width= 14 * scale)

            for i in range(len(node_list_d) - 1):
                draw.ellipse((node_list_d[i][0] - node_circle_radius, node_list_d[i][1] - node_circle_radius,
                              node_list_d[i][0] + node_circle_radius, node_list_d[i][1] + node_circle_radius),
                             fill=(204,255,255), outline=(204,255,255))

            for i in range(len(node_list_d) - 1):
                draw.line(((node_list_d[i][0], node_list_d[i][1]), (node_list_d[i + 1][0], node_list_d[i + 1][1])), fill=(0, 0, 203), width= 1 * scale)

            node_circle_radius = 2 * scale
            for i in range(len(node_list_d) - 1):
                draw.ellipse((node_list_d[i][0] - node_circle_radius, node_list_d[i][1] - node_circle_radius,
                              node_list_d[i][0] + node_circle_radius, node_list_d[i][1] + node_circle_radius),
                             fill=(0, 0, 203), outline=(0, 0, 203))

        return self.rrmap_img

    def create_1x1_landscape_inner_img(self, margin_x, margin_y):
        if margin_x != 0 or margin_y != 0:
            result_img = Image.new(self.rrmap_img.mode,
                                   (self.rrmap_img.size[0] + margin_x * 2, self.rrmap_img.size[1] + margin_y * 2),
                                   (255, 255, 255))
            result_img.paste(self.rrmap_img, (margin_x, margin_y))
            self.rrmap_img = result_img

        if self.scale != 1:
            self.rrmap_img = self.rrmap_img.resize(
                (self.rrmap_img.size[0] * self.scale, self.rrmap_img.size[1] * self.scale), Image.LANCZOS)

        draw = ImageDraw.Draw(self.rrmap_img)

        # draw nodes and legs on the resistance map
        node_circle_radius = 2 * self.scale
        display_offset_x = (margin_x + self.base_margin + self.ladder_margin) * self.scale
        display_offset_y = (margin_y + self.base_margin + self.ladder_margin) * self.scale

        # node_list_d = []
        node_list = np.array([[28, 92], [28, 76], [28, 60], [28, 44],
                              [28, 28], [56, 28], [76, 28], [96, 28],
                              [124, 28], [124, 44], [124, 60], [124, 76],
                              [124, 92], [96, 92], [76, 92], [56, 92], [28, 92]])

        magic_number = 12
        node_list_d = []
        for i in range(len(node_list)):
            node_list_d.append([(node_list[i, 0] - magic_number) * self.scale + display_offset_x,
                                (node_list[i, 1] - magic_number) * self.scale + display_offset_y])

        # for j in range(len(self.weight_list)):
        #    for i in range(len(self.weight_list)):
        #        if self.weight_list[i, j] != 0:
        #            draw.line(((node_list_d[i][0], node_list_d[i][1]), (node_list_d[j][0], node_list_d[j][1])),
        #                      fill=(0, 0, 203), width=1 * self.scale)
        for i in range(len(node_list_d) - 1):
            draw.line(((node_list_d[i][0], node_list_d[i][1]), (node_list_d[i + 1][0], node_list_d[i + 1][1])),
                      fill=(0, 0, 203), width=1 * self.scale)

        for i in range(len(node_list_d) - 1):
            draw.ellipse((node_list_d[i][0] - node_circle_radius, node_list_d[i][1] - node_circle_radius,
                          node_list_d[i][0] + node_circle_radius, node_list_d[i][1] + node_circle_radius),
                         fill=(0, 0, 203), outline=(0, 0, 203))

        return self.rrmap_img


class SpoolMobile(Mobile):
    def __init__(self, id, x, y, angle, dt, max_voltage, tn_gradient, rv_gradient, rv_intercept, gear_ratio, transmission_efficiency, tread, tread_to_tail, trail, gc_x, gc_y, weight,  offset_x, offset_y, rrmap, scale, margin_x, margin_y, num_of_gridx, num_of_gridy, grid_size, landscape, vision_scale):
        super().__init__(id, x, y, angle, dt, max_voltage, tn_gradient, rv_gradient, rv_intercept, gear_ratio, transmission_efficiency, tread, tread_to_tail, trail, gc_x, gc_y, weight,  offset_x, offset_y, rrmap, scale, margin_x, margin_y, num_of_gridx, num_of_gridy, grid_size, landscape, vision_scale)
        self.loader = [Folk(self.id, self.x, self.y, self.angle, self.dt, 0, 0, 0, 0, 0, 0, self.tread, self.tread_to_tail, self.trail, self.gc_x, self.gc_y, self.weight, self.offset_x, self.offset_y, self.rrmap, self.scale, self.margin_x, self.margin_y, self.num_of_gridx, self.num_of_gridy, self.grid_size, self.landscape, vision_scale),
                  Chassis(self.id, self.x, self.y, self.angle, self.dt, 0, 0, 0, 0, 0, 0, self.tread, self.tread_to_tail, self.trail, self.gc_x, self.gc_y, self.weight, self.offset_x, self.offset_y, self.rrmap, self.scale, self.margin_x, self.margin_y, self.num_of_gridx, self.num_of_gridy, self.grid_size, self.landscape, vision_scale),
                  Mobile(self.id, self.x, self.y, self.angle, self.dt, 0, 0, 0, 0, 0, 0, self.tread, self.tread_to_tail, self.trail, self.gc_x, self.gc_y, self.weight, self.offset_x, self.offset_y, self.rrmap, self.scale, self.margin_x, self.margin_y, self.num_of_gridx, self.num_of_gridy, self.grid_size, self.landscape, vision_scale),
                  Right_wheel(self.id, self.x, self.y, self.angle, self.dt, 3.0, 3805, 4838, 32, 203.7, 0.813, self.tread, self.tread_to_tail, self.trail, self.gc_x, self.gc_y, self.weight, self.offset_x, self.offset_y, self.rrmap, self.scale, self.margin_x, self.margin_y, self.num_of_gridx, self.num_of_gridy, self.grid_size, self.landscape, vision_scale),
                  Left_wheel(self.id, self.x, self.y, self.angle, self.dt, 3.0, 3805, 5040, 896, 203.7, 0.813, self.tread, self.tread_to_tail, self.trail, self.gc_x, self.gc_y, self.weight, self.offset_x, self.offset_y, self.rrmap, self.scale, self.margin_x, self.margin_y, self.num_of_gridx, self.num_of_gridy, self.grid_size, self.landscape, vision_scale),
                  Tail_wheel(self.id, self.x, self.y, self.angle, self.dt, 0, 0, 0, 0, 0, 0, self.tread, self.tread_to_tail, self.trail, self.gc_x, self.gc_y, self.weight, self.offset_x, self.offset_y, self.rrmap, self.scale, self.margin_x, self.margin_y, self.num_of_gridx, self.num_of_gridy, self.grid_size, self.landscape, vision_scale)]

        self.trajectory_file_path = 'C:/Users/88310/Downloads/TrajectoryList.npy'
        self.trajectory_list = np.zeros((1, 8))
        # print('left:%f right:%f v_left:%f v_right:%f Chassis angle:%f Tail angle:%f Mobile at x:%f y:%f' % (
        #     0, 0, 0, 0, self.loader[2].angle, self.tail_wheel_angle, self.loader[2].position[0, 0],
        #     self.loader[2].position[1, 0]))
        self.trajectory_list = np.array([0, 0, 0, 0, self.loader[2].angle, self.tail_wheel_angle, self.loader[2].position[0, 0], self.loader[2].position[1, 0]])

    def draw(self, canvas, vision):
        for b in self.loader:
            b.draw(canvas, vision)

            # vision_img = ImageTk.PhotoImage(vision_img)
            # w.create_image(vision_margin_x, vision_margin_y + 120 * scale, image=vision_img, anchor=NW)

    def movem(self, left, right, canvas, vision, save):

        # 左右前輪の速度VL,VRを求める（尾輪角度はφwを使用）
        #   ①
        # SpoolMobileとしてその固有構成部品であるキャスター軸に作用する速度成分Vx_caster、Vy_casterを計算する
        #   ①左右速度VL, VRからω、ρを求める
        # 　②キャスター軸に作用する速度成分Vを求める
        #   ➂始動時の尾輪角度φwとφcasterの差を求める
        #   ④キャスター軸回転円の接線方向成分Vs=V・sin(φcaster - φw)を求める
        #   ⑤尾輪進行方向成分Vw=V・cos(φcaster - φw)を求める
        #   ⑥Vx_caster, Vy_casterを求める
        #   ⑦VL, VRを求める
        #   ⑧ωs = Vs / trailを求めて、φw = φw + δt・ωsを更新する
        #  3⃣⑦で求めた速度VL,VRを各構成部品の速度入力としてmoveする

        # self.rc = self.loader[2].rc
        # self.caster_angle = self.loader[2].caster_angle
        left_move = float(left)  # こうしないとnp.arrayに格納できない。。。
        right_move = float(right)  # こうしないとnp.arrayに格納できない。。。

        self.tail_wheel_angle = self.loader[5].tail_wheel_angle
        # print('rc = %f left = %f right = %f' % (self.rc, left, right))
        # 接地部品の中心座標を取得：getCenterPosition()
        right_wheel_center = self.loader[3].center  # 右駆動輪
        left_wheel_center = self.loader[4].center  # 左駆動輪
        tail_wheel_center = self.loader[5].center  # 従動輪

        # 各接地点と重心との距離を算出
        r_leg = np.linalg.norm(right_wheel_center - self.gravity_center)
        l_leg = np.linalg.norm(left_wheel_center - self.gravity_center)
        t_leg = np.linalg.norm(tail_wheel_center - self.gravity_center)
        # print('Distance R:%f L:%f T:%f' % (r_leg, l_leg, t_leg))

        # 各接地部品の接地点と重心との距離から各接地点の荷重を計算する gf
        r_load = self.weight * (t_leg + l_leg - r_leg) / (2 * r_leg)
        l_load = self.weight * (t_leg + r_leg - l_leg) / (2 * l_leg)
        t_load = self.weight * (l_leg + r_leg - t_leg) / (2 * t_leg)
        # print('Load R:%f L:%f T:%f' % (r_load, l_load, t_load))

        # 分配された荷重を駆動輪、従動輪部品に与えて走行抵抗によって発生する負荷を得る
        # 接地点座標における摩擦係数を取得して負荷[mN]を算出する
        r_registance = self.loader[3].getRegistance(r_load)
        l_registance = self.loader[4].getRegistance(l_load)
        t_registance = self.loader[5].getRegistance(t_load)
        # print('Registance R:%f L:%f T:%f' % (r_registance, l_registance, t_registance))

        # 尾輪の負荷を駆動輪に分配する
        if self.rc==math.inf: # 前進/後退の場合、従動輪負荷トルクを等配分で追加
            r_total_registance = r_registance + t_registance / 2
            l_total_registance = l_registance + t_registance / 2
        else: # 前進/後退以外の場合、従動輪負荷を配分で追加
            t_radius = math.sqrt(self.rc ** 2 + self.tail_mount_distance ** 2)
            # print('t_radius = %f rc = %f tail_mount = %f' % (t_radius, self.rc, self.tail_mount_distance))
            r_total_registance = r_registance + t_registance * (t_radius - (self.rc - self.tread / 2)) / self.tread
            l_total_registance = l_registance + t_registance * ((self.rc + self.tread / 2) - t_radius) / self.tread

        # 尾輪負荷を分配した左右駆動輪の速度を取得する
        v_right = self.loader[3].getVelocity(right, r_total_registance)
        v_left = self.loader[4].getVelocity(left, l_total_registance)
        # print('v_left = %f v_right = %f' % (v_left, v_right))
        if left < 0: # 入力が負値の場合は速度も負値とする
            v_left = -1 * v_left
        if right < 0: # 入力が負値の場合は速度も負値とする
            v_right = -1 * v_right

        # 速度v_left/v_rightから決定される尾輪の進行方向角度φcasterを計算する
        # φcasterはキャスター軸の旋回円の接線方向角度だが、通常、実際の尾輪方向φwとは異なっている
        self.avw = (v_right - v_left) / self.tread  # 左旋回が正
        self.v = (v_right + v_left) / 2
        if self.avw == 0:
            self.rc = math.inf
            if self.v > 0:
                self.caster_angle = 0
            if self.v < 0:
                self.caster_angle = 180
        else:
            self.rc = (v_right + v_left) * (self.tread / 2) / (v_right - v_left)  # 左旋回が正
            self.caster_angle = math.degrees(math.acos(self.rc / math.sqrt(self.rc ** 2 + self.tread_to_tail ** 2)))
        # print('rc = %f' % self.rc)
        # print('rc:%f v:%f caster_angle:%f tail_wheel_angle:%f diff:%f' % (self.rc, self.v, self.caster_angle, self.tail_wheel_angle, self.caster_angle - self.tail_wheel_angle))
        # print('angle = %f' % self.angle)
        # print('Expected v_left = %f v_right = %f' % (v_left, v_right))

        # φw≠φcasterの場合、つまり旋回方向と尾輪進行方向が異なる場合。
        angle_difference = round(self.caster_angle - self.tail_wheel_angle, 6)
        # print('angle_diff=%f' % angle_difference)
        # 尾輪角度が定常旋回時のキャスター角度と一致した場合には計算しない
        if angle_difference != 0 and angle_difference != 180: # 定常旋回中は計算しない
            # キャスター軸に作用する速度成分Vw：尾輪進行方向の速度成分と
            # Vs：キャスター軸に対して垂直方向（尾輪接地点を中心とする半径ｓの回転）の速度成分を計算する
            # キャスター軸中心の速度v_center_velocity(vs, vw)
            caster_center_velocity = np.zeros((2, 1))
            caster_center_velocity[0, 0] = (self.avw * math.sqrt(self.rc ** 2 + self.tread_to_tail ** 2)) * math.sin(math.radians(self.tail_wheel_angle - self.caster_angle))  # Vs
            caster_center_velocity[1, 0] = (self.avw * math.sqrt(self.rc ** 2 + self.tread_to_tail ** 2)) * math.cos(math.radians(self.tail_wheel_angle - self.caster_angle))  # Vw
            # print('rc=%f tread_to_tail=%f' % (self.rc, self.tread_to_tail))
            # print('avw:%f caster_angle:%f tail_wheel_angle:%f Vs:%f Vw:%f' % (self.avw, self.caster_angle, self.tail_wheel_angle, caster_center_velocity[0, 0], caster_center_velocity[1, 0]))

            # 車両座標系に回転
            rot_matrix_caster = [
                [math.cos(math.radians(self.tail_wheel_angle)), math.sin(math.radians(self.tail_wheel_angle))],
                [-1 * math.sin(math.radians(self.tail_wheel_angle)), math.cos(math.radians(self.tail_wheel_angle))]]
            rot_matrix_caster = np.array(rot_matrix_caster)
            v_caster = rot_matrix_caster @ caster_center_velocity # (Vx_caster, Vy_caster)
            # print('Vx_caster:%f Vy_caster:%f' % (v_caster[0, 0], v_caster[1, 0]))

            # 尾輪方向が旋回方向と異なる場合のv_left, v_rightを計算する
            v_left = v_caster[1, 0] - v_caster[0, 0] * (self.tread / (2 * self.tread_to_tail))
            v_right = v_caster[1, 0] + v_caster[0, 0] * (self.tread / (2 * self.tread_to_tail))
            # print('Actual v_left = %f v_right = %f' % (v_left, v_right))
            # print('caster_center_velocity = %f' % caster_center_velocity[0, 0])
            # 尾輪キャスター軸の回転角速度で尾輪角度を更新する
            avw_caster = caster_center_velocity[0, 0] / self.trail # rad/sec
            # print('trail = %f avw_caster = %f' % (self.trail, avw_caster))

            # print('avw_caster=%f tail_wheel_angle=%f dt=%f' % (avw_caster, self.tail_wheel_angle, self.dt))
            # self.tail_wheel_angle = np.mod(self.tail_wheel_angle - math.degrees(self.dt * avw_caster), 360)
            self.tail_wheel_angle = (self.tail_wheel_angle - math.degrees(self.dt * avw_caster)) % 360
            # print('avw_caster=%f tail_wheel_angle=%f dt=%f' % (avw_caster, self.tail_wheel_angle, self.dt))

            # δt秒後の角度変化によって定常旋回時のキャスター軸角度を越えてしまった場合は、尾輪角度をキャスター軸角度にする
            # 尾輪がキャスター軸を中心にして反時計回りに移動していく場合、すなわち尾輪接地点を中心とするキャスター軸中心点の角速度が反時計回り（負の値）の場合
            # δt秒後の角度変化によって定常旋回時の尾輪角度(= キャスター軸角度)を越えてしまった以降は尾輪角度をキャスター軸角度にする
            if avw_caster < 0:
                # δt後の尾輪角度がキャスター角度を超えてしまう場合、すなわちδｔ秒後に両社の角度差の符号が反転する場合
                # 尾輪が右から左へと移動してキャスター角を超える場合
                if (self.loader[5].tail_wheel_angle - self.caster_angle) * (self.tail_wheel_angle - self.caster_angle) < 0:
                    self.tail_wheel_angle = self.caster_angle
                    # print('!!!crossing from right!!!')
            # 尾輪がキャスター軸を中心にして時計回りに移動していく場合、すなわち尾輪接地点を中心とするキャスター軸中心点の角速度が時計回り（正の値）の場合
            # δt秒後の角度変化によって定常旋回時の尾輪角度(= キャスター軸角度 + 180)を越えてしまった以降は尾輪角度をキャスター軸角度にする
            elif avw_caster > 0:
                # δt後の尾輪角度がキャスター角度 + 180を超えてしまう場合、すなわちδｔ秒後に両社の角度差の符号が反転する場合
                # 尾輪が右から左へと移動してキャスター角 + 180度を超える場合
                if (self.loader[5].tail_wheel_angle - 180 - self.caster_angle) * (self.tail_wheel_angle - 180 - self.caster_angle) < 0:
                    self.tail_wheel_angle = self.caster_angle + 180
                    # print('!!!crossing from left!!!')
            self.loader[5].tail_wheel_angle = self.tail_wheel_angle
            # print('avw_caster:%f caster_angle:%f tail_wheel_angle:%f diff:%f' % (avw_caster, self.caster_angle, self.loader[5].tail_wheel_angle, self.loader[5].tail_wheel_angle - self.caster_angle))

        # 駆動輪部品から取得した速度v_left/v_rightをすべての部品に渡してmoveを実行する
        # print('let loader move')
        for b in self.loader:
            b.move(v_left, v_right, canvas, vision)
        # print('left:%f right:%f v_left:%f V_right:%f Chassis angle:%f Tail angle:%f Mobile at x:%f y:%f' % (left_move, right_move, v_left, v_right, self.loader[2].angle, self.tail_wheel_angle, self.loader[2].position[0, 0], self.loader[2].position[1, 0]))
        if save == 1:
            # stack_list = np.array([left_move, right_move, v_left, v_right, self.loader[2].angle, self.tail_wheel_angle, self.loader[2].position[0, 0], self.loader[2].position[1, 0]])
            self.trajectory_list = np.vstack((self.trajectory_list, np.array([left_move, right_move, v_left, v_right, self.loader[2].angle, self.tail_wheel_angle, self.loader[2].position[0, 0], self.loader[2].position[1, 0]])))

    # def get_vision(self, x, y, angle):
    #    vision = self.landscape.copy()
    #    for b in self.loader:
    #        b.position[0, 0], b.position[1, 0], b.angle = x, y, angle
    #        b.drawImage(ImageDraw.Draw(vision))
    #    return vision

    # def update_vision(self, x, y, angle):
    #     for b in self.loader:
    #         b.position[0, 0], b.position[1, 0], b.angle = x, y, angle
    #         b.drawCanvas(self.w)

