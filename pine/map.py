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
    from .utils import get_color_dict, get_course_node_data, \
        init_node_data, init_resistance_data, init_weight_data, init_garden_axis, course_line
except:
    raise

class ImageCreator:
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
        self.cfg = cfg
        self.debug = debug

        _, course_node_numbers_nodes = \
            get_course_node_data(course_type=cfg.COURSE_TYPE)
        resistance_list = \
            init_resistance_data(cfg.RESISTANCE_LIST_PATH, cfg.NUM_OF_GRID_X)
        weight_list = init_weight_data(cfg.WEIGHT_LIST_PATH)
        node_list = init_node_data(cfg.NODE_LIST_PATH)

        # ロボットのPILイメージオブジェクト
        self.mobile = SpoolMobile(
            6, 28, 92, 180, 0.05, 0, 0, 0, 0, 0, 0, 10.625, 13.78, 1, 0, -6, 1000, 0, 8, 
            resistance_list, 1, 0, 0, 
            cfg.NUM_OF_GRID_X, cfg.NUM_OF_GRID_Y, cfg.GRID_SIZE,
            None, cfg.VISION_SCALE)

        # 倉庫背景のPILイメージオブジェクト
        rrmap_vision_img =  \
            ResistanceMap(
                cfg.RESISTANCE_LIST_PATH,
                get_color_dict()).generateImageFile(cfg.VISION_BACKGROUND_PATH)
        self.vision = Landscape(
            rrmap_vision_img, 
            weight_list, node_list, 
            cfg.BASE_MARGIN, cfg.LADDER_MARGIN, 1).create_1x1_landscape_vision_img_wide4(
                cfg.VISION_MARGIN_X, cfg.VISION_MARGIN_Y,
                course_line(course_node_numbers_nodes, node_list), cfg.VISION_SCALE)
        #self.vision = vision_img_org.copy()

        # Donkeycarパーツクラスの戻り値初期化：イメージ画像配列
        self.image_array = np.zeros((cfg.IMAGE_H, cfg.IMAGE_W, cfg.IMAGE_DEPTH))

        print('[ImageCreator] init completed')

    def update_image_array(self, pos_x, pos_y, angle):
        """
        インスタンス変数上に格納された姿勢情報(self.pos_x, self.pos_y, self.angle)を
        もとに最新位置のイメージ画像配列をself.image_arrayへ格納する。
        引数：
            pos_x       X座標値(単位:studs)
            pos_y       Y座標値(単位:studs)
            angle       方向(単位:度)
        戻り値：
            なし
        """
        # Noneの場合は0に置き換え
        pos_x = 0 if pos_x is None else pos_x
        pos_y = 0 if pos_y is None else pos_y
        angle = 0 if angle is None else angle
        # np.nan の場合は無視
        if pos_x == np.nan or pos_y == np.nan or angle == np.nan:
            if self.debug:
                print('[ImageCreator] omit updating image array because position or angle is nan!')

        # 切り出していないフルサイズのビジョン画面にエージェントを描画する
        next_vision = self.vision.copy()
        for b in self.mobile.loader:
            b.position[0, 0], b.position[1, 0] = pos_x, pos_y
            b.angle = angle
            b.draw(vision=next_vision)
        # エージェントを描画したフルサイズのビジョン画面をそのときの
        # エージェント位置に合わせて切り出し、入力画像サイズに成形する
        # これがローダーカメラ入力の画像
        next_vision_cropped_resized = self.get_torch_view(next_vision, 60, 
            self.mobile.loader[2].position[0, 0],
            self.mobile.loader[2].position[1, 0],
            self.mobile.loader[2].angle,
            self.cfg.VISION_SCALE, self.cfg.IMAGE_W, self.cfg.IMAGE_H,
            self.cfg.VISION_MARGIN_X, self.cfg.VISION_MARGIN_Y)
        if self.debug:
            print('[ImageCreator] next_vision_cropped_resized updated')
            print(next_vision_cropped_resized)
            print(type(next_vision_cropped_resized))
        self.image_array = dk.utils.img_to_arr(next_vision_cropped_resized)
        #dk.utils.arr_to_img(next_vision_cropped_resized).save(
        #'next_vision_after.jpg', quality=100)

    def run(self, pos_x, pos_y, angle):
        """
        姿勢情報をもとにドラクエ風2Dマップ画像を作成し、
        イメージ配列化して返却する。
        引数：
            pos_x       X座標値(単位:studs)
            pos_y       Y座標値(単位:studs)
            angle       方向(単位:度)
        戻り値：
            image_array イメージ画像(nd.array型(120,160,3)形式)
        """
        self.update_image_array(pos_x, pos_y, angle)
        return self.image_array
    
    def run_threaded(self, pos_x, pos_y, angle):
        raise NotImplementedError('[ImageCreator] not supported threaded = True')

    def get_torch_view(self, overall_view, torch_radius, 
    pos_x, pos_y, rotate_angle, v_scale, view_width, view_height, offset_x, offset_y):
        """
        PILイメージoverall_view を指定された座標、方向、サイズ、オフセットに従って
        ロボットイメージを編集しドラクエ風2Dマップ状のPILイメージ化して返却する。
        引数：
            overall_view        ロボットPILイメージ
            torch_radius        松明可視半径
            pos_x               X座標値
            pos_y               Y座標値
            rotate_angle        方向
            v_scale             スケール倍率
            view_width          幅
            view_height         高さ
            offset_x            オフセット(X軸)
            offset_y            オフセット(Y軸)
        戻り値：
            toach_view          編集後のドラクエ風2Dマップ状のPILイメージ
        """
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

