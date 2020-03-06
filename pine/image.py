# -*- coding: utf-8 -*-
"""
Marvelmind 2台のデータを読み込み、
nd.array型(120, 160, 3)形式のイメージ配列を取得する
Donkeycar パーツクラスを提供する。
"""
import donkeycar as dk
import numpy as np
from marvelmind import MarvelmindHedge
from AgentView import update_head_position, update_tail_position, draw, next_vision_img

class DefaultConfig:
    HEAD_HEDGE_ID = ''
    HEAD_HEDGE_TTY = '/dev/ttyACM0'
    TAIL_HEDGE_ID = ''
    TAIL_HEDGE_TTY = '/dev/ttyACM1'

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
        self.cfg = DefaultConfig() if cfg is None else cfg
        self.head_hedge = MarvelmindHedge(
            tty=self.cfg.HEAD_HEDGE_TTY,
            recieveUltrasoundRawDataCallback=update_head_position)
        self.tail_hedge = MarvelmindHedge(
            tty=self.cfg.TAIL_HEDGE_TTY,
            recieveUltrasoundRawDataCallback=update_tail_position)
        self.image = np.zeros((120, 160,3))

    def run_threaded(self):
        draw()
        self.image = next_vision_img
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
        pass


if __name__ == '__main__':
    V = dk.vehicle.Vehicle()

    V.add(MapImageCreator(), outputs=['cam/image_array'])

    class PrintImage:
        def run(self, image_array):
            print('image_array = {}'.format(str(image_array)))

    V.add(PrintImage(), inputs=['cam/image_array'])

    V.start(rate_hz=20, max_loop_count=10000)
