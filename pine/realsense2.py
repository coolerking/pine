# -*- coding: utf-8 -*-
"""
Donkeycar用Intel RealSense T265 トラッキングカメラから姿勢情報を取得する
Donkeycar パーツクラスを提供するモジュール。
（ImageCreatorの入力となるデータを作成する）

動作はロボットをmyconfig.pyで指定した初期位置に配置していることを前提とする。

| 項目     | T265 pose                                | ImageCreator                        |
|:---------|:-----------------------------------------|:------------------------------------|
| 原点     | カメラ正面中央(初期位置に必ず置く)         | 自動補正される(どこにおいても可)     |
| 位置X軸  | カメラ正面から右方向(単位:m)               | ロボット正面から前方向(単位:studs)   |
| 位置Y軸  | カメラ正面から真上方向(単位:m)             | ロボット正面から右方向(単位:studs)   |
| 位置Z軸  | カメラ正面から真後方向(単位:m)             | ロボット正面から真下方向(単位:studs) |
| 方向X軸  | カメラ正面から右方向右回り(単位:ラジアン)   | ロボット正面から前方向(単位:studs)   |
| 方向Y軸  | カメラ正面から真上方向右回り(単位:ラジアン) | ロボット正面から右方向(単位:studs)   |
| 方向Z軸  | カメラ正面から真後方向右回り(単位:ラジアン) | ロボット正面から真下方向(単位:studs) |

"""
import time
import logging
import math # as m
import numpy as np
try:
    import pyrealsense2 as rs
except:
    print('[realsense2] This module requires pyrealsense2 package!')
    raise
from .utils import convert_studs

class PoseReader:
    """
    Intel RealSense Tracking Camera T265 からImageCreatorの入力データ
    （姿勢情報）を取得する Donkeycar パーツクラス。
    """
    def __init__(self, cfg, debug=False):
        """
        トラッキングカメラオブジェクトを初期化する。
        引数：
            cfg         config.py/myconfig.pyオブジェクト
            debug       デバッグフラグ（デフォルトFalse）
        戻り値：
            なし
        """
        self.cfg = cfg
        self.debug = debug
        self.offset_x = convert_studs(cfg.START_X_CM, unit='cm')
        self.offset_y = convert_studs(cfg.START_Y_CM, unit='cm')
        self.offset_z = convert_studs(cfg.START_Z_CM, unit='cm')
        self.offset_angle = cfg.START_ANGLE_DEGREES
        print('[OFFSET] {:.3f},{:.3f},{:.3f},{:.3f}'.format(self.offset_x, self.offset_y, self.offset_z, self.offset_angle))
        self.camera = T265(image_output=False, wait_secs=cfg.WAIT_INTERVAL, debug=debug)
    
    def update(self):
        """
        別スレッドが生成されたら、このメソッドが呼び出される。
        T265からセンサデータを取得する。
        引数：
            なし
        戻り値：
            なし
        """
        self.camera.update()

    def run_threaded(self):
        """
        パーツクラスTemplate Methodのひとつ。threadedが真である場合、
        run()のかわりに呼び出される。
        T265から最後に取得したImageCreatorの入力データ(位置座標および方向)を返却する。
        引数：
            なし
        戻り値：
            pos_x               位置情報X軸（単位：studs）
            pos_y               位置情報Y軸（単位：studs）
            angle               方向（単位：度）
        """
        pos_x, _, pos_z, \
        _, _, _, \
        _, _, _, \
        _, _, _, \
        _, _, _, \
        _, _, _, _, \
        _, _, \
        _, _, ang_z, \
        _, _ = self.camera.run_threaded()
        pos_x, pos_y, angle = self.convert_pose(pos_x, pos_z, ang_z)
        if self.debug:
            print('{}, {}, {}'.format(str(pos_x), str(pos_y), str(angle)))
        return pos_x, pos_y, angle

    def run(self):
        """
        パーツクラスTemplate Methodのひとつ。threadedが偽である場合、
        run_threaded()のかわりに呼び出される。
        T265からImageCreatorの入力データ(位置座標および方向)を取得し返却する。
        引数：
            なし
        戻り値：
            pos_x               位置情報X軸（単位：studs）
            pos_y               位置情報Y軸（単位：studs）
            angle               方向（単位：度）
        """
        pos_x, _, pos_z, \
        _, _, _, \
        _, _, _, \
        _, _, _, \
        _, _, _, \
        _, _, _, _, \
        _, _, \
        _, _, ang_z, \
        _, _ = self.camera.run()
        pos_x, pos_y, angle = self.convert_pose(pos_x, pos_z, ang_z)
        if self.debug:
            print('{}, {}, {}'.format(str(pos_x), str(pos_y), str(angle)))
        return pos_x, pos_y, angle

    def shutdown(self):
        """
        T265側のスレッドを終了する。
        引数：
            なし
        戻り値：
            なし
        """
        self.camera.shutdown()

    def convert_pose(self, pos_x, pos_z, ang_z):
        """
        T265出力(pos_x, pos_z, ang_z)をもとに
        ImageCreator入力データに編集する。
        引数：
            pos_x       T265位置情報(X座標, 単位:m)
            pos_z       T265位置情報(Z座標, 単位:m)
            ang_z       T265オイラー角(Z座標, 単位:ラジアン)
        戻り値：
            pos_x       倉庫内位置情報(X座標, 単位:studs)
            pos_y       倉庫内位置情報(Y座標, 単位:studs)
            angle       倉庫内方向(単位:度)
        """
        x = convert_studs(float(-pos_z), unit='m') + convert_studs(self.offset_x, unit='cm')
        y = convert_studs(float(pos_x), unit='m') + convert_studs(self.offset_y, unit='cm')
        #pos_z = convert_studs(float(pos_y), unit='m') + convert_studs(self.offset_z, unit='cm')
        angle = float((float(ang_z) * float(-1.0)) + float(self.offset_angle))
        return x, y, angle

class T265:
    '''
    Intel RealSense Tracking Camera T265 から全センサデータを取得する
    Donkeycarパーツクラス。
    '''

    def __init__(self, image_output=False, wait_secs=0.1, debug=False):
        """
        RealSense T265トラッキングカメラからデータを取得するパーツクラス。
        引数：
            image_output    T265に搭載された2つの魚眼カメラのうち片方から
                            画像ストリームを取得する(USB3.0推奨)。
                            デフォルトはFalse、runを実行すると常にNoneが返却される。
            wait_secs       センサ有効化/無効化直後の待機時間(秒)、デフォルトは0.1秒。
            debug           デバッグフラグ。真値にすると標準出力にログを出力する。
        戻り値：
            なし
        """
        self.debug = debug
        if self.debug:
            print('[T265] __init__ start')
        self.wait_secs = wait_secs
        self.image_output = image_output

        # RealSenseパイプラインを宣言し、実際のデバイスとセンサをカプセル化
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)

        if self.image_output:
            # 現時点で両カメラを有効にする必要あり
            cfg.enable_stream(rs.stream.fisheye, 1) # 左カメラ
            cfg.enable_stream(rs.stream.fisheye, 2) # 右カメラ

        # 要求された校正でストリーミング開始
        self.pipe.start(cfg)
        if self.debug:
            print('[T265] __init__ wait {} secs'.format(str(self.wait_secs)))
        time.sleep(self.wait_secs)

        # スレッド開始
        self.running = True
        
        zero_vec = (0.0, 0.0, 0.0)
        self.pos = zero_vec
        self.vel = zero_vec
        self.acc = zero_vec
        self.e_vel = zero_vec
        self.e_acc = zero_vec
        self.rot = (0.0, 0.0, 0.0, 0.0)
        self.ang = zero_vec
        self.posemap_conf = 0x0 # 失敗
        self.pose_conf = 0x0 # 失敗
        self.left_img = None
        self.right_img = None
        if self.debug:
            print('[T265] __init__ end')

    def poll(self):
        try:
            frames = self.pipe.wait_for_frames()
        except Exception as e:
            if self.debug:
                print(e)
            logging.error(e)
            return

        if self.image_output:
            # 左魚眼ガメラからイメージを取得する
            left = frames.get_fisheye_frame(1)
            self.left_img = np.asanyarray(left.get_data())
            # 右魚眼ガメラからイメージを取得する
            right = frames.get_fisheye_frame(2)
            self.right_img = np.asanyarray(right.get_data())


        # 位置情報フレームをフェッチ
        pose = frames.get_pose_frame()

        if pose:
            data = pose.get_pose_data()
            # 位置座標
            self.pos = (data.translation.x, data.translation.y, data.translation.z)
            # 速度
            self.vel = (data.velocity.x, data.velocity.y, data.velocity.z)
            # 加速度
            self.acc = (data.acceleration.x, data.acceleration.y, data.acceleration.z)
            # 角速度
            self.e_vel = (data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z)
            # 角加速度
            self.e_acc = (data.angular_acceleration.x, data.angular_acceleration.y, data.angular_acceleration.z)
            # 四元数
            self.rot = (data.rotation.w, -data.rotation.z, data.rotation.x, -data.rotation.y)
            # オイラー角
            self.ang = self.get_eular_angle()
            # マップ信頼度：0x0-失敗、0x1-低、0x2-中、0x3-高
            self.posemap_conf = data.mapper_confidence
            # 位置座標信頼度：0x0-失敗、0x1-低、0x2-中、0x3-高
            self.pose_conf = data.tracker_confidence
            logging.debug('[T265] poll() pos(%f, %f, %f)' % (self.pos[0], self.pos[1], self.pos[2]))
            #logging.debug('[T265] poll() ang(%f, %f, %f)' % (self.ang[0], self.ang[1], self.ang[2]))
            if self.debug:
                print('[T265] poll() pos(%f, %f, %f)' % (self.pos[0], self.pos[1], self.pos[2]))
                print('[T265] poll() vel(%f, %f, %f)' % (self.vel[0], self.vel[1], self.vel[2]))
                print('[T265] poll() ang(%f, %f, %f)' % (self.ang[0], self.ang[1], self.ang[2]))
                print('[T265] poll() rot(%f, %f, %f, %f)' % (self.rot[0], self.rot[1], self.rot[2], self.rot[3]))
                print('[T265] poll() eular vel(%f, %f, %f)' % (self.e_vel[0], self.e_vel[1], self.e_vel[2]))
                print('[T265] poll() eular acc(%f, %f, %f)' % (self.e_acc[0], self.e_acc[1], self.e_acc[2]))
                print('[T265] poll() conf map:%d pose:%d' % (self.posemap_conf, self.pose_conf))
                print('[T265] poll() left is None:{} right is None:{}'.format(str(self.left_img is None), str(self.right_img is None)))

    def get_eular_angle(self):
        """
        インスタンス変数 `self.rot` （四元数）から姿勢角速度を算出する。
        引数：
            なし
        戻り値：
            (roll, pitch, yaw)  初期位置を基準としたオイラー角（ラジアン）
        """
        w, x, y, z = self.rot[0], self.rot[1], self.rot[2], self.rot[3]
        roll  =  math.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / math.pi
        pitch =  -math.asin(2.0 * (x*z - w*y)) * 180.0 / math.pi
        yaw   =  math.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / math.pi
        return (roll, pitch, yaw)

    def update(self):
        """
        別スレッドが生成されたら、このメソッドが呼び出される。
        T265からセンサデータを取得する。
        インスタンス変数runningが真である間、poll()を実行する。
        引数：
            なし
        戻り値：
            なし
        """
        while self.running:
            self.poll()

    def run_threaded(self):
        """
        パーツクラスTemplate Methodのひとつ。threadedが真である場合、
        run()のかわりに呼び出される。
        T265で取得可能なすべての最新センサデータを返却する。
        最新センサデータは本メソッド実行時に取得していない（別スレッド
        により更新）。
        引数：
            なし
        戻り値：
            pos_x               位置情報X軸（単位：メートル）
            pos_y               位置情報Y軸（単位：メートル）
            pos_z               位置情報Z軸（単位：メートル）
            vel_x               速度X軸（単位：メートル/秒）
            vel_y               速度Y軸（単位：メートル/秒）
            vel_z               速度Z軸（単位：メートル/秒）
            e_vel_x             角速度X軸、gyr_xに相当（単位：ラジアン/秒）
            e_vel_y             角速度Y軸、gyr_yに相当（単位：ラジアン/秒）
            e_vel_z             角速度Z軸、gyr_zに相当（単位：ラジアン/秒）
            acc_x               加速度X軸（単位：メートル/秒^2）
            acc_y               加速度Y軸（単位：メートル/秒^2）
            acc_z               加速度Z軸（単位：メートル/秒^2）
            e_acc_x             角加速度X軸（単位：ラジアン/秒^2）
            e_acc_y             角加速度Y軸（単位：ラジアン/秒^2）
            e_acc_z             角加速度Z軸（単位：ラジアン/秒^2）
            rot_i               四元数(Qi)
            rot_j               四元数(Qj)
            rot_k               四元数(Qk)
            rot_l               四元数(Ql)
            posemap_conf        poseマップ信頼度：0x0-失敗、0x1-低、0x2-中、0x3-高
            pose_conf           pose信頼度：0x0-失敗、0x1-低、0x2-中、0x3-高
            ang_x               オイラー角X軸(ロール)（単位：ラジアン）
            ang_y               オイラー角Y軸(ピッチ)（単位：ラジアン）
            ang_z               オイラー角Z軸(ヨー)（単位：ラジアン）
            left_image_array    左カメライメージ(nd.array型、(800,848)形式)
            right_image_array   右カメライメージ(nd.array型、(800,848)形式)
        """
        return self.pos[0], self.pos[1], self.pos[2], \
            self.vel[0], self.vel[1], self.vel[2], \
            self.e_vel[0], self.e_vel[1], self.e_vel[2], \
            self.acc[0], self.acc[1], self.acc[2], \
            self.e_acc[0], self.e_acc[1], self.e_acc[2], \
            self.rot[0], self.rot[1], self.rot[2], self.rot[3], \
            self.posemap_conf, self.pose_conf , \
            self.ang[0], self.ang[1], self.ang[2], \
            self.left_img, self.right_img

    def run(self):
        """
        パーツクラスTemplate Methodのひとつ。threadedが偽である場合、
        run_threaded()のかわりに呼び出される。
        T265で取得可能なすべての最新センサデータ（本メソッド呼び出し時
        に取得）を返却する。
        引数：
            なし
        戻り値：
            pos_x               位置情報X軸（単位：メートル）
            pos_y               位置情報Y軸（単位：メートル）
            pos_z               位置情報Z軸（単位：メートル）
            vel_x               速度X軸（単位：メートル/秒）
            vel_y               速度Y軸（単位：メートル/秒）
            vel_z               速度Z軸（単位：メートル/秒）
            e_vel_x             角速度X軸、gyr_xに相当（単位：ラジアン/秒）
            e_vel_y             角速度Y軸、gyr_yに相当（単位：ラジアン/秒）
            e_vel_z             角速度Z軸、gyr_zに相当（単位：ラジアン/秒）
            acc_x               加速度X軸（単位：メートル/秒^2）
            acc_y               加速度Y軸（単位：メートル/秒^2）
            acc_z               加速度Z軸（単位：メートル/秒^2）
            e_acc_x             角加速度X軸（単位：ラジアン/秒^2）
            e_acc_y             角加速度Y軸（単位：ラジアン/秒^2）
            e_acc_z             角加速度Z軸（単位：ラジアン/秒^2）
            rot_i               四元数(Qi)
            rot_j               四元数(Qj)
            rot_k               四元数(Qk)
            rot_l               四元数(Ql)
            posemap_conf        poseマップ信頼度：0x0-失敗、0x1-低、0x2-中、0x3-高
            pose_conf           pose信頼度：0x0-失敗、0x1-低、0x2-中、0x3-高
            ang_x               オイラー角X軸(ロール)（単位：ラジアン）
            ang_y               オイラー角Y軸(ピッチ)（単位：ラジアン）
            ang_z               オイラー角Z軸(ヨー)（単位：ラジアン）
            left_image_array    左カメライメージ(nd.array型、(800,848)形式)
            right_image_array   右カメライメージ(nd.array型、(800,848)形式)
        """
        self.poll()
        return self.run_threaded()

    def shutdown(self):
        """
        パーツクラスTemplate Methodのひとつ。終了時処理。
        マルチスレッドループを閉じ、T265パイプを停止する。
        引数：
            なし
        戻り値：
            なし
        """
        if self.debug:
            print('[T265] shutdown start')
        self.running = False
        if self.debug:
            print('[T265] shutdown wait {} secs'.format(str(self.wait_secs)))
        time.sleep(self.wait_secs)
        self.pipe.stop()
        if self.debug:
            print('[T265] shutdown end')


if __name__ == "__main__":
    c = T265(image_output=True, debug=True)
    while True:
        c.run()
        #print(pos)
        time.sleep(0.1)
    c.shutdown()