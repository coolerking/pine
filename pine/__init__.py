# -*- coding: utf-8 -*-
'''
姿勢情報を取得するためのPoseReaderオブジェクト取得するための
ファクトリ関数と、姿勢情報からドラクエ風2Dマップ画像イメージ配列
を生成するImageCreatorを取得するためのファクトリ関数を提供するモジュール。
'''

def get_image_creator(cfg, debug=False):
    """
    ImageCreatorファクトリ関数。
    ImageCreatorオブジェクトを返却する。
    引数：
        cfg     config.py/myconfig.py オブジェクト（必須）
        debug   デバッグオプション（デフォルトFalse）
    戻り値：
        ImageCreatorオブジェクト
    """
    from .map import ImageCreator
    return ImageCreator(cfg, debug=debug)

def get_pose_reader(cfg, debug=False):
    """
    PoseReaderファクトリ関数。
    引数としてあたえられたcfg内のPOSE_DEVICE_TYPE値に対応する
    PoseReaderオブジェクトを返却する。
    引数：
        cfg     config.py/myconfig.py オブジェクト（必須）
        debug   デバッグオプション（デフォルトFalse）
    戻り値：
        PoseReaderオブジェクト
    例外：
        NotImplementedError POSE_DEVICE_TYPEにサポート外の値を指定した場合
    """
    try:
        if cfg.POSE_DEVICE_TYPE == 'double_hedges':
            from .double_hedges import PoseReader
        elif cfg.POSE_DEVICE_TYPE == 'realsense2':
            from .realsense2 import PoseReader
        else:
            raise NotImplementedError('not supported POSE_DEVICE_TYPE = {}'.format(
                str(cfg.POSE_DEVICE_TYPE)))
        return PoseReader(cfg, debug=debug)
    except:
        raise
