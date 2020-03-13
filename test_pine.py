# -*- coding: utf-8 -*-
"""
疎通テスト用モジュール。
"""
import donkeycar as dk
from time import sleep

class PrintPose:
    def __init__(self):
        print('pos_x, pos_y, angle')

    def run(self, pos_x, pos_y, angle):
        print('{}, {}, {}'.format(str(pos_x), str(pos_y), str(angle)))

def test_pine_double_hedges():
    cfg = dk.load_config()
    V = dk.vehicle.Vehicle()
    
    from pine.double_hedges import PoseReader
    V.add(PoseReader(cfg), outputs=['pose/x', 'pose/y', 'pose/angle'])

    from pine.map import ImageCreator
    V.add(ImageCreator(cfg), inputs=['pose/x', 'pose/y', 'pose/angle'], outputs=['cam/image_array'])

    from donkeycar.parts.datastore import TubHandler
    th = TubHandler(path=cfg.DATA_PATH)
    tub = th.new_tub_writer(
        inputs=['cam/image_array', 'pose/x', 'pose/y', 'pose/angle'],
        types=['image_array', 'float', 'float', 'float'], user_meta={})
    V.add(tub,
        inputs=['cam/image_array', 'pose/x', 'pose/y', 'pose/angle'], 
        outputs=["tub/num_records"])

    V.add(PrintPose(), inputs=['pose/x', 'pose/y', 'pose/angle'])

    try:
        V.start(rate_hz=20, max_loop_count=3600)
    except KeyboardInterrupt:
        print('stopped')

def test_pine_realsense2():
    cfg = dk.load_config()
    V = dk.vehicle.Vehicle()
    
    from pine.realsense2 import PoseReader
    V.add(PoseReader(cfg), outputs=['pose/x', 'pose/y', 'pose/angle'])

    from pine.map import ImageCreator
    V.add(ImageCreator(cfg), inputs=['pose/x', 'pose/y', 'pose/angle'], outputs=['cam/image_array'])

    from donkeycar.parts.datastore import TubHandler
    th = TubHandler(path=cfg.DATA_PATH)
    tub = th.new_tub_writer(
        inputs=['cam/image_array', 'pose/x', 'pose/y', 'pose/angle'],
        types=['image_array', 'float', 'float', 'float'], user_meta={})
    V.add(tub,
        inputs=['cam/image_array', 'pose/x', 'pose/y', 'pose/angle'], 
        outputs=["tub/num_records"])

    V.add(PrintPose(), inputs=['pose/x', 'pose/y', 'pose/angle'])

    try:
        V.start(rate_hz=20, max_loop_count=3600)
    except KeyboardInterrupt:
        print('stopped')

def test_pine_rs():
    cfg = dk.load_config()
    V = dk.vehicle.Vehicle()
    
    from pine.realsense2 import PoseReader2
    V.add(PoseReader(cfg), outputs=['pose/x', 'pose/y', 'pose/z', 'pose/ang_x', 'pose/ang_y', 'pose/ang_z'])

    class Prt:
        def run(self, x, y, z, ax, ay, az):
            print('{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}'.format(x, y, z, ax, ay, az))

    V.add(Prt(), inputs=['pose/x', 'pose/y', 'pose/z', 'pose/ang_x', 'pose/ang_y', 'pose/ang_z'])

    try:
        V.start(rate_hz=20, max_loop_count=3600)
    except KeyboardInterrupt:
        print('stopped')

if __name__ == '__main__':
    #print('[start] double hedges')
    #test_pine_double_hedges()
    #sleep(5.0)
    #print('[start] realsense2')
    #test_pine_realsense2()

    test_pine_rs()