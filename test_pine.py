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
        V.start(rate_hz=20, max_loop_count=10000)
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
        V.start(rate_hz=cfg.DRIVE_LOOP_HZ, 
            max_loop_count=cfg.MAX_LOOPS)
    except KeyboardInterrupt:
        print('stopped')

def test_pine_rs():
    cfg = dk.load_config()
    V = dk.vehicle.Vehicle()
    
    from pine.realsense2 import PoseReader
    V.add(PoseReader(cfg), outputs=['pose/x', 'pose/y', 'pose/angle'])

    V.add(PrintPose(), inputs=['pose/x', 'pose/y', 'pose/angle'])

    try:
        V.start(rate_hz=20, max_loop_count=7200)
    except KeyboardInterrupt:
        print('stopped')

def test_pose():
    cfg = dk.load_config()
    V = dk.vehicle.Vehicle()

    from pine.realsense2 import PoseReader
    V.add(PoseReader(cfg), outputs=['pose/real/x', 'pose/real/y', 'pose/real/angle'])

    from pine.map import ImageCreator
    V.add(ImageCreator(cfg),
        inputs=['pose/real/x', 'pose/real/y', 'pose/real/angle'], 
        outputs=['cam/real/image_array'])

    class CopyImage:
        def run(self, image_array):
            return image_array
    V.add(CopyImage(), inputs=['cam/real/image_array'], outputs=['cam/image_array'])

    import os
    from donkeycar.parts.datastore import TubHandler
    os.makedirs(os.path.join(cfg.CAR_PATH, 'data/real'))
    th = TubHandler(path=os.path.join(cfg.CAR_PATH, 'data/real'))
    tub = th.new_tub_writer(
        inputs=['cam/image_array', 'pose/real/x', 'pose/real/y', 'pose/real/angle'],
        types=['image_array', 'float', 'float', 'float'], user_meta={})
    V.add(tub,
        inputs=['cam/image_array', 'pose/real/x', 'pose/real/y', 'pose/real/angle'], 
        outputs=["tub/real/num_records"])

    from pine.double_hedges import PoseReader
    V.add(PoseReader(cfg), outputs=['pose/hedge/x', 'pose/hedge/y', 'pose/hedge/angle'])

    from pine.map import ImageCreator
    V.add(ImageCreator(cfg),
        inputs=['pose/hedge/x', 'pose/hedge/y', 'pose/hedge/angle'], 
        outputs=['cam/hedge/image_array'])

    V.add(CopyImage(), inputs=['cam/hedge/image_array'], outputs=['cam/image_array'])

    os.makedirs(os.path.join(cfg.CAR_PATH, 'data/hedge'))
    th = TubHandler(path=os.path.join(cfg.CAR_PATH, 'data/hedge'))
    tub = th.new_tub_writer(
        inputs=['cam/image_array', 'pose/hedge/x', 'pose/hedge/y', 'pose/hedge/angle'],
        types=['image_array', 'float', 'float', 'float'], user_meta={})
    V.add(tub,
        inputs=['cam/image_array', 'pose/hedge/x', 'pose/hedge/y', 'pose/hedge/angle'], 
        outputs=["tub/hedge/num_records"])

    class PrintBoth:
        def __init__(self):
            print('hedge_x, hedge_y, hedge_angle, real_x, real_y, real_angle')
        def run(self, hx, hy, ha, rx, ry, ra):
            print('{:3f},{:.3f},{:3f},{:3f},{:.3f},{:3f}'.format(hx, hy, ha, rx, ry, ra))
    V.add(PrintBoth(), inputs=[
        'pose/real/x', 'pose/real/y', 'pose/real/angle', 
        'pose/hedge/x', 'pose/hedge/y', 'pose/hedge/angle'])


    try:
        V.start(rate_hz=cfg.DRIVE_LOOP_HZ, 
            #max_loop_count=10000) 
            max_loop_count=cfg.MAX_LOOPS)
    except KeyboardInterrupt:
        print('stopped')

if __name__ == '__main__':
    '''
    print('[start] double hedges')
    test_pine_double_hedges()
    sleep(5.0)
    print('[start] realsense2')
    test_pine_realsense2()
    '''
    #test_pine_rs()
    test_pose()