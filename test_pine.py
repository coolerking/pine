# -*- coding: utf-8 -*-


import donkeycar as dk

'''
def test_pine():
    from pine.image import MapImageCreator
    cfg = dk.load_config()
    V = dk.vehicle.Vehicle()

    V.add(MapImageCreator(cfg), outputs=['cam/image_array'])

    th = TubHandler(path=cfg.DATA_PATH)
    tub = th.new_tub_writer(inputs=['cam/image_array'], types=['image_array'], user_meta={})
    V.add(tub, inputs=['cam/image_array'], outputs=["tub/num_records"])

    try:
        V.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)
    except KeyboardInterrupt:
        print('stopped')
    else:
        raise
'''
def test_pine():
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

    try:
        V.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)
    except KeyboardInterrupt:
        print('stopped')

if __name__ == '__main__':
    test_pine()