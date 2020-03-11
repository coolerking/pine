# -*- coding: utf-8 -*-
from pine.image import MapImageCreator
from donkeycar.parts.datastore import TubHandler
import donkeycar as dk

def test_pine():
    cfg = dk.load_config()
    V = dk.vehicle.Vehicle()

    V.add(MapImageCreator(cfg), outputs=['cam/image_array'])

    th = TubHandler(path=cfg.DATA_PATH)
    tub = th.new_tub_writer(inputs=['cam/image_array'], types=ty['image_array']pes, user_meta={})
    V.add(tub, inputs=['cam/image_array'], outputs=["tub/num_records"])

    try:
        V.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)
    except KeyboardInterrupt:
        print('stopped')
    else:
        raise

if __name__ == '__main__':
    test_pine()