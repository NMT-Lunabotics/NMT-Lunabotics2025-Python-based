import pyrealsense2 as rs
import numpy as np
import cv2


pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.infrared, 1, 1280, 800, rs.format.y8, 30)  # IR1, 1280x800 @ 30 FPS

pipeline.start(config)


device = pipeline.get_active_profile().get_device()
depth_sensor = device.first_depth_sensor()
if depth_sensor.supports(rs.option.emitter_enabled):
    depth_sensor.set_option(rs.option.emitter_enabled, 1)  


print("Streaming infrared video... Press ESC to exit.")


try:
    while True:

        frames = pipeline.wait_for_frames()

        ir_frame = frames.get_infrared_frame()


        if not ir_frame:
            continue
        ir_image = np.asanyarray(ir_frame.get_data())


        cv2.imshow('Infrared Stream (D455)', ir_image)


    
        if cv2.waitKey(1) & 0xFF == 27:
            break


finally:

    pipeline.stop()
    cv2.destroyAllWindows()
    print("Stream stopped.")
