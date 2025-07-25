from diffusion.single_realsense import SingleRealsense
from multiprocessing.managers import SharedMemoryManager
import multiprocessing as mp
import time


shm_manager = SharedMemoryManager()
shm_manager.start()


serial_numbers = SingleRealsense.get_connected_devices_serial()

realsense = SingleRealsense(
    shm_manager=shm_manager,
    serial_number=serial_numbers[0],
    resolution=(1280, 720),
    capture_fps=30,
    enable_color=True,
    enable_depth=True
)

#realsense.run()
print(realsense.is_ready)

with realsense:
    realsense.start_wait()
    time.sleep(1)
    while True:
        try:
            data = realsense.get(k=6)
            #print(realsense.is_ready)
            if data is not None:
                color_image = data.get('color')
                #print(color_image)
        except KeyboardInterrupt:
            break

shm_manager.shutdown()
