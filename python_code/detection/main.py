from ble_connect import connect
from plane_constraint import pos_calculate
import asyncio
from frame_process import vision_loop
import time

pos = [0,0,0]
time.sleep(3)  # 等待3秒，确保摄像头初始化完成
try:
    # 获得目标像素坐标
    pixel_x, pixel_y = vision_loop()

    if pixel_x is None or pixel_y is None:
        print("视觉采集失败，不继续下发坐标！")
    else:
        pos = pos_calculate((pixel_x,pixel_y))
        if pos is None:
            print("pos计算错误,不发送坐标")
        else:
            asyncio.run(connect(pos[0],pos[1],pos[2]))

except Exception as e:
    print("异常:",e)
