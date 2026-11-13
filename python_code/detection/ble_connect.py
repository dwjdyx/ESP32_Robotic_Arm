import asyncio
import struct
from bleak import BleakScanner, BleakClient

# 替换成 ble.c 中实际定义的 UUID
DEVICE_NAME = "ESP32-NimBLE"
RX_UUID = "0xFF01"

def pack_coord(x: int, y: int, z: int) -> bytes:
    """打包 XYZ 坐标，单位 mm,使用有符号 int16 大端序。"""
    if not all(-32768 <= v <= 32767 for v in (x, y, z)):
        raise ValueError("坐标超出 int16 范围")

    return (
        bytes([0xA1, 0xB2])
        + struct.pack(">hhh", x, y, z)
        + bytes([0xC3])
    )

async def connect(x,y,z):
    print("正在搜索 ESP32-S3...")

    device = await BleakScanner.find_device_by_name(
        DEVICE_NAME, timeout=10.0
    )

    if device is None:
        print("未找到设备，请检查 ESP32 的 BLE 广播名称")
        return

    async with BleakClient(device) as client:
        print("BLE 连接成功")

        frame = pack_coord(x, y, z)

        print(f"发送坐标:X={x}, Y={y}, Z={z}")
        print("发送数据：", frame.hex(" "))

        # True 表示使用 Write With Response
        await client.write_gatt_char(
            RX_UUID, frame, response=True
        )

        print("坐标帧写入完成")

# if __name__ == "__main__":
    # asyncio.run(main())