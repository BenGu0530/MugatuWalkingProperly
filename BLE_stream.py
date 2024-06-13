import asyncio
from bleak import BleakScanner, BleakClient
import platform
from datetime import datetime
import struct

SERVICE_UUID = "180A"
DATA_CHARACTERISTIC_UUID = "00002a58-0000-1000-8000-00805f9b34fb"

def notification_handler(sender, data):
    print(f"通知: {datetime.now()} - {sender}: {data}")
    try:
        current_time, current_pos, current_actual_pos, current_velocity, ax, ay, az, gx, gy, gz, A_left, A_right, f_left, f_right = struct.unpack('<f f f f f f f f f f f f f f', data)
        print(f"Time: {current_time}")
        print(f"Pos: {current_pos}")
        print(f"Actual Pos: {current_actual_pos}")
        print(f"Velocity: {current_velocity}")
        print(f"Acceleration: ({ax}, {ay}, {az})")
        print(f"Gyro: ({gx}, {gy}, {gz})")
        print(f"Amplitude Left: {A_left}")
        print(f"Amplitude Right: {A_right}")
        print(f"Frequency Left: {f_left}")
        print(f"Frequency Right: {f_right}")
    except struct.error as e:
        print(f"数据解码错误: {e}")

async def connect_to_device(address):
    print(f"尝试连接到设备 {address}...")
    client = BleakClient(address)
    try:
        await asyncio.wait_for(client.connect(), timeout=15.0)
        if client.is_connected:
            print("连接成功!")
            return client
        else:
            print("连接失败!")
            return None
    except asyncio.TimeoutError:
        print("连接超时!")
    except Exception as e:
        print(f"连接失败: {e}")
    return None

async def get_services_with_retry(client, max_retries=5):
    retries = 0
    while retries < max_retries:
        try:
            services = await client.get_services()
            return services
        except Exception as e:
            if "拒绝访问" in str(e):
                print(f"获取服务时出错: {e}，正在重试...")
                retries += 1
                await asyncio.sleep(1)
            else:
                raise e
    raise Exception("无法获取服务: 达到最大重试次数")

async def run():
    print("开始扫描BLE设备...")
    try:
        devices = await BleakScanner.discover()
    except Exception as e:
        print(f"扫描设备时出错: {e}")
        return

    if not devices:
        print("未发现任何设备")
        return

    target_device = None
    for device in devices:
        print(f"发现设备: {device.name}, {device.address}")
        if device.name == "Arduino":
            target_device = device
            break

    if target_device is None:
        print("未找到目标设备: Arduino")
        return

    print(f"找到目标设备: {target_device.name}, {target_device.address}")

    client = await connect_to_device(target_device.address)
    if client is None:
        print("无法连接到设备")
        return

    try:
        services = await get_services_with_retry(client)

        for service in services:
            print(f"Service UUID: {service.uuid}")
            for characteristic in service.characteristics:
                print(f"  Characteristic UUID: {characteristic.uuid}")

        await client.start_notify(DATA_CHARACTERISTIC_UUID, notification_handler)
        print("开始接收通知...")

        # 持续接收通知
        while True:
            await asyncio.sleep(1)

    except Exception as e:
        print(f"运行时出错: {e}")

if __name__ == "__main__":
    print(f"Python version: {platform.python_version()}")
    print(f"Platform: {platform.system()} {platform.release()}")
    try:
        asyncio.run(run())
    except Exception as e:
        print(f"运行时出错: {e}")


