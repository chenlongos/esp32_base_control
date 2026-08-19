"""
ESP32-C3 距离 / 转向闭环测试（验证烧录固件的物理量换算）
用法:
  python tests/test_move_distance.py [/dev/cu.usbmodemXXXX]

交互命令:
  f <mm> [speed]   前进指定距离(mm)   例: f 500 60
  b <mm> [speed]   后退指定距离(mm)   例: b 300
  l <deg> [speed]  原地左转(度)       例: l 90 40
  r <deg> [speed]  原地右转(度)       例: r 90
  e                读取编码器累计计数
  demo             脚本化演示(前进500mm + 左转90° + 前进500mm)
  q                退出

说明: 固件内部用轮径 D=62mm、轴距 L=160mm、PPR=4680 换算成编码器计数，
      MOVE_DISTANCE 的 target 对直行为 mm、对转向为 0.1°。
"""

import serial
import struct
import sys
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem1201"
BAUD = 115200

# 命令字
INIT, CONFIG, MOVE_DISTANCE, GET_ENCODER, GET_STATUS, RESET = 0x01, 0x02, 0x23, 0x22, 0x21, 0xFF
ACK, NACK, STATUS = 0x80, 0x81, 0x91

# 物理参数（与固件 / odometry.py 一致）
WHEEL_DIAMETER_MM = 62.0
WHEELBASE_MM = 160.0
PPR = 4680
MM_PER_COUNT = (WHEEL_DIAMETER_MM * 3.141592653589793) / PPR

G, R, B, Y, X = "\033[92m", "\033[91m", "\033[94m", "\033[93m", "\033[0m"


def frame(cmd, payload=b""):
    chk = cmd ^ len(payload)
    for b in payload:
        chk ^= b
    return bytes([0xAA, 0x55, cmd, len(payload)]) + payload + bytes([chk])


def recv(ser, timeout=1.0):
    dl = time.time() + timeout
    while time.time() < dl:
        if ser.read(1) == b"\xaa" and ser.read(1) == b"\x55":
            hdr = ser.read(2)
            if len(hdr) < 2:
                return None
            cmd, ln = hdr[0], hdr[1]
            data = ser.read(ln) if ln > 0 else b""
            if len(data) < ln or not ser.read(1):
                return None
            return {"cmd": cmd, "data": data}
    return None


def ack(ser, timeout=0.5):
    dl = time.time() + timeout
    while time.time() < dl:
        f = recv(ser)
        if f and f["cmd"] == ACK:
            return True
        if f and f["cmd"] == NACK:
            err = f["data"][1] if len(f["data"]) > 1 else -1
            print(f"{R}NACK 错误码 0x{err:02x}{X}")
            return False
    return False


def init(ser):
    ser.write(frame(INIT))
    return ack(ser)


def config(ser):
    ser.write(frame(CONFIG, struct.pack(">HH", PPR, 20000)))
    return ack(ser)


def read_encoders(ser):
    """返回 (left, right) 编码器累计计数"""
    ser.write(frame(GET_ENCODER))
    f = recv(ser)
    if f and f["cmd"] == GET_ENCODER and len(f["data"]) >= 8:
        left = struct.unpack(">i", f["data"][0:4])[0]
        right = struct.unpack(">i", f["data"][4:8])[0]
        return left, right
    return None


def read_rpm(ser):
    ser.write(frame(GET_STATUS))
    f = recv(ser)
    if f and f["cmd"] == STATUS and len(f["data"]) >= 5:
        state = f["data"][0]
        rpm1 = struct.unpack(">h", f["data"][1:3])[0]
        rpm2 = struct.unpack(">h", f["data"][3:5])[0]
        return state, rpm1, rpm2
    return None


def move(ser, direction, speed, target_raw):
    """direction: 0=前进 1=后退 2=左转 3=右转
       target_raw: 直行=mm, 转向=0.1°(如 90° 传 900)"""
    payload = bytes([direction, speed]) + struct.pack(">i", target_raw)
    ser.write(frame(MOVE_DISTANCE, payload))
    return ack(ser, 0.5)


def wait_motion_done(ser, timeout=15.0):
    """轮询 GET_STATUS，直到两电机 RPM 均为 0（距离/转向到达后自动刹车）"""
    dl = time.time() + timeout
    while time.time() < dl:
        r = read_rpm(ser)
        if r and r[1] == 0 and r[2] == 0:
            return True
        time.sleep(0.1)
    return False


def show_travel(ser, tag):
    enc = read_encoders(ser)
    if enc:
        l_mm = enc[0] * MM_PER_COUNT
        r_mm = enc[1] * MM_PER_COUNT
        print(f"  {tag} 编码器 L={enc[0]:+7d} R={enc[1]:+7d}  →  里程 L={l_mm:+7.1f}mm R={r_mm:+7.1f}mm")
    else:
        print(f"  {tag} {R}读取编码器失败{X}")


def main():
    print(f"\n{B}ESP32-C3 距离/转向闭环测试{X}  {PORT}")
    print(f"  物理参数: 轮径 D={WHEEL_DIAMETER_MM}mm  轴距 L={WHEELBASE_MM}mm  PPR={PPR}")

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.3)
        time.sleep(0.3)
    except Exception as e:
        print(f"{R}串口错误: {e}{X}")
        return

    if not init(ser):
        print(f"{R}INIT 失败{X}")
        ser.close()
        return
    if not config(ser):
        print(f"{R}CONFIG 失败{X}")
        ser.close()
        return
    print(f"{G}[就绪]{X}\n")

    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        parts = line.split()
        c = parts[0].lower()

        try:
            if c == "q":
                break

            elif c == "e":
                show_travel(ser, "编码器")

            elif c in ("f", "b", "l", "r"):
                val = float(parts[1])
                spd = int(parts[2]) if len(parts) > 2 else 60
                spd = max(1, min(100, spd))
                if c in ("f", "b"):
                    direction = 0 if c == "f" else 1
                    target_raw = int(val)  # mm
                    unit = "mm"
                else:
                    direction = 2 if c == "l" else 3
                    target_raw = int(round(val * 10))  # 0.1°
                    unit = "°"
                if not move(ser, direction, spd, target_raw):
                    continue
                print(f"{Y}执行中... {c} {val}{unit} @ {spd}{X}")
                if wait_motion_done(ser):
                    print(f"{G}[到达]{X}")
                    show_travel(ser, "   ")
                else:
                    print(f"{R}[超时]{X}")

            elif c == "demo":
                print(f"{Y}演示: 前进500mm → 左转90° → 前进500mm{X}")
                for direction, target, spd, tag in [
                    (0, 500, 60, "前进500mm"),
                    (2, 900, 40, "左转90°"),
                    (0, 500, 60, "前进500mm"),
                ]:
                    if not move(ser, direction, spd, target):
                        break
                    wait_motion_done(ser)
                    show_travel(ser, f"  {tag}")
                print(f"{G}[演示完成]{X}")

            else:
                print(f"{R}? {c}{X}  (f/b/l/r <值> [速度] | e | demo | q)")

        except (ValueError, IndexError):
            print(f"{R}参数错误，用法: f/b/l/r <值> [速度]{X}")
        except Exception as e:
            print(f"{R}{e}{X}")

    ser.write(frame(RESET))
    ser.close()
    print(f"{B}退出{X}")


if __name__ == "__main__":
    main()
