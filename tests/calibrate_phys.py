"""
ESP32-C3 物理参数自动标定（轮径 D、轴距 L）
用法:
  python tests/calibrate_phys.py [/dev/cu.usbmodemXXXX]

标定流程:
  1. 在平整地面用尺子/标记做参考线
  2. 选 1 进入"轮径标定"：
       - 让底盘从标记线前进 1000mm（或更长，标定精度更高）
       - 用尺子量实际走过的距离
       - 输入后回算 D = 62.0 × 1000 / 实际距离
  3. 选 2 进入"轴距标定"：
       - 让底盘原地转 10 圈（3600°）
       - 用地面标记或量角器测实际转过的总角度
       - 输入后回算 L = 160.0 × 3600 / 实际角度
  4. 输入 y 把新参数下发到固件（不持久化，掉电丢）

误差来源与可达到精度（标定后）:
  - 编码器量化: ±0.1°（单步）
  - 直线标定测量误差: 尺子 ±1mm → ±0.1%
  - 转向标定测量误差: 量角器 ±2° → ±0.06%
  - 综合: 90° 旋转预期 ±0.3° 以内，360° 旋转预期 ±1° 以内
"""

import math
import serial
import struct
import sys
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem1201"
BAUD = 115200

# 命令字
INIT, CONFIG, SET_PHYS, GET_PHYS = 0x01, 0x02, 0x03, 0x04
MOVE_DISTANCE, GET_ENCODER, GET_STATUS, RESET = 0x23, 0x22, 0x21, 0xFF
ACK, NACK, STATUS, PHYS_DATA = 0x80, 0x81, 0x91, 0x94

# 标称初始值
D_NOMINAL = 62.0   # mm
L_NOMINAL = 160.0  # mm
PPR = 4680

# 标定默认值
CAL_DISTANCE_MM = 1000.0   # 直线标定走多远
CAL_TURNS       = 10        # 转向标定转几圈
CAL_SPEED       = 40        # 标定速度（低速更稳）

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


def drain(ser, t=0.1):
    """排空接收缓冲，避免上一次响应干扰下一次命令"""
    deadline = time.time() + t
    while time.time() < deadline:
        ser.read(64)


def expect_ack(ser, timeout=1.0):
    f = recv(ser, timeout)
    if f and f["cmd"] == ACK:
        return True
    if f and f["cmd"] == NACK:
        err = f["data"][1] if len(f["data"]) > 1 else -1
        print(f"{R}NACK 错误码 0x{err:02x}{X}")
        return False
    return False


def init(ser):
    ser.write(frame(INIT))
    return expect_ack(ser)


def config(ser):
    ser.write(frame(CONFIG, struct.pack(">HH", PPR, 20000)))
    return expect_ack(ser)


def get_phys(ser):
    ser.write(frame(GET_PHYS))
    f = recv(ser)
    if f and f["cmd"] == PHYS_DATA and len(f["data"]) >= 4:
        d = struct.unpack(">H", f["data"][0:2])[0] * 0.1
        l = struct.unpack(">H", f["data"][2:4])[0] * 0.1
        return d, l
    return None, None


def set_phys(ser, d_mm, l_mm):
    d_q10 = int(round(d_mm * 10))
    l_q10 = int(round(l_mm * 10))
    if not (200 <= d_q10 <= 1000 and 500 <= l_q10 <= 5000):
        print(f"{R}参数超范围: D={d_mm}mm L={l_mm}mm{X}")
        return False
    ser.write(frame(SET_PHYS, struct.pack(">HH", d_q10, l_q10)))
    return expect_ack(ser)


def read_rpm(ser):
    ser.write(frame(GET_STATUS))
    f = recv(ser)
    if f and f["cmd"] == STATUS and len(f["data"]) >= 5:
        rpm1 = struct.unpack(">h", f["data"][1:3])[0]
        rpm2 = struct.unpack(">h", f["data"][3:5])[0]
        return rpm1, rpm2
    return None, None


def move(ser, direction, speed, target_raw, timeout=20.0):
    payload = bytes([direction, speed]) + struct.pack(">i", target_raw)
    ser.write(frame(MOVE_DISTANCE, payload))
    if not expect_ack(ser, 0.5):
        return False
    dl = time.time() + timeout
    while time.time() < dl:
        r = read_rpm(ser)
        if r and r[0] == 0 and r[1] == 0:
            return True
        time.sleep(0.1)
    return False


def calibrate_diameter(ser):
    """轮径标定：让底盘前进 CAL_DISTANCE_MM mm，量实际距离，反推真实 D"""
    print(f"\n{B}── 轮径标定 ──{X}")
    print(f"  即将让底盘前进 {CAL_DISTANCE_MM:.0f} mm（速度 {CAL_SPEED}）")
    print(f"  {Y}请在地面画一条起点线，让车头对齐起点。回车开始；或输入 mm 距离（默认 {CAL_DISTANCE_MM:.0f}）：{X}")
    line = input("  > 距离mm [回车用默认]: ").strip()
    try:
        dist = float(line) if line else CAL_DISTANCE_MM
    except ValueError:
        dist = CAL_DISTANCE_MM
    print(f"  开始移动 {dist:.0f} mm...")
    if not move(ser, 0, CAL_SPEED, int(dist)):
        print(f"{R}移动超时{X}")
        return None
    actual = input(f"  {Y}请用尺子量实际走过的距离 (mm): {X}").strip()
    try:
        actual_mm = float(actual)
    except ValueError:
        print(f"{R}输入无效{X}")
        return None
    if actual_mm < 50 or actual_mm > 5000:
        print(f"{R}距离异常 ({actual_mm} mm)，放弃{X}")
        return None
    d_real = D_NOMINAL * dist / actual_mm
    delta_pct = (d_real - D_NOMINAL) / D_NOMINAL * 100
    print(f"  {G}实际 D = {d_real:.2f} mm{X}（标称 {D_NOMINAL}, 偏差 {delta_pct:+.2f}%）")
    return d_real


def calibrate_wheelbase(ser):
    """轴距标定：让底盘原地转 N 圈，量实际转角，反推真实 L"""
    print(f"\n{B}── 轴距标定 ──{X}")
    print(f"  即将让底盘原地左转 {CAL_TURNS} 圈（共 {CAL_TURNS*360}°，速度 {CAL_SPEED}）")
    print(f"  {Y}请在地面画一条车头朝向线。回车开始；或输入圈数（默认 {CAL_TURNS}）：{X}")
    line = input("  > 圈数 [回车用默认]: ").strip()
    try:
        turns = float(line) if line else CAL_TURNS
    except ValueError:
        turns = CAL_TURNS
    deg_total = turns * 360
    target_raw = int(round(deg_total * 10))  # 0.1°
    print(f"  开始原地左转 {deg_total:.0f}°...")
    if not move(ser, 2, CAL_SPEED, target_raw, timeout=60.0):
        print(f"{R}转动超时{X}")
        return None
    actual = input(f"  {Y}请用地面标记/量角器量实际转过的总角度 (°)，例如 3590 或 3610: {X}").strip()
    try:
        actual_deg = float(actual)
    except ValueError:
        print(f"{R}输入无效{X}")
        return None
    if actual_deg < 100 or actual_deg > 7200:
        print(f"{R}角度异常 ({actual_deg}°)，放弃{X}")
        return None
    l_real = L_NOMINAL * deg_total / actual_deg
    delta_pct = (l_real - L_NOMINAL) / L_NOMINAL * 100
    print(f"  {G}实际 L = {l_real:.2f} mm{X}（标称 {L_NOMINAL}, 偏差 {delta_pct:+.2f}%）")
    return l_real


def main():
    print(f"\n{B}ESP32-C3 物理参数标定工具{X}  端口={PORT}")
    print(f"  标称: D={D_NOMINAL}mm  L={L_NOMINAL}mm  PPR={PPR}")

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
    drain(ser)
    if not config(ser):
        print(f"{R}CONFIG 失败{X}")
        ser.close()
        return
    drain(ser)

    d_cur, l_cur = get_phys(ser)
    if d_cur is None:
        d_cur, l_cur = D_NOMINAL, L_NOMINAL
    print(f"  固件当前: D={d_cur:.1f}mm  L={l_cur:.1f}mm\n")

    d_new, l_new = d_cur, l_cur

    while True:
        print(f"{B}菜单:{X}")
        print(f"  1) 标定轮径 D（当前 {d_cur:.2f}mm）")
        print(f"  2) 标定轴距 L（当前 {l_cur:.2f}mm）")
        print(f"  3) 手动设置 D 和 L")
        print(f"  4) 把新参数 D={d_new:.2f} L={l_new:.2f} 下发到固件")
        print(f"  5) 读当前固件参数")
        print(f"  q) 退出")
        c = input("> ").strip().lower()

        if c == "q":
            break
        elif c == "1":
            r = calibrate_diameter(ser)
            if r is not None:
                d_new = r
        elif c == "2":
            r = calibrate_wheelbase(ser)
            if r is not None:
                l_new = r
        elif c == "3":
            try:
                d = float(input("  D (mm): "))
                l = float(input("  L (mm): "))
                d_new, l_new = d, l
                print(f"  待下发: D={d_new:.2f}  L={l_new:.2f}")
            except ValueError:
                print(f"{R}输入无效{X}")
        elif c == "4":
            if set_phys(ser, d_new, l_new):
                d_cur, l_cur = d_new, l_new
                print(f"  {G}已下发 D={d_cur:.2f}mm  L={l_cur:.2f}mm{X}")
                print(f"  {Y}提示: 这些值在 RAM 中，掉电后丢失。{X}")
                print(f"       想永久生效，把固件里 WHEEL_DIAMETER_MM_DEFAULT / WHEELBASE_MM_DEFAULT")
                print(f"       改成 {d_cur:.2f} 和 {l_cur:.2f} 后重新烧录固件。")
            drain(ser)
        elif c == "5":
            d, l = get_phys(ser)
            if d is not None:
                d_cur, l_cur = d, l
                print(f"  D={d_cur:.2f}mm  L={l_cur:.2f}mm")

    ser.write(frame(RESET))
    ser.close()
    print(f"{B}退出{X}")


if __name__ == "__main__":
    main()
