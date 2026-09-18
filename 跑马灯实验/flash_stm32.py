# -*- coding: utf-8 -*-
"""
STM32 串口 ISP 烧录脚本 (AN3155 bootloader 协议)
针对正点原子 STM32F103 开发板的 CH340 一键下载电路。
用法:
    python flash_stm32.py probe            # 只探测: 进入bootloader并读取芯片ID
    python flash_stm32.py flash            # 正式烧录 hex
"""
import sys, time, struct
import serial

PORT = "COM5"
BAUD = 115200
HEX_FILE = r"C:\Users\Administrator\Desktop\跑马灯实验\Output\atk_f103.hex"
FLASH_BASE = 0x08000000
DEVICE_ID_F103_HD = 0x0414  # STM32F103 高密度 (512K) 芯片ID

ACK = 0x79
NACK = 0x1F


def open_port():
    # 8 数据位, 偶校验, 1 停止位 (bootloader 要求 8E1)
    ser = serial.Serial(port=PORT, baudrate=BAUD, bytesize=8,
                        parity=serial.PARITY_EVEN, stopbits=1,
                        timeout=1.0)
    return ser


def sync(ser):
    """发送 0x7F 并等待 ACK, 返回是否成功"""
    ser.reset_input_buffer()
    for _ in range(5):
        ser.write(b"\x7F")
        r = ser.read(1)
        if r == bytes([ACK]):
            return True
    return False


def send_cmd(ser, cmd):
    """发送命令(带异或校验)并等待 ACK"""
    ser.reset_input_buffer()
    ser.write(bytes([cmd, cmd ^ 0xFF]))
    r = ser.read(1)
    return r == bytes([ACK])


def read_len_data(ser):
    """读一个长度字节 N, 再读 N+1 字节数据, 返回数据bytes"""
    n = ser.read(1)
    if not n:
        return None
    data = ser.read(n[0] + 1)
    return data


def get_device_id(ser):
    if not send_cmd(ser, 0x02):
        return None
    d = read_len_data(ser)
    if d is None or len(d) < 2:
        return None
    return (d[0] << 8) | d[1]


def get_version(ser):
    if not send_cmd(ser, 0x00):
        return None
    return read_len_data(ser)


def enter_bootloader(ser, rts_val, dtr_reset_state):
    """
    用一键下载电路把芯片弄进 bootloader.
    正点原子 CH340 一键下载电路(DTR#/RTS# 低电平有效):
        RTS -> BOOT0,  DTR -> NRST(复位)
    这里枚举极性组合. 返回 None
    """
    ser.setRTS(rts_val)                 # 选择 BOOT0 电平
    ser.setDTR(dtr_reset_state)         # 先保持复位
    time.sleep(0.1)
    ser.setDTR(not dtr_reset_state)     # 释放复位
    time.sleep(0.2)
    return None


def try_enter_bootloader(ser):
    """依次尝试所有 DTR/RTS 极性组合, 返回成功进入时(已同步)的策略描述"""
    combos = [
        (False, False, "RTS低(BOOT0=1),DTR低复位"),
        (True,  False, "RTS高(BOOT0=1),DTR低复位"),
        (False, True,  "RTS低(BOOT0=1),DTR高复位"),
        (True,  True,  "RTS高(BOOT0=1),DTR高复位"),
    ]
    for rts_val, dtr_reset, desc in combos:
        enter_bootloader(ser, rts_val, dtr_reset)
        if sync(ser):
            return desc
    return None


def probe():
    ser = open_port()
    # 先什么都不做, 看是否已经在 bootloader 里
    if sync(ser):
        vid = get_version(ser)
        pid = get_device_id(ser)
        print(f"[OK] 芯片已在 bootloader 模式")
        print(f"     版本信息: {vid.hex() if vid else '?'}")
        print(f"     芯片 ID: 0x{pid:04X}" if pid else "     芯片 ID: 读取失败")
        ser.close()
        return 0
    # 尝试一键下载策略
    desc = try_enter_bootloader(ser)
    if desc:
        pid = get_device_id(ser)
        print(f"[OK] 进入 bootloader ({desc})")
        print(f"     芯片 ID: 0x{pid:04X}" if pid else "     芯片 ID: 读取失败")
        ser.close()
        return 0
    ser.close()
    print("[FAIL] 无法与 bootloader 同步")
    print("       请检查: 1) 板子已上电  2) COM5 串口  3) 或手动把 BOOT0 跳到 1 后按复位键")
    return 1


def parse_hex(path):
    """解析 Intel HEX, 返回 {addr: byte}"""
    data = {}
    ext_lin_addr = 0
    min_addr = 0xFFFFFFFF
    max_addr = 0
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line[0] != ":":
                continue
            ln = int(line[1:3], 16)
            addr = int(line[3:7], 16)
            rtype = int(line[7:9], 16)
            if rtype == 0x04:
                ext_lin_addr = int(line[9:13], 16) << 16
            elif rtype == 0x00:
                a = ext_lin_addr + addr
                for i in range(ln):
                    data[a + i] = int(line[9 + 2*i:11 + 2*i], 16)
                if a < min_addr:
                    min_addr = a
                if a + ln - 1 > max_addr:
                    max_addr = a + ln - 1
            elif rtype == 0x01:
                break
    return data, min_addr, max_addr


def erase(ser):
    if not send_cmd(ser, 0x43):  # 全片擦除 (mass erase)
        return False
    return True


def write_block(ser, addr, block):
    """写一个数据块 (最多256字节), 要求4字节对齐"""
    if not send_cmd(ser, 0x31):
        return False
    # 地址 4 字节 + 异或校验
    a = struct.pack(">I", addr)
    ser.write(a + bytes([a[0] ^ a[1] ^ a[2] ^ a[3]]))
    if ser.read(1) != bytes([ACK]):
        return False
    # 长度 N(=len-1) + 数据 + 异或校验
    n = len(block) - 1
    chk = n
    for b in block:
        chk ^= b
    ser.write(bytes([n]) + bytes(block) + bytes([chk]))
    if ser.read(1) != bytes([ACK]):
        return False
    return True


def go(ser, addr):
    if not send_cmd(ser, 0x21):
        return False
    a = struct.pack(">I", addr)
    ser.write(a + bytes([a[0] ^ a[1] ^ a[2] ^ a[3]]))
    return ser.read(1) == bytes([ACK])


def do_flash():
    data, min_addr, max_addr = parse_hex(HEX_FILE)
    print(f"[..] hex 解析: 地址范围 0x{min_addr:08X} ~ 0x{max_addr:08X}, {len(data)} 字节")

    ser = open_port()
    entered = False
    if sync(ser):
        entered = True
        print("[OK] 芯片已在 bootloader 模式")
    else:
        desc = try_enter_bootloader(ser)
        if desc:
            entered = True
            print(f"[OK] 进入 bootloader ({desc})")

    if not entered:
        print("[FAIL] 无法进入 bootloader, 请手动 BOOT0=1 后按复位")
        ser.close()
        return 1

    pid = get_device_id(ser)
    print(f"[..] 芯片 ID: 0x{pid:04X}" if pid else "[..] 芯片 ID 读取失败")
    if pid and pid != DEVICE_ID_F103_HD:
        print(f"[WARN] 芯片 ID 0x{pid:04X} 与预期 0x{DEVICE_ID_F103_HD:04X} 不同, 继续...")

    print("[..] 全片擦除...")
    if not erase(ser):
        print("[FAIL] 擦除失败")
        ser.close()
        return 1
    print("[OK] 擦除完成")

    # 按 256 字节对齐写
    start = min_addr & ~0xFF
    end = (max_addr + 0xFF) & ~0xFF
    print(f"[..] 写入 0x{start:08X} ~ 0x{end:08X} ...")
    for a in range(start, end, 256):
        block = bytes([data.get(a + i, 0xFF) for i in range(256)])
        if not write_block(ser, a, block):
            print(f"[FAIL] 写入 0x{a:08X} 失败")
            ser.close()
            return 1
    print(f"[OK] 写入完成, 共 {(end - start) // 256} 个块")

    # 校验(可选): 读回前几个字节
    print("[..] 跳转到用户程序 0x%08X ..." % FLASH_BASE)
    if not go(ser, FLASH_BASE):
        print("[FAIL] 跳转失败")
        ser.close()
        return 1
    print("[OK] 烧录完成, 芯片已重启运行")
    # 释放 BOOT0(切回 RTS低=BOOT0=0), 复位
    ser.setRTS(False)
    ser.setDTR(False)
    time.sleep(0.05)
    ser.setDTR(True)
    ser.close()
    return 0


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "probe"
    if mode == "probe":
        sys.exit(probe())
    elif mode == "flash":
        sys.exit(do_flash())
    else:
        print("用法: python flash_stm32.py [probe|flash]")
        sys.exit(2)
