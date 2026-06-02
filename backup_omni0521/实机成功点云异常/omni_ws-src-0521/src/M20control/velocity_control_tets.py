import socket
import json
import time

# --- 请确认你的机器人 IP ---
SERVER_IP = "10.21.31.103" 
PORT = 30000

def build_header(data_len):
    h = bytearray(16)
    h[0], h[1], h[2], h[3] = 0xeb, 0x91, 0xeb, 0x90
    h[4], h[5] = data_len & 0xFF, (data_len >> 8) & 0xFF
    h[6], h[7] = 1, 0
    h[8] = 0x01
    return h

def send_velocity(sock, x=0.0, y=0.0, yaw=0.0):
    """发送单次速度指令"""
    t_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    payload = {
        "PatrolDevice": {
            "Type": 2,
            "Command": 21,
            "Time": t_str,
            "Items": {
                "X": float(x),
                "Y": float(y),
                "Z": 0.0,
                "Roll": 0.0,
                "Pitch": 0.0,
                "Yaw": float(yaw)
            }
        }
    }
    j_str = json.dumps(payload, separators=(',', ':')).encode('utf-8')
    pkt = build_header(len(j_str)) + j_str
    sock.sendto(pkt, (SERVER_IP, PORT))

def move_for_duration(sock, action_name, duration, x, y, yaw):
    """
    以 20Hz 的频率连续发送速度指令
    """
    print(f"▶️ 开始动作: {action_name} | X:{x}, Y:{y}, Yaw:{yaw} | 持续 {duration}秒")
    hz = 20
    interval = 1.0 / hz
    steps = int(duration * hz)
    
    for _ in range(steps):
        send_velocity(sock, x, y, yaw)
        time.sleep(interval)
        
    # 动作结束后，发送 0 速度使其刹车
    for _ in range(3):
        send_velocity(sock, 0.0, 0.0, 0.0)
        time.sleep(interval)
    print(f"⏹️ {action_name} 结束，原地待命。")

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print("=== 机器狗全向运动综合测试 ===")
    print("⚠️  请确保切换至 [常规控制模式]！")
    print("⚠️  测试将在 3 秒后开始，按 Ctrl+C 可紧急停止...\n")
    time.sleep(3)
    
    # 设定测试速度 (15% 比例)
    SPEED = 0.3
    SPEED1=0.5
    ROT_SPEED = 0.50 # 旋转速度可以稍微大一点点，防止摩擦力太大转不动
    
    try:
        # 1. 向前
        move_for_duration(sock, "向前走 (X正向)", 2.0, x=SPEED, y=0.0, yaw=0.0)
        time.sleep(1.0)
        
        # 2. 向后
        move_for_duration(sock, "向后退 (X负向)", 2.0, x=-SPEED, y=0.0, yaw=0.0)
        time.sleep(1.0)
        
        # 3. 向左平移
        move_for_duration(sock, "向左平移 (Y正向)", 2.0, x=0.0, y=SPEED1, yaw=0.0)
        time.sleep(1.0)
        
        # 4. 向右平移
        move_for_duration(sock, "向右平移 (Y负向)", 2.0, x=0.0, y=-SPEED1, yaw=0.0)
        time.sleep(1.0)
        
        # 5. 原地逆时针旋转 (左转)
        move_for_duration(sock, "逆时针旋转 (Yaw正向)", 2.0, x=0.0, y=0.0, yaw=ROT_SPEED)
        time.sleep(1.0)
        
        # 6. 原地顺时针旋转 (右转)
        move_for_duration(sock, "顺时针旋转 (Yaw负向)", 2.0, x=0.0, y=0.0, yaw=-ROT_SPEED)
        
        print("\n✅ 所有动作测试完毕！")

    except KeyboardInterrupt:
        print("\n\n🛑 [紧急停止] 检测到 Ctrl+C！正在刹车！")
    finally:
        # 确保安全，退出前狂发停止指令
        for _ in range(10):
            send_velocity(sock, x=0.0, y=0.0, yaw=0.0)
            time.sleep(0.05)
        sock.close()

if __name__ == "__main__":
    main()
