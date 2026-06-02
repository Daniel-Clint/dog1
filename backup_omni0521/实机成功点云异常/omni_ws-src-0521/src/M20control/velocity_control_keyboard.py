import socket
import json
import time
import sys
import select
import termios
import tty

# --- 机器人配置 ---
SERVER_IP = "10.21.31.103" 
PORT = 30000

# --- 速度配置 ---
SPEED_X = 0.3     # 前后 (15%)
SPEED_Y = 0.50     # 左右平移 (30%，两倍速)
ROT_SPEED = 0.50  # 旋转 (25%)

msg = """
===  机器狗遥控器 ===
控制说明：
---------------------------
   Q   W   E
   A   S   D
---------------------------
按住不放移动，松开按键即停止。
Ctrl+C: 安全退出
"""

def build_header(data_len):
    h = bytearray(16)
    h[0], h[1], h[2], h[3] = 0xeb, 0x91, 0xeb, 0x90
    h[4], h[5] = data_len & 0xFF, (data_len >> 8) & 0xFF
    h[6], h[7] = 1, 0
    h[8] = 0x01
    return h

def send_velocity(sock, x=0.0, y=0.0, yaw=0.0):
    t_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    payload = {
        "PatrolDevice": {
            "Type": 2, "Command": 21, "Time": t_str,
            "Items": {"X": float(x), "Y": float(y), "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": float(yaw)}
        }
    }
    j_str = json.dumps(payload, separators=(',', ':')).encode('utf-8')
    pkt = build_header(len(j_str)) + j_str
    sock.sendto(pkt, (SERVER_IP, PORT))

def get_key(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    # select 等待用户输入，超过 timeout 则返回空
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = None
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print(msg)
    
    try:
        while True:
            # 等待 0.15 秒，如果没有按键输入，则 key 为 None
            # 这个 0.15s 是为了衔接键盘自动连发的间隔
            key = get_key(settings, timeout=0.15)
            
            if key is not None:
                if key == '\x03': # Ctrl+C
                    break
                
                k = key.lower()
                vx, vy, vyaw = 0.0, 0.0, 0.0
                status = "停止"

                if k == 'w':
                    vx, status = SPEED_X, "前进"
                elif k == 's':
                    vx, status = -SPEED_X, "后退"
                elif k == 'a':
                    vy, status = SPEED_Y, "左移"
                elif k == 'd':
                    vy, status = -SPEED_Y, "右移"
                elif k == 'q':
                    vyaw, status = ROT_SPEED, "左转"
                elif k == 'e':
                    vyaw, status = -ROT_SPEED, "右转"
                
                # 打印当前状态
                sys.stdout.write(f"\r状态: [{status.ljust(4)}] 按键中...        ")
                send_velocity(sock, vx, vy, vyaw)
            else:
                # 超时未收到按键，说明用户松开了手
                sys.stdout.write(f"\r状态: [停止] 已松开按键        ")
                send_velocity(sock, 0.0, 0.0, 0.0)
            
            sys.stdout.flush()

    except Exception as e:
        print(f"\n错误: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n\n🛑 已断开遥控，确保机器人停止。")
        for _ in range(5):
            send_velocity(sock, 0.0, 0.0, 0.0)
            time.sleep(0.05)
        sock.close()

if __name__ == "__main__":
    main()
