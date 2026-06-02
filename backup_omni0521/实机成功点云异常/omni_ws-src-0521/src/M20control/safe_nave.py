import socket
import json
import time
import math

SERVER_IP = "10.21.31.103" # 根据你的实际IP修改
PORT = 30000

def build_header(data_len):
    h = bytearray(16)
    h[0], h[1], h[2], h[3] = 0xeb, 0x91, 0xeb, 0x90
    h[4], h[5] = data_len & 0xFF, (data_len >> 8) & 0xFF
    h[6], h[7] = 1, 0
    h[8] = 0x01
    return h

def send_cmd(sock, msg_type, cmd, items):
    t_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    payload = {
        "PatrolDevice": {
            "Type": msg_type,
            "Command": cmd,
            "Time": t_str,
            "Items": items
        }
    }
    j_str = json.dumps(payload, separators=(',', ':')).encode('utf-8')
    pkt = build_header(len(j_str)) + j_str
    sock.sendto(pkt, (SERVER_IP, PORT))

def recv_resp(sock):
    try:
        data, _ = sock.recvfrom(4096)
        if len(data) > 16:
            return json.loads(data[16:].decode('utf-8'))
    except Exception:
        pass
    return None

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    
    print("=== 机器狗 0.2m 相对位移测试 ===")
    
    # ---------------------------------------------------------
    # 1. 获取当前位置 (协议 1007/2)
    # ---------------------------------------------------------
    print("\n[1/3] 正在获取当前地图坐标...")
    send_cmd(sock, 1007, 2, {})
    resp = recv_resp(sock)
    
    if not resp or "Items" not in resp.get("PatrolDevice", {}):
        print("❌ 无法获取当前位置，请检查网络或机器人定位状态！")
        return
        
    items = resp["PatrolDevice"]["Items"]
    if items.get("Location", 1) == 1:
        print("❌ 机器人定位丢失 (Location=1)，无法执行导航任务！")
        return
        
    curr_x = items.get("PosX", 0.0)
    curr_y = items.get("PosY", 0.0)
    curr_yaw = items.get("Yaw", 0.0)
    print(f"✅ 当前坐标 -> X: {curr_x:.3f}, Y: {curr_y:.3f}, Yaw: {curr_yaw:.3f} rad")

    # ---------------------------------------------------------
    # 2. 计算正前方 0.2m 的绝对坐标
    # ---------------------------------------------------------
    distance = 0.4
    target_x = curr_x + distance * math.cos(curr_yaw)
    target_y = curr_y + distance * math.sin(curr_yaw)
    target_yaw = curr_yaw # 保持朝向不变
    
    print(f"\n[2/3] 计算目标坐标 -> X: {target_x:.3f}, Y: {target_y:.3f}")
    
    # ---------------------------------------------------------
    # 3. 下发单点导航任务 (协议 1003/1)
    # ---------------------------------------------------------
    nav_items = {
        "Value": 999,          # 临时测试点编号
        "MapID": 0,
        "PosX": target_x,
        "PosY": target_y,
        "PosZ": 0.0,
        "AngleYaw": target_yaw,
        "PointInfo": 1,        # 1=任务点 (到点精度高，到达后停止)
        "Gait": 0x3002,        # 12290 平地步态
        "Speed": 1,            # 1=低速
        "Manner": 0,           # 0=前进行走
        "ObsMode": 0,          # 0=开启避障功能
        "NavMode": 0           # 0=直线导航 (短距离移动推荐直线，更安全)
    }
    
    print("\n[3/3] 准备发送移动指令。")
    print("⚠️  警告：按 [Ctrl+C] 可随时触发紧急停止！")
    time.sleep(2) # 给用户反应时间
    
    send_cmd(sock, 1003, 1, nav_items)
    
    # 获取任务下发响应
    ack_resp = recv_resp(sock)
    if ack_resp:
        ack_items = ack_resp["PatrolDevice"]["Items"]
        if ack_items.get("ErrorCode", 0) != 0:
            err = ack_items.get("ErrorCode")
            print(f"❌ 任务下发失败，错误码: {hex(err)}")
            return
            
    print("✅ 移动指令已下发，开始监控状态...")
    
    # ---------------------------------------------------------
    # 4. 循环监控导航状态 (协议 1007/1)
    # ---------------------------------------------------------
    try:
        while True:
            send_cmd(sock, 1007, 1, {})
            status_resp = recv_resp(sock)
            
            if status_resp:
                st_items = status_resp["PatrolDevice"]["Items"]
                status = st_items.get("Status", 0)
                err = st_items.get("ErrorCode", 0)
                
                if err != 0:
                    print(f"⚠️ 导航异常终止，错误码: {hex(err)}")
                    break
                    
                if status == 3:
                    print("🔄 状态: [3] 正在导航中...")
                elif status == 4:
                    print("🎯 状态: [4] 导航完成！成功到达目标点。")
                    break
                elif status == 2:
                    print("⏳ 状态: [2] 导航预处理中...")
                    
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        # ---------------------------------------------------------
        # 5. 紧急停止 (协议 1004/1)
        # ---------------------------------------------------------
        print("\n\n🛑 检测到 Ctrl+C！触发紧急停止！")
        send_cmd(sock, 1004, 1, {})
        print("✅ 已发送取消导航指令 (1004/1)。机器人应立即停止。")
        
    finally:
        sock.close()

if __name__ == "__main__":
    main()
