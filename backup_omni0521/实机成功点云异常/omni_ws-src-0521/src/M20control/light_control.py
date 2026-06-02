import socket
import json
import time

def build_protocol_header(data_length: int, msg_id: int = 1, asdu_format: int = 0x01) -> bytearray:
    header = bytearray(16)
    header[0] = 0xeb
    header[1] = 0x91
    header[2] = 0xeb
    header[3] = 0x90
    header[4] = data_length & 0xFF
    header[5] = (data_length >> 8) & 0xFF
    header[6] = msg_id & 0xFF
    header[7] = (msg_id >> 8) & 0xFF
    header[8] = asdu_format
    return header

SERVER_IP = "10.21.31.103"
PORT = 30000

FRONT_LIGHT = 0
BACK_LIGHT = 0

current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

asdu_dict = {
	"PatrolDevice": {
		"Type": 1101,
		"Command": 2,
		"Time": "2023-01-01 00:00:00",
		"Items": {
             "Front": 0,
             "Back": 1
		}
	}

      	}



asdu_data = json.dumps(asdu_dict, separators=(',', ':')).encode('utf-8')
data_length = len(asdu_data)

header = build_protocol_header(data_length=data_length, msg_id=1, asdu_format=0x01)
message = header + asdu_data

try:
    client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_sock.sendto(message, (SERVER_IP, PORT))
    print("Light control command sent successfully!")
    print("Expected status: Front [1], Back [1]")
except Exception as e:
    print("Send failed:", e)
finally:
    client_sock.close()
