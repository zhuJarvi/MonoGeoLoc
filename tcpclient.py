import socket
import sys

def tcp_client(host: str = "127.0.0.1", port: int = 8888):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        try:
            sock.connect((host, port))
            print(f"connected to {host}:{port}")

            while True:
                data = sock.recv(1024)
                if not data:
                    break
                
                if len(data) == 13 and data[0] == 0x59 and data[1] == 0x74 and data[11] == 0xED and data[12] == 0xED:
                    yaw_data = int(data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5]
                    pitch_data = int(data[6] << 24) | (data[7] << 16) | (data[8] << 8) | data[9]
                    if yaw_data >= 0x80000000:  
                        yaw_data = yaw_data - 0x100000000  
                    if pitch_data >= 0x80000000: 
                        pitch_data = pitch_data - 0x100000000  
                    print(f"收到响应：yaw={yaw_data*0.00001}, pitch={pitch_data*0.00001}")


        except ConnectionRefusedError:
            print(f"failed to connect to {host}:{port} - server not running or connection refused")
        except Exception as e:
            print(f"error occurred: {e}")

if __name__ == "__main__":
    if len(sys.argv) == 3:
        HOST = sys.argv[1]
        PORT = int(sys.argv[2])
    else:
        HOST = "192.168.147.128"
        PORT = 2424

    tcp_client(HOST, PORT)