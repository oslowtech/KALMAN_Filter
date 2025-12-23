import serial
import numpy as np
import matplotlib.pyplot as plt
from time import time, sleep

SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
TIMEOUT = 1
DT = 0.1

def init_kalman():
    A = np.array([
        [1, DT],
        [0, 1]
    ])
    B = np.array([
        [0.5 * DT * DT],
        [DT]
    ])
    H = np.array([[1, 0]])
    Q = np.array([
        [0.01, 0],
        [0, 0.1]
    ])
    R = np.array([[9]])
    x = np.array([[0], [0]])
    P = np.eye(2)
    return A, B, H, Q, R, x, P

def kalman_predict(x, P, A, B, Q, u):
    x_pred = A @ x + B @ u
    P_pred = A @ P @ A.T + Q
    return x_pred, P_pred

def kalman_update(x, P, H, R, z):
    y = z - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x_new = x + K @ y
    P_new = (np.eye(2) - K @ H) @ P
    return x_new, P_new

def parse_serial_line(line):
    parts = line.replace("ALT:", "").replace("ACC:", "").split()
    altitude = float(parts[0])
    acceleration = float(parts[1])
    return altitude, acceleration

def run():
    A, B, H, Q, R, x, P = init_kalman()
    
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
    
    time_data = []
    raw_altitude = []
    filtered_altitude = []
    start = time()
    
    try:
        while True:
            line = ser.readline().decode().strip()
            
            if not line.startswith("ALT:"):
                continue
            
            try:
                alt, acc = parse_serial_line(line)
                
                u = np.array([[acc]])
                x, P = kalman_predict(x, P, A, B, Q, u)
                
                z = np.array([[alt]])
                x, P = kalman_update(x, P, H, R, z)
                
                elapsed = time() - start
                time_data.append(elapsed)
                raw_altitude.append(alt)
                filtered_altitude.append(x[0, 0])
                
                print(f"Alt: {x[0,0]:.2f} m | Vel: {x[1,0]:.2f} m/s")
                
                sleep(DT)
                
            except:
                pass
                
    except KeyboardInterrupt:
        ser.close()
        
        plt.figure(figsize=(10, 5))
        plt.plot(time_data, raw_altitude, linestyle="dotted", label="Raw")
        plt.plot(time_data, filtered_altitude, linewidth=2, label="Filtered")
        plt.xlabel("Time (s)")
        plt.ylabel("Altitude (m)")
        plt.title("Kalman Filter - ESP32")
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    run()
