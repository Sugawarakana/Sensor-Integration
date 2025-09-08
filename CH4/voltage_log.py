import serial

ser = serial.Serial('COM3', 115200)  # 根据实际串口号和波特率修改
with open('serial_output.txt', 'w', encoding='utf-8') as f:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore')
        print(line, end='')  # 显示在控制台
        f.write(line)        # 写入文件
        f.flush()          # 实时写入文件