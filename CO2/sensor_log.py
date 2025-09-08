from serial import Serial
import csv
from datetime import datetime

ser = Serial('COM4', 9600)   # serial port

def get_csv_writer():
    f = open('gas_data2.csv', 'a', newline='', encoding='utf-8')
    writer = csv.writer(f)
    if f.tell() == 0:
        writer.writerow(['time', 'co2', 'voc', 'co21', 'voc1'])
    return f, writer

if __name__ == '__main__':
    f, writer = get_csv_writer()
    count = 0
    caps = [''] * 4
    flag = 0
    while count<10000000:
        line = ser.readline().decode('utf-8')[:-1]
        # print(line)
        if "m): " in line:
            flag = 100
        for i in range(4):
            if i == 0:
                identifier = 'CO2 equ(ppm): '
            if i == 1:
                identifier = 'CO21 equ(ppm): '
            if i == 2:
                identifier = 'VOC(isobutylene) equ(ppb): '
            if i == 3:
                identifier = 'VOC1(isobutylene) equ(ppb): '
            idx = line.find(identifier)
            # print(identifier)
            # print(idx)
            if idx != -1:
                val = line.split(': ')[1]
                caps[i] = val[:-1]
               
        f.flush()
        count += 1 # comment this line to run indefinitely
        flag += 1
        if flag == 104:
            now = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            writer.writerow([now] + caps)
    f.close()