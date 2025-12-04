import serial
import time
import threading
from datetime import datetime

class SERIAL_SYNC:
    def __init__(self, PORT, BAUD_RATE=115200, event=None, loop_time=50, timeout=100, try_timeout=200, verbose=False):
        self.buf = b''
        self.connect = False
        self.last_get_time = time.time() * 1000
        self.last_send_time = 0
        self.last_try_time = 0
        self.loop_time = loop_time
        self.timeout = timeout
        self.try_timeout = try_timeout
        self.Event = event
        self.PORT = PORT
        self.BAUD_RATE = BAUD_RATE
        self.verbose = verbose
        try:
            self.try_connect()
            self.connect = True
        except:
            self.connect = False
    
    def try_connect(self):
        self.ser = serial.Serial(self.PORT, self.BAUD_RATE, timeout = 0, write_timeout=0.01, rtscts=True)
        self.last_get_time = time.time() * 1000
    
    def update(self):
        current_time = time.time() * 1000
        if not self.connect:
            if current_time-self.last_try_time>self.try_timeout:
                if (self.verbose): print("Disconnect! Retry to connect.")
                self.last_try_time = current_time
                try:
                    self.try_connect()
                    self.connect = True
                except:
                    self.close()
                    return
            else:
                return
        if current_time-self.last_send_time>self.loop_time:
            self.write(0,[],0)
        
        try:
            if self.ser.in_waiting:
                self.buf += self.ser.read(self.ser.in_waiting)
            while True:
                if b'E;' in self.buf and b'D\t' in self.buf:
                    start_index = self.buf.find(b'D\t')
                    if start_index<0:
                        break
                    end_index = self.buf.find(b'E;')
                    if end_index<0:
                        break
                    data_raw = self.buf[(start_index+2):end_index]
                    self.buf=self.buf[end_index+2:]
                    
                    hash = -data_raw[-1]
                    for byte in data_raw:
                        hash += byte
                        hash %= 256
                    if data_raw[-1]==hash:
                        self.last_get_time = current_time

                        id = int(data_raw[0:4], 16)
                        len = int(data_raw[4:5], 16)
                        data=b''
                        for i in range(len):
                            data+=(int(data_raw[5+i*2:5+(i+1)*2], 16).to_bytes(1, 'big'))

                        if self.Event and id!=0:
                            try:
                                self.Event(id, data, len)
                            except Exception as e:
                                print(e)
                else:
                    self.buf=self.buf[-512:]
                    break
        except:
            # self.close()
            pass
        
        if (current_time - self.last_get_time > self.timeout):
            print("NOOOO")
            self.close()
    
    def write(self, id, data, len=8):
        if not self.connect:
            return
        try:
            sendBuf = b''
            sendBuf+=f'{id:04X}{len%16:X}'.lower().encode()
            if type(data) == bytes:
                for i in range(len):
                    sendBuf+=f'{data[i]%256:02X}'.lower().encode()
            elif type(data) == list:
                for i in range(len):
                    if type(data[i]) == bytes: sendBuf+=f'{data[i][0]%256:02X}'.lower().encode()
                    elif type(data[i]) == int:  sendBuf+=f'{data[i]%256:02X}'.lower().encode()
                    else: raise ValueError("不合法輸入")
            hash = 0
            for byte in sendBuf:
                hash = (hash + byte)%256

            sendBuf += hash.to_bytes(1, 'little', signed=False)
            sendBuf = b'D\t'+sendBuf+b'E;'

            self.ser.write(sendBuf)
            self.last_send_time=time.time() * 1000
        except:
            # self.close()
            pass

    def ok(self):
        if not self.connect:
            return False
        if (time.time() * 1000-self.last_get_time>self.timeout):
            return False
        return True
    
    def close(self):
        self.connect = False
        try:
            if self.ser:
                self.ser.close()
                self.ser=None
        except:
            pass
