#!/usr/bin/env python3
import tkinter as tk
from tkinter import messagebox
import time
from socket import *


PLC_IP = "192.168.1.3"  
PLC_PORT = 8501 

BUFSIZE = 4096

class kvHostLink:
    addr = ()
    destfins = []
    srcfins = []
    port = PLC_PORT
    

    def __init__(self, host):
        self.addr = host, self.port

    def sendrecive(self, command):
        s = socket(AF_INET, SOCK_DGRAM)
        s.settimeout(2)

        s.sendto(command, self.addr)
        rcvdata = s.recv(BUFSIZE)

        return rcvdata

    def mode(self, mode):
        senddata = 'M' + mode
        rcv = self.sendrecive((senddata + '\r').encode())
        return rcv

    def unittype(self):
        rcv = self.sendrecive("?k\r".encode())
        return rcv

    def errclr(self):
        senddata = 'ER'
        rcv = self.sendrecive((senddata + '\r').encode())
        return rcv

    def er(self):
        senddata = '?E'
        rcv = self.sendrecive((senddata + '\r').encode())
        return rcv

    def settime(self):
        dt_now = datetime.datetime.now()
        senddata = 'WRT ' + str(dt_now.year)[2:]
        senddata = senddata + ' ' + str(dt_now.month)
        senddata = senddata + ' ' + str(dt_now.day)
        senddata = senddata + ' ' + str(dt_now.hour)
        senddata = senddata + ' ' + str(dt_now.minute)
        senddata = senddata + ' ' + str(dt_now.second)
        senddata = senddata + ' ' + dt_now.strftime('%w')
        rcv = self.sendrecive((senddata + '\r').encode())
        return rcv
        
    def set(self, address):
        rcv = self.sendrecive(('ST ' + address + '\r').encode())
        return rcv

    def reset(self, address):
        rcv = self.sendrecive(('RS ' + address + '\r').encode())
        return rcv

    def sts(self, address, num):
        rcv = self.sendrecive(('STS ' + address + ' ' + str(num) + '\r').encode())
        return rcv

    def rss(self, address, num):
        rcv = self.sendrecive(('RSS ' + address + ' ' + str(num) + '\r').encode())
        return rcv

    def read(self, addresssuffix):
        rcv = self.sendrecive(('RD ' + addresssuffix + '\r').encode())
        return rcv

    def reads(self, addresssuffix, num):
        rcv = self.sendrecive(('RDS ' + addresssuffix + ' ' + str(num) + '\r').encode())
        return rcv

    def write(self, addresssuffix, data):
        rcv = self.sendrecive(('WR ' + addresssuffix + ' ' + data + '\r').encode())
        return rcv

    def writs(self, addresssuffix, num, data):
        rcv = self.sendrecive(('WRS ' + addresssuffix + ' ' + str(num) + ' ' + data + '\r').encode())
        return rcv

    def mws(self, addresses):
        rcv = self.sendrecive(('MWS ' + addresses + '\r').encode())
        return rcv

    def mwr(self):
        rcv = self.sendrecive(('MWR\r').encode())
        return rcv


class App:
    def __init__(self, master):
        self.master = master
        master.title("Communication")  # ウィンドウのタイトル
        
        # 入力用テキストボックス（数値1）
        self.label1 = tk.Label(master, text="数値1:")
        self.label1.grid(row=1, column=0, padx=5, pady=5)  # ラベルを左に配置
        self.entry1 = tk.Entry(master)
        self.entry1.grid(row=1, column=1, padx=5, pady=5)  # テキストボックスを右に配置
        self.entry1.insert(0, "1")  # 初期値を1に設定

        # 入力用テキストボックス（数値2）
        self.label2 = tk.Label(master, text="数値2:")
        self.label2.grid(row=2, column=0, padx=5, pady=5)  # ラベルを左に配置
        self.entry2 = tk.Entry(master)
        self.entry2.grid(row=2, column=1, padx=5, pady=5)  # テキストボックスを右に配置
        self.entry2.insert(0, "1")  # 初期値を1に設定

        # 送信ボタン
        self.send_button = tk.Button(master, text="送信", command=self.send_values)
        self.send_button.grid(row=3, columnspan=2, pady=10)  # ボタンを中央に配置

        # 結果表示用テキストボックス
        self.result_label = tk.Label(master, text="結果:")
        self.result_label.grid(row=4, column=0, padx=5, pady=5)  # ラベルを左に配置
        self.result_entry = tk.Entry(master, state='readonly')
        self.result_entry.grid(row=4, column=1, padx=5, pady=5)  # テキストボックスを右に配置

        # 通信オブジェクト
        # self.mcp = mcprotocol()

    def send_values(self):
        try:
            

            num1 = self.entry1.get()
            num2 = self.entry2.get()

            print(f"Sending values to PLC: num1={num1}, num2={num2}")
            # 上位リンクでの通信＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
            self.kv = kvHostLink('192.168.1.3')
            data = self.kv.writs('EM1',2,num1 + ' ' + num2)
            # data = self.kv.write('EM1', num1)
            data = self.kv.set('MR100')
            time.sleep(0.2)
            result = self.kv.read('EM3').decode('utf-8')
            data = self.kv.reset('MR100' )
            # ここまで＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
            print(f"Result to PLC:", result)

            # 結果を表示
            result_value = int(result)
            self.result_entry.config(state='normal')  # 一時的に編集可能にする
            self.result_entry.delete(0, tk.END)  # クリア
            self.result_entry.insert(0, str(result_value))  # 結果を挿入
            self.result_entry.config(state='readonly')  # 再び読み取り専用にする

        except ValueError as ve:
            messagebox.showerror("入力エラー", str(ve))
        except Exception as e:
            messagebox.showerror("通信エラー", str(e))

            

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
