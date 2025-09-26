import serial
from threading import Thread
import pandas as pd
import schema_pb2 as schema
from collections import defaultdict, deque
from google.protobuf import json_format
import sys
import time
from queue import Queue
import os
from datetime import datetime

ALIGNMENT_BYTE = 0x33
CRC_POLY= 0x1011
CRC_SEED = 0x35
BAUDRATE=1000000

class FjalarParser:
    def __init__(self):
        self.data = defaultdict(lambda: ([], []))
        self.message_queue = deque(maxlen=100)
        self.backup_stream = None

    def open_serial(self, path):
        self.stream = serial.Serial(port=path, baudrate=BAUDRATE)

        outdir = os.path.join("dashboard", "data")
        os.makedirs(outdir, exist_ok=True)
        now = datetime.now()
        filename = now.strftime("data_%Y_%m_%d_%H_%M_%S.bin")
        self.backup_stream = open(os.path.join(outdir, filename), "wb")

        self._reader_t = Thread(target=self._reader_thread)
        self._reader_t.start()
        print("opened serial")

    def open_file(self, path):
        self.stream = open(path, "rb")
        print("opened file")

    def _read(self, len):
        buf = self.stream.read(len)
        if self.backup_stream is not None:
            self.backup_stream.write(buf)

        return buf

    def _write(self, buf):
        self.stream.write(buf)

    def _write_message(self, msg):
        msg_buf = msg.SerializeToString()
        output_buf = bytes([ALIGNMENT_BYTE, len(msg_buf)]) + msg_buf
        crc = self._crc16(CRC_POLY, CRC_SEED, output_buf)
        output_buf += bytes([crc & 0x00ff, (crc >> 8) & 0x00ff ])
        self._write(output_buf)

    def _reader_thread(self):
        while True:
            alignment = self._read(1)[0]
            if (alignment != ALIGNMENT_BYTE):
                print("invalid byte")
                continue
            length = self._read(1)[0]
            data = self._read(length)
            received_crc = 0
            received_crc = self._read(1)[0]
            received_crc += self._read(1)[0] << 8
            all_bytes = bytes([alignment, length]) + data
            crc = self._crc16(CRC_POLY, CRC_SEED, all_bytes)
            if (crc != received_crc):
                print("invalid crc", received_crc, crc)
                continue
            msg = schema.FjalarMessage()
            msg.ParseFromString(data)
            time = msg.time / 1000
            data = json_format.MessageToDict(msg.data, always_print_fields_with_no_presence=True)
            print(data)
            try:
                self.message_queue.append(data)
            except:
                pass

            try:
                data = data[next(iter(data.keys()))] # ignore the message name
                for field in data.keys():
                    self.data[field][0].append(time)
                    self.data[field][1].append(data[field])
            except:
                print("invalid data", data)

    def _crc16(self, poly, seed, buf):
        crc = seed
        for byte in buf:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ poly
                else:
                    crc = crc << 1
        return crc & 0xFFFF

    def clear_flash(self):
        msg = schema.FjalarMessage()
        data_msg = schema.FjalarData()
        clear_flash_msg = schema.ClearFlash()
        data_msg.clear_flash.CopyFrom(clear_flash_msg)
        msg.data.CopyFrom(data_msg)
        self._write_message(msg)

    def enter_idle(self):
        msg = schema.FjalarMessage()
        data_msg = schema.FjalarData()
        enter_idle_msg = schema.EnterIdle()
        data_msg.enter_idle.CopyFrom(enter_idle_msg)
        msg.data.CopyFrom(data_msg)
        self._write_message(msg)

    def enter_initiate(self):
        msg = schema.FjalarMessage()
        data_msg = schema.FjalarData()
        lora_msg = schema.LORA_GcbToFjalar()
        lora_msg.ready_initiate_fjalar = True
        lora_msg.ready_launch_fjalar = False
        data_msg.lora_gcb_to_fjalar.CopyFrom(lora_msg)
        msg.data.CopyFrom(data_msg)
        self._write_message(msg)

    def enter_launch(self):
        msg = schema.FjalarMessage()
        data_msg = schema.FjalarData()
        lora_msg = schema.LORA_GcbToFjalar()
        lora_msg.ready_initiate_fjalar = False
        lora_msg.ready_launch_fjalar = True
        data_msg.lora_gcb_to_fjalar.CopyFrom(lora_msg)
        msg.data.CopyFrom(data_msg)
        self._write_message(msg)

    def toggle_sudo(self):
        msg = schema.FjalarMessage()
        data_msg = schema.FjalarData()
        set_sudo_msg = schema.SetSudo()
        set_sudo_msg.enabled = not self.data["sudo"][1][-1]
        data_msg.set_sudo.CopyFrom(set_sudo_msg)
        msg.data.CopyFrom(data_msg)
        self._write_message(msg)

    def read_flash(self, index, length):
        msg = schema.FjalarMessage()
        data_msg = schema.FjalarData()
        read_flash_msg = schema.ReadFlash()
        read_flash_msg.start_index = index
        read_flash_msg.length = length
        data_msg.read_flash.CopyFrom(read_flash_msg)
        msg.data.CopyFrom(data_msg)
        self._write_message(msg)

        start = time.time()
        index_to_read = 0

        while True:
            if time.time() - start > 1:
                return None
            if len(self.message_queue) > 0:
                msg = self.message_queue.pop()
                print(msg)
                # if msg.HasField("flash_data"):
                #     if msg.data.flash_data.start_index == index and len(msg.data.flash_data.data) == length:
                #         return msg.data.flash_data.data
                #     else:
                #         print(msg.data.flash_data.start_index, index, len(msg.data.flash_data.data), length)
                #         print("got a different flash message somewhow")
