#!/usr/bin/env python3

import serial
import json


def nested_set(dic, keys, value):
    for key in keys[:-1]:
        dic = dic.setdefault(key, {})
    dic[keys[-1]] = value


class CsvRxStream:
    BAUD_RATE = 115200
    CSV_HEADERS_STRING = '/timestamp,/frame,/odometry/x,/odometry/y,/odometry/rotation,/odometry/quadrant_count,/odometry/drift_count,/correlation/x,/correlation/y,/correlation/error_ratio,/location/x,/location/y,/location/match_size,/location/downsample_errors,/location/xor_error_ratio,/location/quality,/location/scale,/threshold,'
    CSV_HEADERS = [[
        int(key) if key.isdigit() else key for key in header.split('/')[1:]
    ] for header in CSV_HEADERS_STRING.split(',')][:-1]

    def __init__(self, serial_device, log_file=None):
        self.latest_message = {}
        self.serial = serial.Serial(serial_device, CsvRxStream.BAUD_RATE)
        self.log_file = None
        if log_file:
            self.log_file = open(log_file, 'w')
            self.log_file.write(CsvRxStream.CSV_HEADERS_STRING + ',\n')

    def read_message(self):
        line_bytes = self.serial.readline()
        line_string = line_bytes.decode('ascii', 'ignore')
        try:
            csv_values = json.loads('[' + line_string + '0]')[:-1]
        except json.decoder.JSONDecodeError as e:
            print(line_string, e)
            return None
        if len(csv_values) != len(CsvRxStream.CSV_HEADERS):
            return None
        if self.log_file:
            self.log_file.write(line_string)
        for keys, value in zip(CsvRxStream.CSV_HEADERS, csv_values):
            nested_set(self.latest_message, keys, value)
        return self.latest_message
