#!/usr/bin/env python3

import serial


class CsvRxStream:
    BAUD_RATE = 115200
    CSV_HEADERS = '/timestamp,/frame,/odometry/x,/odometry/y,/odometry/rotation,/odometry/quadrant_count,/odometry/drift_count,/correlation/x,/correlation/y,/correlation/error_ratio,/location/x,/location/y,/location/match_size,/location/downsample_errors,/location/xor_error_ratio,/location/quality,/location/scale,/threshold,'.split(
        ',')

    def __init__(self, serial_device, log_file=None):
        self.latest_entry = {}
        self.serial = serial.Serial(serial_device, CsvRxStream.BAUD_RATE)
        self.log_file = None
        if log_file:
            self.log_file = open(log_file, 'w')
            self.log_file.write(','.join(CsvRxStream.CSV_HEADERS) + ',\n')

    def read_line(self):
        line_bytes = self.serial.readline()
        line_string = line_bytes.decode('ascii', 'ignore')
        line_tokens = line_string.split(',')
        if len(line_tokens) != len(CsvRxStream.CSV_HEADERS):
            return None
        for i, token in enumerate(line_tokens[:-1]):
            try:
                self.latest_entry[CsvRxStream.CSV_HEADERS[i]] = float(token)
            except ValueError as e:
                print(e)
        if self.log_file:
            self.log_file.write(line_string)
        return self.latest_entry

    def run(self, callback):
        while True:
            if self.read_line() is not None:
                callback(self.latest_entry)


csv_rx_stream = CsvRxStream('/dev/ttyUSB0', 'log.csv')
csv_rx_stream.run(lambda x: print(x))
