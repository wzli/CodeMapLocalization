#!/usr/bin/env python3
from codemap.serial import CsvRxStream
import json
import argparse

parser = argparse.ArgumentParser(
    description='Receive and record localization data from serial stream')
parser.add_argument('device', type=str, help="Input serial device")
parser.add_argument('-c',
                    '--csv',
                    type=str,
                    help="Output CSV file",
                    default=None)
args = parser.parse_args()

csv_rx_stream = CsvRxStream(args.device, args.csv)
csv_rx_stream.run(lambda x: print(json.dumps(x, indent=2)))
