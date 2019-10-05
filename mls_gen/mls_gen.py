#!/usr/bin/env python3
import argparse, yaml
from collections import OrderedDict
from mls import LFSR

parser = argparse.ArgumentParser(description='Generate M-sequences using LFSR')
parser.add_argument('n_bits', type=int, help='bit length of the LFSR')
parser.add_argument('--seed', type=int, help='start state of the LFSR', default = 1)
args = parser.parse_args()

lfsr = LFSR(args.n_bits, args.seed)
sequence = lfsr.sequence_string()

mls_data = OrderedDict((
    ('word_length' , args.n_bits) ,
    ('sequence_length' , len(sequence)),
    ('sequence_hash' , hash(sequence)),
    ('sequence' , sequence),
))

yaml_file_name = 'mls_' + str(args.n_bits) + '.yaml'

def ordered_dump(data, stream=None, Dumper=yaml.Dumper, **kwds):
    class OrderedDumper(Dumper):
        pass
    def _dict_representer(dumper, data):
        return dumper.represent_mapping(
                yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
                data.items())
    OrderedDumper.add_representer(OrderedDict, _dict_representer)
    return yaml.dump(data, stream, OrderedDumper, **kwds)

with open(yaml_file_name, 'w') as yaml_file:
    ordered_dump(mls_data, yaml_file, default_flow_style = False)

print(f'generated {yaml_file_name}')
