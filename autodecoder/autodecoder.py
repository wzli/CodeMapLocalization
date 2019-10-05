#!/usr/bin/env python3

import yaml
import numpy as np
import keras
import tensorflow as tf

config = tf.ConfigProto()
config.gpu_options.allow_growth = True
session = tf.Session(config=config)

word_length = 17
dmls_file = "dmls_17.yaml"

with open(dmls_file, 'r') as stream:
    dmls = yaml.safe_load(stream)

sequence = dmls["sequence"]
word_position_map = {}
word_vectors = []
position_vectors = []

for position in range(len(sequence) - word_length + 1):
    word = sequence[position : position + 17]
    word_position_map[int(word, 2)] = position
    word_vectors.append(np.fromstring(word, dtype=np.int8) - ord('0'))
    position_vectors.append(np.array([len(sequence), position], dtype = np.int32))


for word in range(2**17):
    if word not in word_position_map:
        word_string = format(word, '0' + str(word_length) + 'b');
        word_vectors.append(np.fromstring(word_string, dtype=np.int8) - ord('0'))
        position_vectors.append(np.array([0, 0], dtype = np.int32))

x_train = np.vstack(word_vectors).astype(np.float32)[..., np.newaxis]
y_train = np.vstack(position_vectors)


''' this is for binary output encoding
for position in range(len(sequence) - word_length + 1):
    word_string = sequence[position : position + 17]
    position_string = '1' + format(position, '0' + str(word_length) + 'b');
    word_position_map[int(word_string, 2)] = position
    word_vectors.append(np.fromstring(word_string, dtype=np.int8) - ord('0'))
    position_vectors.append(np.fromstring(position_string, dtype=np.int8) - ord('0'))

empty_position_string = np.zeros((word_length + 1));
for word in range(2**word_length):
    if word not in word_position_map:
        word_string = format(word, '0' + str(word_length) + 'b');
        word_vectors.append(np.fromstring(word_string, dtype=np.int8) - ord('0'))
        position_vectors.append(empty_position_string)

x_train = np.vstack(word_vectors).astype(np.float32)#[..., np.newaxis]
y_train = np.vstack(position_vectors)
''' and None

model = keras.models.Sequential([
    keras.layers.Conv1D(8, 3),
    keras.layers.Conv1D(8, 3),
    keras.layers.Conv1D(8, 3),
    keras.layers.Conv1D(8, 3),
    keras.layers.Flatten(),
    keras.layers.Dense(2, activation='relu'),
])

model.compile(
    optimizer='adam',
    loss='mean_squared_error')


model.fit(x_train, y_train, epochs=1000)
model.summary()
model.evaluate(x_train, y_train, verbose=2)

model.save("model.h5")

print(model.predict(x_train[10:20]))
print(y_train[10:20])
