#!/usr/bin/env python3

## Mostly copied from bagpy (https://github.com/jmscslgroup/bagpy/)

import os
import sys
import rosbag
import pandas as pd
import numpy as np

number_of_boards = 9

joint_names = [
    'elbow_joint',
    'distal_radioulnar_joint',
    'radiocarpal_abductor_joint',
    'radiocarpal_flexor_joint',
    'thumb_abduction_joint',
    'thumb_proximal_joint',
    'thumb_middle_joint',
    'thumb_distal_joint',
    'index_abduction_joint',
    'index_proximal_joint',
    'index_middle_joint',
    'index_distal_joint',
    'middle_abduction_joint',
    'middle_proximal_joint',
    'middle_middle_joint',
    'middle_distal_joint',
    'ring_abduction_joint',
    'ring_proximal_joint',
    'ring_middle_joint',
    'ring_distal_joint',
    'little_abduction_joint',
    'little_proximal_joint',
    'little_middle_joint',
    'little_distal_joint'
]

def find(s, ch):
    return [i for i, ltr in enumerate(s) if ltr == ch]

class bagreader():
    def __init__(self, bagfile):
        print('[DEBUG] Processing file "{0}"'.format(bagfile))
        slashindices = find(bagfile, '/')

        if len(slashindices) > 0:
            self.filename = bagfile[slashindices[-1]:]
            self.dir = bagfile[slashindices[0]:slashindices[-1]]
        else:
            self.filename = bagfile
            self.dir = './'

        self.reader = rosbag.Bag(bagfile)

        info = self.reader.get_type_and_topic_info()
        self.topic_tuple = info.topics.values()
        self.topics = info.topics.keys()

        self.message_types = [t1.msg_type for t1 in self.topic_tuple]
        self.n_messages = [t1.message_count for t1 in self.topic_tuple]
        self.frequency = [t1.frequency for t1 in self.topic_tuple]

        self.topic_table = pd.DataFrame(list(zip(self.topics, self.message_types, self.n_messages, self.frequency)), columns=['Topics', 'Types', 'Message Count', 'Frequency'])

        self.start_time = self.reader.get_start_time()
        self.end_time = self.reader.get_end_time()

        split_name = bagfile.split('/')[-1][0:-4]
        date, gest_name = split_name.split('_')[-1], '_'.join(split_name.split('_')[:-1])
        self.datafolder = os.path.join('/tmp', 'bag_to_csv', gest_name, date)

        if os.path.exists(self.datafolder):
            print('[INFO] Data folder {0} already exists. Not creating.'.format(self.datafolder))
        else:
            try:
                os.makedirs(self.datafolder)
            except OSError:
                print('[ERROR] Failed to create data folder.'.format(self.datafolder))

        print('topics: {0}, types: {1}, n_messages: {2}, frequencies: {3}'.format(self.topics, self.message_types, self.n_messages, self.frequency))

        rec_flag_topic = [t for t in self.topics if 'rec_flag' in t][0]
        tactile_topic = [t for t in self.topics if 'tactile_states' in t][0]
        gt_topic = [t for t in self.topics if 'joint_states' in t][0]

        last_val = 0
        rec_start = []
        rec_stop = []
        for topic, msg, t in self.reader.read_messages(topics=rec_flag_topic, start_time=None, end_time=None):
            if msg.data != last_val:
                last_val = msg.data
                if msg.data == 1:
                    rec_start.append(t)
                elif msg.data == 0:
                    rec_stop.append(t)

        print('[INFO] rec_starts: {0}, rec_stops: {1}'.format(rec_start, rec_stop))

        board_cols = []
        for idx in range(number_of_boards):
            board_cols.extend(['board{0}_{1}'.format(idx, j) for j in np.arange(32)])
        print('[INFO] Generated board columns')

        extracted_data = pd.DataFrame(columns=joint_names+board_cols+['timestamp'])

        for t_start, t_stop in zip(rec_start, rec_stop):
            for topic, msg, t in self.reader.read_messages(topics=tactile_topic, start_time=t_start, end_time=t_stop):
                df_entry = {'timestamp': t}
                for sensor in msg.sensors:
                    sensor_dict = {'{0}_{1}'.format(sensor.name, j):val for j,val in enumerate(sensor.values)}
                    df_entry.update(sensor_dict)
                extracted_data = extracted_data.append(pd.Series(df_entry), ignore_index=True)

        print('[INFO] Extracted tactile data')
        extracted_data = extracted_data.set_index('timestamp')

        extracted_gt = pd.DataFrame(columns=joint_names)
        for topic, msg, t in self.reader.read_messages(topics=gt_topic, start_time=t_start, end_time=t_stop):
            df_entry = {name:val for name,val in zip(msg.name, msg.position)}
            df_entry['timestamp'] = t
            extracted_gt = extracted_gt.append(pd.Series(df_entry), ignore_index=True)
        print('[INFO] Extracted GT data')

        extracted_gt = extracted_gt.set_index('timestamp')
        # Now we have to interpolate the gt data to fit the bracelet values
        # this is a bit tricky, because timestamps might not overlap.
        # Therefore we first *extend* the gt index by the tactile data index, simply filling in NaN values.
        # As we still have the original data at the original points in time, we can now interpolate linearly
        # and afterwards remove the original data at non-overlapping timestamps. (Data inserted after the last
        # original timestamp is filled with the last valid value, which should be fine).
        # Also, as initial tactile data might be present before the first joint states message, we use backfill
        # after linear interpolation to replace leading NaNs in gt data.
        extracted_gt = extracted_gt.reindex(extracted_gt.index.union(extracted_data.index)).interpolate('linear').reindex(extracted_data.index)

        extracted_data[joint_names] = extracted_gt[joint_names]
        extracted_data = extracted_data.interpolate('backfill')
        del extracted_gt
        extracted_data.to_csv(os.path.join(self.datafolder, 'data.csv'), index=True, header=True)


if __name__ == '__main__':
    print('Starting script')

    if len(sys.argv) != 2:
        print('[ERROR] Wrong number of arguments!\nUsage: python convert_to_csv.py FILE_PATH|DIR_PATH')
        sys.exit(1)

    path_arg = sys.argv[1]

    if os.path.isdir(path_arg):
        for bag in [f for f in os.listdir(os.path.abspath(path_arg)) if f.split('.')[-1] == 'bag']:
            bagreader(os.path.join(os.path.abspath(path_arg), bag))
    elif os.path.isfile(path_arg):
        bagreder(os.path.abspath(path_arg))
    else:
        print('[ERROR] Invalid path or filename! ({0})'.format(os.path.abspath(path_arg)))
        sys.exit(1)
