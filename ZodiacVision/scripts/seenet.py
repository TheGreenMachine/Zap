import networktables as nt
import time
import sys
import csv
import argparse
import datetime


def program_description():
    return 'Team 1816 Network Tables Viewing Utility for the 2019 Deep Space Season'


def view():
    # global values
    msg = "\rcoords: ({} {});" \
          " yaw, pitch: ({:.2f}, {:.2f});" \
          " contours: {};" \
          " targets: {}," \
          " fps: {}".format(
        int(values['center_x']), int(values['center_y']),
        values['yaw'], values['pitch'],
        int(values['contours']),
        int(values['targets']),
        int(values['fps']))
    if args['log']:
        with open('vision_net_values.csv', mode='a+') as vnv_file:
            vnv_writer = csv.writer(vnv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            vnv_writer.writerow(
                [datetime.datetime.now(), int(values['center_x']), int(values['center_y']), int(values['contours']),
                 int(values['targets'])])
    sys.stdout.write(msg)


def value_changed(table, key, value, isNew):
    global values
    values[key] = value
    if key == 'contours':
        view()


def start_listener():
    # During vision init
    table.addEntryListener(value_changed, key="width")
    table.addEntryListener(value_changed, key="height")
    # Updated during match
    table.addEntryListener(value_changed, key='center_x')
    table.addEntryListener(value_changed, key='center_y')
    table.addEntryListener(value_changed, key='yaw')
    table.addEntryListener(value_changed, key='contours')
    table.addEntryListener(value_changed, key='targets')
    table.addEntryListener(value_changed, key='pitch')
    table.addEntryListener(value_changed, key='fps')


parser = argparse.ArgumentParser(description=program_description())
parser.add_argument('-l', '--log',
                    action='store_true',
                    default=False,
                    help='enable logging')
args = vars(parser.parse_args())
values = {}
values['width'] = -99
values['height'] = -99
values['center_x'] = -99
values['center_y'] = -99
values['contours'] = -99
values['targets'] = -99
values['yaw'] = -99
values['pitch'] = -99
values['fps'] = -99

nt.NetworkTables.initialize(server='10.18.16.2')
table = nt.NetworkTables.getTable('SmartDashboard')

view()

start_listener()

while True:
    time.sleep(1)
