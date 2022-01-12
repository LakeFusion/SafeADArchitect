#!/usr/bin/env python3

# Copyright (c) 2022 Lake Fusion Technologies GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import rclpy
import argparse
import glob
import sys
import os.path
import time
from datetime import datetime as dt

try:
    sys.path.append(glob.glob('./carla-0.9.11.egg')[0])
except IndexError:
    pass

import carla

try:
    import queue
except ImportError:
    import Queue as queue


gRun = True
gQueues = []


def makeQueue(registerEvent):
    q = queue.Queue()
    registerEvent(q.put)
    gQueues.append(q)


def main():
    global gRun

    argparser = argparse.ArgumentParser(description='LFT Lidar Post-Processing Script for SafeADArchitect')

    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')

    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')

    argparser.add_argument(
        '-r', '--recPath',
        metavar='P',
        default='',
        help='Path to the recording folder to save the clouds as PCD (optional)')

    args = argparser.parse_args()

    baseTime = time.time()

    if os.path.exists(args.recPath):
        recording = True
        recPath = os.path.join(args.recPath, dt.fromtimestamp(baseTime).strftime("%Y%m%d_%H%M%S"))
        from NextRecordingLidar import NextRecordingLidar as SafeADLidar
        from RecordingCamera import RecordingCamera as SafeADCamera
    else:
        recording = False
        recPath = ''
        from NextLidar import NextLidar as SafeADLidar

    client = carla.Client(args.host, args.port)

    client.set_timeout(2.0)

    print('\n\nStarting Lidar Post-Processing, Press CTRL-C to end execution\n\n')

    try:
        world = client.get_world()

        actors = world.get_actors()

        while (len(actors) == 0):
            print("No actors found! Retrying...")
            time.sleep(0.2)
            actors = world.get_actors()

        # roleName: (topicName, shortRangeFlag)
        lidarInitList = {'lidar_front_left': ('left', True),
                         'lidar_front_center': ('center', True),
                         'lidar_front_long': ('center11', False),
                         'lidar_front_right': ('right', True)}
        lidarList = []
        camList = []

        lidarCount = 0
        lidarCountMax = len(lidarInitList) - 1

        # make a queue for the timestamp data
        makeQueue(world.on_tick)

        qIndex = 1
        for actor in actors:
            if (actor.type_id == "sensor.lidar.ray_cast_semantic"):
                try:
                    if lidarCount > lidarCountMax:
                        break

                    initTuple = lidarInitList[actor.attributes['role_name']]

                    lidarList.append(SafeADLidar(initTuple[0], initTuple[1], actor, rclpy, world, qIndex, recPath))
                    makeQueue(actor.listen)
                    qIndex += 1

                    print("Connecting to " + actor.attributes['role_name'])
                    lidarCount += 1
                except KeyError:
                    print("Lidar name " + actor.attributes['role_name'] + " is not in the list, ignoring...")
                finally:
                    pass
            else:
                if recording and (actor.type_id == "sensor.camera.rgb"):
                    camList.append(SafeADCamera(actor, recPath, len(camList), qIndex))
                    makeQueue(actor.listen)
                    qIndex += 1

        while True:
            data = [q.get(timeout=2.0) for q in gQueues]
            ts = baseTime + data[0].elapsed_seconds

            for lidar in lidarList:
                lidar.processCloud(data, ts)

            for lidar in lidarList:
                lidar.publish()

            for cam in camList:
                cam.save(ts, data)

    except KeyboardInterrupt:
        gRun = False

    except RuntimeError as e:
        gRun = False
        print(e)

    finally:
        pass


if __name__ == '__main__':
    # setup ROS node
    rclpy.init()

    try:
        while gRun:
            main()

    except KeyboardInterrupt:
        print('\nStopped')
