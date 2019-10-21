#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
import time
import keyboard
import numpy as np
import open3d as o3d
import shutil
import math

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import random

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    import queue
except ImportError:
    import Queue as queue


class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world, *sensors, **kwargs):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout):
        #print('tick')
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        #print('print data in tick')
        #print(dir(data))
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            #print('print data in _retrieve_data')
            #print(dir(data))
            if data.frame == self.frame:
                return data



def process_lidar(LidarMeasurement, pcd):

    print('process_lidar')

    ##=========PointCloud===============
    lidar_array=np.frombuffer(LidarMeasurement.raw_data, dtype=np.dtype('f4'))
    #lidar_array = np.asarray(LidarMeasurement.raw_data, np.float32)
    print (lidar_array)
    lidar_array = lidar_array.reshape((-1, 3))
    lidar_array_final=np.array(lidar_array)
    lidar_array_final[:,2] *= -1
    print('lidar_array created')

    # From numpy to Open3D
    pcd.points = o3d.utility.Vector3dVector(lidar_array_final)

    #rotate
    rotation=np.ndarray(shape=(3,1))
    rotation[0][0]=math.radians(-90)
    rotation[1][0]=0
    rotation[2][0]=math.radians(180)

    pcd.rotate(rotation, center=True)

    print ('pcd.points created')
    #print(pcd.points)
    print(np.shape(pcd.points))

    return pcd

def delete_files(folder):
    for the_file in os.listdir(folder):
        file_path = os.path.join(folder, the_file)
        try:
            if os.path.isfile(file_path):
                os.unlink(file_path)
        #elif os.path.isdir(file_path): shutil.rmtree(file_path)
        except Exception as e:
            print(e)


def main():
    actor_list = []

    first_while_iteration=True

    #Lidar open3d objects
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Open3D', width=1280, height=720, left=50, top=50, visible=True)
    pcd = o3d.geometry.PointCloud()

    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    world = client.get_world()

    try:
        m = world.get_map()
        start_pose = random.choice(m.get_spawn_points())
        waypoint = m.get_waypoint(start_pose.location)





        blueprint_library = world.get_blueprint_library()

        vehicle = world.spawn_actor(
            random.choice(blueprint_library.filter('vehicle.*')),
            start_pose)
        actor_list.append(vehicle)
        vehicle.set_simulate_physics(False)
        #vehicle.set_autopilot(True)

        camera_lidar_blueprint=world.get_blueprint_library().find('sensor.lidar.ray_cast')
        camera_lidar_blueprint.set_attribute('channels', '128')
        camera_lidar_blueprint.set_attribute('rotation_frequency', '10.0')
        camera_lidar_blueprint.set_attribute('range', '2000')
        camera_lidar_blueprint.set_attribute('points_per_second', '448000')
        camera_lidar_blueprint.set_attribute('upper_fov', '90.0')
        camera_lidar_blueprint.set_attribute('lower_fov', '-90.0')
        #camera_lidar_blueprint.set_attribute('sensor_tick', '1.0')
        transform = carla.Transform(carla.Location(x=-5.5, z=2.8))
        camera_lidar = world.spawn_actor(camera_lidar_blueprint, transform, attach_to=vehicle)

        actor_list.append(camera_lidar)

        # Create a synchronous mode context.
        with CarlaSyncMode(world, camera_lidar, fps=10) as sync_mode:
            while True:

                # Advance the simulation and wait for the data.
                snapshot, lidar_data = sync_mode.tick(timeout=10.0)

                # Choose the next waypoint and update the car location.
                waypoint = random.choice(waypoint.next(1.5))
                vehicle.set_transform(waypoint.transform)


                print('raw_data')
                print(lidar_data.raw_data)

                #Delete previous saves
                if first_while_iteration:
                    delete_files('/home/edu/Dropbox/Test/Files')

                #Process lidar
                pcd = process_lidar(lidar_data, pcd)
                o3d.io.write_point_cloud("/home/edu/Dropbox/Test/Files/copy_of_fragment_{}.pcd".format(lidar_data.frame_number), pcd)
                print ('pcd.points en CarlaSyncMode')
                print(np.shape(pcd.points))
                
                if first_while_iteration:

                    vis.add_geometry(pcd)

                vis.update_geometry()
                vis.poll_events()
                vis.update_renderer()
                #time.sleep(2)

                first_while_iteration == False

                if keyboard.is_pressed('q'):  # if key 'q' is pressed 
                    return


    finally:

        print('destroying actors.')
        for actor in actor_list:
            actor.destroy()

        pygame.quit()
        vis.destroy_window()
        print('done.')


if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')