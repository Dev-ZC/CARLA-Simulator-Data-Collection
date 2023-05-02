import sys
import math
import random
import time
import numpy as np
import cv2
import glob
import os

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

#Starter Carla code
client = carla.Client('localhost', 2000)
world = client.get_world()

townName = "Town01"

while True:
    print("What map would you like to spawn in?")
    print("(Town '1', '2', '3',...'7') or '0' for default")
    mapChoice = input(": ")
    if mapChoice == "1":
        world = client.load_world("Town01")
        townName = "Town01"
        break
    elif mapChoice == "2":
        world = client.load_world("Town02")
        townName = "Town02"
        break
    elif mapChoice == "3":
        world = client.load_world("Town03")
        townName = "Town03"
        break
    elif mapChoice == "4":
        world = client.load_world("Town04")
        townName = "Town04"
        break
    elif mapChoice == "5":
        world = client.load_world("Town05")
        townName = "Town05"
        break
    elif mapChoice == "6":
        world = client.load_world("Town06")
        townName = "Town06"
        break
    elif mapChoice == "7":
        world = client.load_world("Town07")
        townName = "Town07"
        break
    elif mapChoice == "0":
        break
    elif mapChoice == "q" or mapChoice == "Q":
        break
    else:
        print("That is not a valid input")

bp_lib = world.get_blueprint_library()
spawn_points = world.get_map().get_spawn_points()

#Finds car from library and creates it within the simulator
# vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
# vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

blueprints = world.get_blueprint_library().filter('vehicle.*')
vehicle_bp = random.choice(blueprints)
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

#Places spectator behind the vehicle
spectator = world.get_spectator()
transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=2.5)),vehicle.get_transform().rotation)
spectator.set_transform(transform)
                                             
#Set camera initial transform
camera_init_trans = carla.Transform(carla.Location(z=2))

def spawn_vehicles(num_vehicles, town_name):
    blueprint_library = world.get_blueprint_library()

    # Get a list of all spawn points
    spawn_points = list(world.get_map().get_spawn_points())

    # Shuffle the list of spawn points
    random.shuffle(spawn_points)

    # Create a list to store spawned vehicles
    spawned_vehicles = []
    
    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_random_device_seed(0)

    # Enable synchronous mode for the traffic manager
    traffic_manager.set_synchronous_mode(True)
    world.tick()

    # Set the distance between vehicles
    traffic_manager.set_global_distance_to_leading_vehicle(2.0)

    # Set random vehicle speed if the method is available
    if hasattr(traffic_manager, 'set_random_vehicle_speed'):
        traffic_manager.set_random_vehicle_speed(True)

    for i in range(num_vehicles):
        # Choose a random spawn point
        spawn_point = spawn_points[i % len(spawn_points)]

        # Try to spawn the vehicle at the spawn point
        try:
            blueprint = random.choice(blueprint_library.filter('vehicle.*'))
            vehicle = world.spawn_actor(blueprint, spawn_point)
        except RuntimeError as error:
            print(error)
            print(f"Trying another spawn point for vehicle {i}...")
            continue

        # Append the spawned vehicle to the list
        spawned_vehicles.append(vehicle)

        # Set up the vehicle's sensors
        ...

    return spawned_vehicles





#Pedestrian spawning function fix <--------------
def spawnPedestrians(client, world):
    walker_bp = world.get_blueprint_library().filter("walker.pedestrian.*")
    controller_bp = world.get_blueprint_library().find('controller.ai.walker')

    actor = []

    for i in range(200):
        trans = carla.Transform()
        trans.location = world.get_random_location_from_navigation()
        trans.location.z += 1

        #walker actor
        walker = random.choice(walker_bp)
        actor = world.spawn_actor(walker, trans)
        world.wait_for_tick()

        #walker AI controller
        controller = world.spawn_actor(controller_bp, carla.Transform(), actor)
        world.wait_for_tick()

        #Pedestrian mangagement
        controller.start()
        controller.go_to_location(world.get_random_location_from_navigation())

        actor.append(actor)
        actor.append(controller)

    while (1):
        time.sleep(0.1)
        
                                                                                                         
#spawnPedestrians(client, world)
    
#Funtion for RGB camera
def rgb_camera_run(camera_init_trans):   
    #Adds the rgb camera
    camera_bp = bp_lib.find('sensor.camera.rgb')
    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)
    ## Change the frequency to be lower
    # camera.listen(lambda image: image.save_to_disk('_out/%06d.png' % image.frame))
    
    #Asks if user wants to record data to disc
    #while True:
        #record = input("Would you like to record data (y/n): ")
        #if moveVehicle == "y":
            #Fix segmentation fault (core dumped) error <--------
            #camera.listen(lambda image: image.save_to_disk('_out/%06d.png' % image.frame))
            #break
        #elif moveVehicle == "n":
            #break
        #else:
            #print("That is not a valid input")

    #Defines camera call back
    def rgb_callback(image, data_dict):
        data_dict['image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
        image.save_to_disk('_out/%06d.png' % image.frame)

    #Gets image size of camera 
    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_x").as_int()

    camera_data = {'image': np.zeros((image_h, image_w, 4))}

    camera.listen(lambda image: rgb_callback(image, camera_data))

    cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RGB Camera', camera_data['image'])
    cv2.waitKey(1)

    while True:
        cv2.imshow('RGB Camera', camera_data['image'])
        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()

    
#Function for Depth Camera
def depth_camera_run(camera_init_trans):
    depth_camera_bp = bp_lib.find('sensor.camera.depth')
    depth_camera = world.spawn_actor(depth_camera_bp, camera_init_trans, attach_to=vehicle)
    
    #Defines depth camera call back
    def depth_callback(image, data_dict):
        image.convert(carla.ColorConverter.LogarithmicDepth)
        data_dict['image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
        image.save_to_disk('_out/%06d.png' % image.frame)
        
    #Gets image size of camera 
    image_w = depth_camera_bp.get_attribute("image_size_x").as_int()
    image_h = depth_camera_bp.get_attribute("image_size_x").as_int()

    depth_camera_data = {'image': np.zeros((image_h, image_w, 4))}

    depth_camera.listen(lambda image: depth_callback(image, depth_camera_data))

    cv2.namedWindow('Depth Camera', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('Depth Camera', depth_camera_data['image'])
    cv2.waitKey(1)

    while True:
        cv2.imshow('Depth Camera', depth_camera_data['image'])

        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()

#Function for Semantic Camera
def sem_camera_run(camera_init_trans):
    sem_camera_bp = bp_lib.find('sensor.camera.semantic_segmentation')
    sem_camera = world.spawn_actor(sem_camera_bp, camera_init_trans, attach_to=vehicle)
    
    #Defines depth camera call back
    def sem_callback(image, data_dict):
        image.convert(carla.ColorConverter.CityScapesPalette)
        data_dict['image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
        image.save_to_disk('_out/%06d.png' % image.frame)
        
    #Gets image size of camera 
    image_w = sem_camera_bp.get_attribute("image_size_x").as_int()
    image_h = sem_camera_bp.get_attribute("image_size_x").as_int()

    sem_camera_data = {'image': np.zeros((image_h, image_w, 4))}

    sem_camera.listen(lambda image: sem_callback(image, sem_camera_data))

    cv2.namedWindow('Semantic Camera', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('Semantic Camera', sem_camera_data['image'])
    cv2.waitKey(1)

    while True:
        cv2.imshow('Semantic Camera', sem_camera_data['image'])

        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
    
#Function for Semantic Camera
def inst_camera_run(camera_init_trans):
    inst_camera_bp = bp_lib.find('sensor.camera.instance_segmentation')
    inst_camera = world.spawn_actor(inst_camera_bp, camera_init_trans, attach_to=vehicle)
    
    #Defines depth camera call back
    def inst_callback(image, data_dict):
        #image.convert(carla.ColorConverter.CityScapesPalette)
        data_dict['image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
        image.save_to_disk('_out/%06d.png' % image.frame)
        
    #Gets image size of camera 
    image_w = inst_camera_bp.get_attribute("image_size_x").as_int()
    image_h = inst_camera_bp.get_attribute("image_size_x").as_int()

    inst_camera_data = {'image': np.zeros((image_h, image_w, 4))}

    inst_camera.listen(lambda image: inst_callback(image, inst_camera_data))

    cv2.namedWindow('Instance Camera', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('Instance Camera', inst_camera_data['image'])
    cv2.waitKey(1)

    while True:
        cv2.imshow('Instance Camera', inst_camera_data['image'])

        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
    
def dvs_camera_run(camera_init_trans):
    dvs_camera_bp = bp_lib.find('sensor.camera.dvs')
    dvs_camera = world.spawn_actor(dvs_camera_bp, camera_init_trans, attach_to=vehicle)
    
    #Defines depth camera call back
    def dvs_callback(data, data_dict):
        dvs_events = np.frombuffer(data.raw_data, dtype=np.dtype([
                    ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', bool)]))
        data_dict['image'] = np.zeros((data.height, data.width, 4), dtype=np.uint8)
        dvs_img = np.zeros((data.height, data.width, 3), dtype=np.uint8)
        dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
        data_dict['image'][:,:,0:3] = dvs_img
        data.save_to_disk('_out/%06d.png' % image.frame)
        
    #Gets image size of camera 
    image_w = dvs_camera_bp.get_attribute("image_size_x").as_int()
    image_h = dvs_camera_bp.get_attribute("image_size_x").as_int()

    dvs_camera_data = {'image': np.zeros((image_h, image_w, 4))}

    dvs_camera.listen(lambda image: dvs_callback(image, dvs_camera_data))

    cv2.namedWindow('DVS Camera', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('DVS Camera', dvs_camera_data['image'])
    cv2.waitKey(1)

    while True:
        cv2.imshow('DVS Camera', dvs_camera_data['image'])

        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
    
def opt_camera_run(camera_init_trans):
    opt_camera_bp = bp_lib.find('sensor.camera.optical_flow')
    opt_camera = world.spawn_actor(opt_camera_bp, camera_init_trans, attach_to=vehicle)
    
    #Defines depth camera call back
    def opt_callback(data, data_dict):
        image = data.get_color_coded_flow()
        img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
        img[:,:,3] = 255
        data_dict['image'] = img
        data.save_to_disk('_out/%06d.png' % image.frame)
        
    #Gets image size of camera 
    image_w = opt_camera_bp.get_attribute("image_size_x").as_int()
    image_h = opt_camera_bp.get_attribute("image_size_x").as_int()

    opt_camera_data = {'image': np.zeros((image_h, image_w, 4))}

    opt_camera.listen(lambda image: opt_callback(image, opt_camera_data))

    cv2.namedWindow('OPT Camera', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('OPT Camera', opt_camera_data['image'])
    cv2.waitKey(1)

    while True:
        cv2.imshow('OPT Camera', opt_camera_data['image'])
        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
    
def startTraffic(numVehicles):
    #Spawns Traffic
    for i in range(numVehicles):
        vehicle_bp = random.choice(bp_lib.filter('vehicle'))
        npc = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
    
    #Sets traffic into motion
    for v in world.get_actors().filter('*vehicle*'):
        v.set_autopilot(True)
    
        
def spawnWalkers():    
    for i in range(50):
            walker_bp = random.choice(bp_lib.filter('walker'))
            walker = world.try_spawn_actor(walker_bp, random.choice(spawn_points))

    control = carla.WalkerControl()
    control.speed = 1.0

    walker.apply_control(control)
    
# Asks to spawn and start traffic
while True:
    moveVehicle = input("Would you like the vehicle to move (y/n): ")
    if moveVehicle == "y":
        numVehicles = input("How many vehicles would you like to spawn?")
        numVehicles = int(numVehicles)
        spawn_vehicles(numVehicles, townName)
        break
    elif moveVehicle == "n":
        break
    else:
        print("That is not a valid input")
        

#Asks to spawn walkers
while True:
    walkerSpawn = input("Would you like to spawn walkers? (y/n): ")
    if walkerSpawn == "y":
        spawnWalkers()
        break
    elif walkerSpawn == "n":
        break
    else:
        print("That is not a valid input")
    
#Ask for a camera sensor to spawn
while True:
    print("What sensor would you like to spawn? ('q' to esc)")
    print("(RGB), (DEPTH), (SEMANTIC), (INST), (DVS), (OPT)")
    sensorChoice = input(": ")
    if sensorChoice == "RGB" or sensorChoice == "rgb":        
        rgb_camera_run(camera_init_trans)
        break
    elif sensorChoice == "DEPTH" or sensorChoice == "depth" or sensorChoice == "Depth":
        #Make choice to start another sensor
        depth_camera_run(camera_init_trans)
        break
    elif sensorChoice == "SEMANTIC" or sensorChoice == "Semantic" or sensorChoice == "semantic":
        #Make choice to start another sensor
        sem_camera_run(camera_init_trans)
        break
    elif sensorChoice == "inst" or sensorChoice == "Inst" or sensorChoice == "INST":
        #Make choice to start another sensor
        inst_camera_run(camera_init_trans)
        break
    elif sensorChoice == "DVS" or sensorChoice == "Dvs" or sensorChoice == "dvs":
        #Make choice to start another sensor
        dvs_camera_run(camera_init_trans)
        break
    elif sensorChoice == "OPT" or sensorChoice == "Opt" or sensorChoice == "opt":
        #Make choice to start another sensor
        opt_camera_run(camera_init_trans)
        break
    elif sensorChoice == "q" or sensorChoice == "Q":
        break
    else:
        print("That is not a valid input")
