import sim
import time
import numpy as np
import math
'''可实现对杯子等物体的非接触探索，并保存点云文件 5/7'''

def get_sensor_data(clientID, sensor_handles):
    """
    从CoppeliaSim中获取传感器数据，返回5个传感器读数,检测到的点的坐标与法向向量在世界坐标系下的表示
    """
    sensor_distances = []
    sensor_normals = []
    
    detectedSurfaceNormalVector_world=[]    
        # 将法向量乘以旋转矩阵来转换到世界坐标系下
        

    #传感器坐标方位 [0.0, -1.5707963705062866, -1.5707752704620361]
    for handle in sensor_handles:
        _, detectionState, detectedPoint, _, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, handle, sim.simx_opmode_blocking)
        if detectionState:
            # print('传感器坐标系',detectedSurfaceNormalVector)
            # 传感器坐标系，-z方向，[0.09801831096410751, -0.0002514557563699782, -0.995184600353241]
            detectedSurfaceNormalVector_sensor = np.array(detectedSurfaceNormalVector)
            # print("传感器坐标系下法向量：=",detectedSurfaceNormalVector_sensor)        
            # 构造旋转矩阵
            res, sensorOrientation = sim.simxGetObjectOrientation(clientID, sensor_handles[2], -1, sim.simx_opmode_blocking)
            # print('传感器坐标方位',sensorOrientation)
            rotationMatrix = np.eye(3)
            rotationMatrix = np.dot(rotationMatrix, np.array([[np.cos(-sensorOrientation[2]), -np.sin(-sensorOrientation[2]), 0], [np.sin(-sensorOrientation[2]), np.cos(-sensorOrientation[2]), 0], [0, 0, 1]]))
            rotationMatrix = np.dot(rotationMatrix, np.array([[np.cos(-sensorOrientation[1]), 0, np.sin(-sensorOrientation[1])], [0, 1, 0], [-np.sin(-sensorOrientation[1]), 0, np.cos(-sensorOrientation[1])]]))
            rotationMatrix = np.dot(rotationMatrix, np.array([[1, 0, 0], [0, np.cos(-sensorOrientation[0]), -np.sin(-sensorOrientation[0])], [0, np.sin(-sensorOrientation[0]), np.cos(-sensorOrientation[0])]]))

            # 将法向量乘以旋转矩阵来转换到世界坐标系下
            detectedSurfaceNormalVector_world = np.dot(rotationMatrix.T, detectedSurfaceNormalVector_sensor)
            sensor_distances.append(np.linalg.norm(detectedPoint))  # 仅使用距离作为位置信息
            sensor_normals.append(detectedSurfaceNormalVector_world)
        else:
            # 如果未检测到物体，存储None
            sensor_distances.append(None)
            sensor_normals.append(None)
    
    # 计算平均法向量
    
    valid_normals = [normal for normal in sensor_normals if normal is not None]
    if valid_normals:
        average_normal = np.mean(valid_normals, axis=0)
    else:
        average_normal = [0, 0, 0]  # 如果没有有效的法向量，则使用默认值
    # print('世界坐标系下平面的平均法向量',average_normal)
    print('5个传感器读数',sensor_distances)
    
    return sensor_distances, average_normal

def detected_point_at_world_coordinates(clientID, sensor_handle, local_point):
    
    _, sensor_position = sim.simxGetObjectPosition(clientID, sensor_handle, -1, sim.simx_opmode_blocking)
    # 获取传感器在世界坐标系中的方向（欧拉角）
    _, sensor_orientation = sim.simxGetObjectOrientation(clientID, sensor_handle, -1, sim.simx_opmode_blocking)
    rotation_matrix = R.from_euler('xyz', sensor_orientation).as_matrix()

    # 将局部点坐标转换为世界坐标
    world_point = sensor_position + np.dot(rotation_matrix, local_point)
    return world_point



def read_sensor():
    # 定义与CoppeliaSim的连接参数
    sim.simxFinish(-1) 
    # 连接到CoppeliaSim
    client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if client_id != -1:
        print('已连接到CoppeliaSim')
    else:
        print('无法连接到CoppeliaSim')
        exit()
    sensor_handles = []
    # res, yuanzhu = sim.simxGetObjectHandle(client_id, 'Cylinder', sim.simx_opmode_blocking)
    
    for i in range(5):
        if i ==0 :
            sensor_name = 'Proximity_sensor'
        else:
            sensor_name = 'Proximity_sensor'+str(i-1)
        res, sensor_handle = sim.simxGetObjectHandle(client_id, sensor_name, sim.simx_opmode_blocking)
        sensor_handles.append(sensor_handle)
    while True:    
        sensor_data,detected_surface_normal_vector = get_sensor_data(client_id, sensor_handles)    
        # print('Proximity Sensor Data:', sensor_data,detected_surface_normal_vector)
        time.sleep(0.5)

def get_tangent_direction(normal):
    """
    获取给定法向量的一个切向向量，限制在一个角度范围内，考虑前一步的切向向量
    """ 
    # 计算切向向量在法向量上的投影
    random_vector = np.random.rand(3)
    # random_vector[1] = -random_vector[1]
    while np.allclose(random_vector, normal) or np.allclose(random_vector, -normal):
        random_vector = np.random.rand(3)
    tangent_projection = np.dot(random_vector, normal) * normal
    tangent_direction = random_vector - tangent_projection
    tangent_direction /= np.linalg.norm(tangent_direction)  
    return tangent_direction

def is_within_ur10_range(position, workspace_radius=1000):
    """
    检查给定位置是否在 UR10 最大活动范围内
    """
    # 计算位置到基座的距离
    distance_to_base = np.linalg.norm(position)

    # 如果距离小于工作空间半径，则位置在 UR10 最大活动范围内
    if distance_to_base <= workspace_radius:
        return True
    else:
        return False
    
def move_away(clientID,targetHandle,distance,average_normal):
    _,currentposition = sim.simxGetObjectPosition(clientID,targetHandle,-1,sim.simx_opmode_blocking)
    target_position = currentposition + distance*average_normal
    sim.simxSetObjectPosition(clientID,targetHandle,-1,target_position,sim.simx_opmode_oneshot)
    print('后退')

def move_towards(clientID, targetHandle, hemisphere_center,distance):
    _, currentPosition = sim.simxGetObjectPosition(clientID, targetHandle, -1, sim.simx_opmode_blocking)
    average_normal = hemisphere_center - currentPosition
    average_normal /= np.linalg.norm(average_normal)
    step_size = 0.005
    num_steps = int(distance / step_size)
    for i in range(1,num_steps+1):
        targetPosition = currentPosition + step_size * average_normal *i
        sim.simxSetObjectPosition(clientID, targetHandle, -1, targetPosition.tolist(), sim.simx_opmode_blocking)
        # time.sleep(0.005)
    print('前进')
    
def move_self_forward(clientID, targetHandle, distance):
    _, tip_handle = sim.simxGetObjectHandle(clientID, 'tip', sim.simx_opmode_blocking)
    
    _, self_direction = sim.simxGetObjectOrientation(clientID, tip_handle, -1, sim.simx_opmode_blocking)
    yaw = self_direction[0]  # Z
    pitch = self_direction[1]  # Y
    roll = self_direction[2]  # X

    Rz = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    Ry = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    # 注意旋转矩阵的乘法顺序，ZYX顺序
    R = Rz @ Ry @ Rx

    # 初始x轴方向向量
    x_vector = np.array([1, 0, 0])

    # 旋转后的x轴方向向量
    rotated_x_vector = R @ x_vector

    # 取反得到负方向
    negative_x_vector = -rotated_x_vector

    _, current_position = sim.simxGetObjectPosition(clientID, tip_handle, -1, sim.simx_opmode_blocking)
    step_size = 0.005
    num_steps = int(distance / step_size)
    for i in range(1,num_steps+1):
        target_position = [
            current_position[0] + negative_x_vector[0] * step_size*i,
            current_position[1] + negative_x_vector[1] * step_size*i,
            current_position[2] + negative_x_vector[2] * step_size*i,
        ]
        sim.simxSetObjectPosition(clientID, targetHandle, -1, target_position, sim.simx_opmode_blocking) 
    print('后退')
           
def move_sideways(clientID, targetHandle, distance,average_normal):
    """
    向侧面移动，以便使机械臂末端构成连续的凸轨迹
    """
    _, currentPosition = sim.simxGetObjectPosition(clientID, targetHandle, -1, sim.simx_opmode_blocking)

    tangent_direction = get_tangent_direction(average_normal)
    # print("切向",tangent_direction)
    # 计算移动后的末端位置
    new_position = currentPosition - tangent_direction * distance

    sim.simxSetObjectPosition(clientID, targetHandle, -1, new_position, sim.simx_opmode_blocking)
    print('侧移')
    return new_position


from scipy.spatial.transform import Rotation as R

def align_object_with_normal_using_quaternion(clientID, targetHandle, normal_vector):
    """
    使用四元数将targetHandle的Z轴与给定的normal_vector对齐
    """
    # 标准化法向量
    # normal_vector = [-vector for vector in normal_vector ]
    
    normal_vector = np.array(normal_vector)
    normal_vector = normal_vector / np.linalg.norm(normal_vector)
    
    _,plane_handle = sim.simxGetObjectHandle(clientID,'Plane',sim.simx_opmode_blocking)
    
    _,position = sim.simxGetObjectPosition(clientID,plane_handle,targetHandle,sim.simx_opmode_blocking)
    
    # print("相对位置===",position)
    # 获取targetHandle当前的方向（四元数）
    _, current_quaternion = sim.simxGetObjectQuaternion(clientID, targetHandle, -1, sim.simx_opmode_blocking)
    current_rotation = R.from_quat(current_quaternion)

    # 计算需要的旋转，使得x轴与normal_vector对齐
    current_z = current_rotation.apply([1, 0, 0])
    rot_axis = np.cross(current_z, normal_vector)
    if np.linalg.norm(rot_axis) < 1e-6:  # 如果两个向量已经对齐
        rot_angle = 0
    else:
        rot_axis = rot_axis / np.linalg.norm(rot_axis)
        rot_angle = np.arccos(np.clip(np.dot(current_z, normal_vector), -1.0, 1.0))

    # 创建四元数表示的旋转
    rotation = R.from_rotvec(rot_angle * rot_axis)
    # 组合旋转：将新旋转应用于当前旋转
    new_rotation = rotation * current_rotation
    new_quaternion = new_rotation.as_quat()

    # 应用新的四元数
    sim.simxSetObjectQuaternion(clientID, targetHandle, -1, new_quaternion.tolist(), sim.simx_opmode_blocking)
    sensor_handles = []
    for i in range(5):
        if i ==0:
                _, handle = sim.simxGetObjectHandle(clientID, 'Proximity_sensor', sim.simx_opmode_blocking)
        else:
            _, handle = sim.simxGetObjectHandle(clientID, f'Proximity_sensor{i-1}', sim.simx_opmode_blocking)
        sensor_handles.append(handle)
    sensor_distances, average_normal = get_sensor_data(clientID, sensor_handles)
    
    print('校准后结果=',sensor_distances)

def set_initial_posture(clientID, targetHandle, hemisphere_center, hemisphere_radius):
    """随机选择一个位于半球表面的初始位置，并设置目标末端的姿态指向球心"""
    position = select_random_position_on_hemisphere(hemisphere_center, hemisphere_radius)
    direction = hemisphere_center - position
    direction /= np.linalg.norm(direction)
    sim.simxSetObjectPosition(clientID, targetHandle, -1, position.tolist(), sim.simx_opmode_blocking)
    align_object_with_normal_using_quaternion(clientID, targetHandle, -direction)

def select_random_position_on_hemisphere(center, radius):
    """从半球表面随机选择一个点"""
    theta = np.random.uniform(0, np.pi*2)
    phi = np.random.uniform(0, np.pi/2)
    x = center[0] + radius * np.sin(phi) * np.cos(theta)
    y = center[1] + radius * np.sin(phi) * np.sin(theta)
    z = center[2] + radius * np.cos(phi)
    if z <= 0.1:
        z = 0.1
    return np.array([x, y, z])

def align_and_move_cycle(clientID, targetHandle, sensor_handles):
    """执行对齐和侧移的循环，直到所有传感器距离均为None"""
    while True:
        sensor_distances, average_normal = get_sensor_data(clientID, sensor_handles)
        if all(distance is None for distance in sensor_distances):
            break
        align_object_with_normal_using_quaternion(clientID, targetHandle, average_normal)
        move_sideways(clientID, targetHandle, 0.005, average_normal)
        if any(d is not None and d < 0.005 for d in sensor_distances):
            move_self_forward(clientID, targetHandle, -0.005)
        if sensor_distances[2] is not None:
            local_point = np.array([0, 0, sensor_distances[2]])
            world_point = detected_point_at_world_coordinates(clientID, sensor_handles[2], local_point)
            # print(world_point,'xxxxxxxxx')
            if world_point[2] < 0.08:
                break

        #     points.append(world_point)
        #     normals.append(average_normal)
        #     print(".............完成对齐..........")


def explore_object(clientID, targetHandle, sensor_handles, hemisphere_center, hemisphere_radius):
    """探索对象的主循环，包括移动至球心、执行对齐和侧移的循环，以及重置初始位置"""
    initial_position = select_random_position_on_hemisphere(hemisphere_center, hemisphere_radius)
    set_initial_posture(clientID, targetHandle, initial_position, hemisphere_center - initial_position)
    move_towards(clientID, targetHandle, hemisphere_center, 0.005)

    while True:
        sensor_distances= get_sensor_data(clientID, sensor_handles)[0]
        if not all(distance is None for distance in sensor_distances):
            align_and_move_cycle(clientID, targetHandle, sensor_handles)
        else:
            set_initial_posture(clientID, targetHandle, hemisphere_center, hemisphere_radius)
            move_towards(clientID, targetHandle, hemisphere_center, 0.005)


import pandas as pd
from datetime import datetime 
def main():
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    # object_name = 'Cylinder'
    object_name = 'cup'
    _, object_handle = sim.simxGetObjectHandle(clientID, object_name, sim.simx_opmode_blocking)
    _, targetHandle = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_blocking)
    sensor_handles = [sim.simxGetObjectHandle(clientID, f'Proximity_sensor{i}', sim.simx_opmode_blocking)[1] for i in range(5)]
    hemisphere_center = np.array([0, -0.6, 0])
    hemisphere_radius = 0.3
    points = []
    normals = []
    
    if clientID != -1:
        print('Connected to CoppeliaSim')
        try:
            while True:
                set_initial_posture(clientID, targetHandle, hemisphere_center, hemisphere_radius)
                # 向目标中心移动直到检测到物体表面
                while True:
                    sensor_distances, average_normal = get_sensor_data(clientID, sensor_handles)
                    if any(distance is not None for distance in sensor_distances):
                        break  # 检测到物体，开始探索
                    move_towards(clientID, targetHandle, hemisphere_center, 0.01)

                # 开始探索
                while True:
                    sensor_distances, average_normal = get_sensor_data(clientID, sensor_handles)
                    if all(distance is None for distance in sensor_distances):
                        print('初始位置不佳，重新探索')
                        break
                    align_object_with_normal_using_quaternion(clientID, targetHandle, average_normal)
                    move_sideways(clientID, targetHandle, 0.005, average_normal)
                    if any(d is not None and d < 0.005 for d in sensor_distances):
                        move_self_forward(clientID, targetHandle, -0.002)
                        print('距离小，后退一小步')
                    if sensor_distances[2] is not None :
                        local_point = np.array([0, 0, sensor_distances[2]])
                        world_point = detected_point_at_world_coordinates(clientID, sensor_handles[2], local_point)
                        if world_point[2] < 0.01:
                            print("高度太低，重置初始姿势")
                            break
                        points.append(detected_point_at_world_coordinates(clientID, sensor_handles[2], [0, 0, sensor_distances[2]]))
                        normals.append(average_normal)
        finally:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"D:/资料/毕设相关/仿真/点云数据集/{object_name}_{timestamp}.xlsx"
            df = pd.DataFrame({
                'Position_X': [p[0] for p in points],
                'Position_Y': [p[1] for p in points],
                'Position_Z': [p[2] for p in points],
                'Normal_X': [n[0] for n in normals],
                'Normal_Y': [n[1] for n in normals],
                'Normal_Z': [n[2] for n in normals]
            })
            df.to_excel(filename, index=False)
            print("数据已保存到Excel文件")
    else:
        print('Failed connecting to CoppeliaSim')

if __name__ == '__main__':
    main()
    # read_sensor()
    
