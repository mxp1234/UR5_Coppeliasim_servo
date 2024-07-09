import sim
import time
import numpy as np
import math
import time
import pandas as pd

'''5-4完成向推动点靠拢'''

def get_sensor_data(clientID, sensor_handles):
    """
    从CoppeliaSim中获取传感器数据，返回5个传感器读数与法向向量在世界坐标系下表示
    仿真环境无法直接获得旋转矩阵，只能通过获取欧拉角得到
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
            # print("传感器坐标系下法向量xingz：=",detectedSurfaceNormalVector_sensor.shape)    
            # 构造旋转矩阵
            res, sensorOrientation = sim.simxGetObjectOrientation(clientID, sensor_handles[2], -1, sim.simx_opmode_blocking)
            # print('传感器坐标方位',sensorOrientation)
            rotationMatrix = np.eye(3)
            rotationMatrix = np.dot(rotationMatrix, np.array([[np.cos(-sensorOrientation[2]), -np.sin(-sensorOrientation[2]), 0], [np.sin(-sensorOrientation[2]), np.cos(-sensorOrientation[2]), 0], [0, 0, 1]]))
            rotationMatrix = np.dot(rotationMatrix, np.array([[np.cos(-sensorOrientation[1]), 0, np.sin(-sensorOrientation[1])], [0, 1, 0], [-np.sin(-sensorOrientation[1]), 0, np.cos(-sensorOrientation[1])]]))
            rotationMatrix = np.dot(rotationMatrix, np.array([[1, 0, 0], [0, np.cos(-sensorOrientation[0]), -np.sin(-sensorOrientation[0])], [0, np.sin(-sensorOrientation[0]), np.cos(-sensorOrientation[0])]]))

            # 将法向量乘以旋转矩阵来转换到世界坐标系下
            detectedSurfaceNormalVector_world = np.dot(rotationMatrix.T, detectedSurfaceNormalVector_sensor)
            
            
            # 传感器坐标系 [0.09802037477493286, -0.00025053624995052814, -0.9951844215393066]
            # 世界坐标系 [-2.30411762e-04 -9.95184422e-01  9.80204183e-02]
            
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
    # print('世界坐标系下平面的法向量',average_normal)
    # print('5个传感器读数',sensor_distances)
    
    return sensor_distances, average_normal


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
        # sensor_data = []
        # for sensor_handle in sensor_handles:    
        #     res, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = sim.simxReadProximitySensor(client_id, sensor_handle, sim.simx_opmode_blocking)
        
        #     if res == 0 and detection_state:  # 检测到物体时才计算距离
        #         distance = np.linalg.norm(detected_point)  # 计算检测点到传感器的距离
        #         sensor_data.append(distance)
                
        #     else:
        #         sensor_data.append(None)  # 如果未检测到物体，将距离设置为 None}
        sensor_data,detected_surface_normal_vector = get_sensor_data(client_id, sensor_handles)    
        # print('Proximity Sensor Data:', sensor_data,detected_surface_normal_vector)
        time.sleep(0.5)

def get_tangent_direction(normal):
    """
    获取给定法向量的一个切向向量，限制在一个角度范围内，考虑前一步的切向向量
    """
    random_vector = np.random.rand(3)
    while np.allclose(random_vector, normal) or np.allclose(random_vector, -normal):
        random_vector = np.random.rand(3)

    # 计算切向向量在法向量上的投影
    tangent_projection = np.dot(random_vector, normal) * normal

    # 计算切向向量（随机向量和投影向量的差）
    tangent_direction = random_vector - tangent_projection
    tangent_direction /= np.linalg.norm(tangent_direction)  # 归一化切向向量

    return tangent_direction

def is_within_ur10_range(clientID , workspace_radius=1000):
    """
    检查给定位置是否在 UR10 最大活动范围内
    """
    _,base_handle = sim.simxGetObjectHandle(clientID,'UR10',sim.simx_opmode_blocking)
    _,link_handle = sim.simxGetObjectHandle(clientID,'UR10_link7',sim.simx_opmode_blocking)
    _,base_position = sim.simxGetObjectPosition(clientID,base_handle,-1,sim.simx_opmode_blocking)
    _,link_position = sim.simxGetObjectPosition(clientID,link_handle,-1,sim.simx_opmode_blocking)
    # 计算位置到基座的距离
    # distance_to_base = np.linalg.norm(position)
    distance_to_base = math.sqrt((base_position[0] - link_position[0])**2 + (base_position[1] - link_position[1])**2 + (base_position[2] - link_position[2])**2)
    # 如果距离小于工作空间半径，则位置在 UR10 最大活动范围内
    if distance_to_base <= workspace_radius:
        return True
    else:
        return False

def get_current_vector(clientID):
    '''得到tip的方向向量
    '''
    _,tip_handle = sim.simxGetObjectHandle(clientID,'tip',sim.simx_opmode_blocking)
    # _ ,average_normal =  get_sensor_data(clientID,sensor_handles)
    _, self_direation = sim.simxGetObjectOrientation(clientID,tip_handle,-1,sim.simx_opmode_blocking)
    alpha = self_direation[0]  # Z
    beta = self_direation[1]   # Y
    gamma = self_direation[2]  # X
            
    # ZYX顺序旋转后的X轴方向（正方向）
    x = math.cos(gamma) * math.cos(beta)
    y = math.sin(gamma) * math.cos(beta)
    z = -math.sin(beta)
    negative_x_vector = [x, -z, -y]
    #xyz的计算结果有问题，只有调整成[x, -z, -y]才对
    return negative_x_vector


def vector_angle(v1, v2):
    """计算两个向量之间的角度"""
    unit_v1 = v1 / np.linalg.norm(v1)
    unit_v2 = v2 / np.linalg.norm(v2)
    dot_product = np.dot(unit_v1, unit_v2)
    angle = np.arccos(dot_product)
    return np.degrees(angle)

def get_force_handle(clientID):
    _,handle1 = sim.simxGetObjectHandle(clientID,'Force_sensor',sim.simx_opmode_blocking)
    _,handle2 = sim.simxGetObjectHandle(clientID,'Force_sensor0',sim.simx_opmode_blocking)
    _,handle3 = sim.simxGetObjectHandle(clientID,'Force_sensor1',sim.simx_opmode_blocking)
    _,handle4 = sim.simxGetObjectHandle(clientID,'Force_sensor2',sim.simx_opmode_blocking)
    _,handle5 = sim.simxGetObjectHandle(clientID,'Force_sensor3',sim.simx_opmode_blocking)
    _,handle6 = sim.simxGetObjectHandle(clientID,'Force_sensor4',sim.simx_opmode_blocking)
    _,handle7 = sim.simxGetObjectHandle(clientID,'Force_sensor5',sim.simx_opmode_blocking)
    _,handle8 = sim.simxGetObjectHandle(clientID,'Force_sensor6',sim.simx_opmode_blocking)
    _,handle9 = sim.simxGetObjectHandle(clientID,'Force_sensor7',sim.simx_opmode_blocking)
    _,handle10 = sim.simxGetObjectHandle(clientID,'Force_sensor8',sim.simx_opmode_blocking)
    _,handle11 = sim.simxGetObjectHandle(clientID,'Force_sensor9',sim.simx_opmode_blocking)
    _,handle12 = sim.simxGetObjectHandle(clientID,'Force_sensor10',sim.simx_opmode_blocking)
    force_sensor_handles = []
    force_sensor_handles = [handle1, handle2, handle3, handle4, handle5, handle6, handle7, handle8, handle9, handle10, handle11, handle12]
    return force_sensor_handles

def get_all_forces(clientID,force_sensor_handles):
    
    force_z = []
    for i in range(12):
        _,state,force,torquevector = sim.simxReadForceSensor(clientID,force_sensor_handles[i],sim.simx_opmode_blocking)
        force = force if force is not None else 0
        force_z.append(force[2])
    return force_z


def get_tangential_distance_and_vector(force_list):
    print(force_list)
    data = np.array([
    [0, force_list[0], force_list[1], 0],
    [force_list[2], force_list[3], force_list[4], force_list[5]],
    [force_list[6], force_list[7], force_list[8], force_list[9]],
    [0, force_list[10], force_list[11], 0]
])
    grid_size = 4
    # 原点坐标设置在网格中心
    origin = np.array([1.5, 1.5])
    
    # 初始化总力矩向量
    total_torque_vector = np.array([0.0, 0.0])
    f_all = sum(force_list)
    # 遍历每个点
    Mx_all = 0
    My_all = 0
    for i in range(grid_size):
        for j in range(grid_size):
            # 当前点的位置
            position = np.array([(j - origin[1]), (origin[0] - i)])
            # 计算位置向量（从原点指向当前点）
            
            
            F_z = -data[i, j]
            # 计算力矩向量（叉积）
            Mx = F_z*position[0]/f_all
            My = F_z*position[1]/f_all
            torque_vector = np.array([position[1] * F_z, -position[0] * F_z])  # 这里是对力矩向量的计算，可以用叉积简化为r[1] * F_z，-r[0] * F_z
            # 将力矩向量添加到总力矩向量中
            Mx_all = Mx_all + Mx
            My_all = My_all + My
            total_torque_vector += torque_vector
    print('COPX与COPy',Mx_all,My_all)
    # 归一化总力矩向量，得到旋转轴的方向向量
    # np.linalg.norm(total_torque_vector)
    distance = 0.1*np.linalg.norm(total_torque_vector)
    rotation_axis = total_torque_vector / np.linalg.norm(total_torque_vector)
    rotation_axis = [rotation_axis[1],-rotation_axis[0]]
    # rotation_axis = [rotation_axis[1],0]
    return rotation_axis,distance,Mx_all
    
    
def move_away(clientID,targetHandle,distance,average_normal):
    _,currentposition = sim.simxGetObjectPosition(clientID,targetHandle,-1,sim.simx_opmode_blocking)
    target_position = currentposition + distance*average_normal
    sim.simxSetObjectPosition(clientID,targetHandle,-1,target_position,sim.simx_opmode_oneshot)
    print('后退')

def move_towards(clientID, targetHandle, distance,average_normal):
    _, currentPosition = sim.simxGetObjectPosition(clientID, targetHandle, -1, sim.simx_opmode_blocking)
    
    targetPosition = currentPosition - distance * average_normal
    sim.simxSetObjectPosition(clientID, targetHandle, -1, targetPosition, sim.simx_opmode_oneshot)
    # print('前进')
def move_backwards(clientID, targetHandle, distance,average_normal):
    _, currentPosition = sim.simxGetObjectPosition(clientID, targetHandle, -1, sim.simx_opmode_blocking)
    
    targetPosition = currentPosition - distance * average_normal
    sim.simxSetObjectPosition(clientID, targetHandle, -1, targetPosition, sim.simx_opmode_oneshot)
    
def move_self_forward(clientID,targetHandle,distance):
    _,tip_handle = sim.simxGetObjectHandle(clientID,'tip',sim.simx_opmode_blocking)
    
    _, self_direation = sim.simxGetObjectOrientation(clientID,tip_handle,-1,sim.simx_opmode_blocking)
    alpha = self_direation[0]  # Z
    beta = self_direation[1]   # Y
    gamma = self_direation[2]  # X
    
    # ZYX顺序旋转后的X轴方向（正方向）
    x = math.cos(gamma) * math.cos(beta)
    y = math.sin(gamma) * math.cos(beta)
    z = -math.sin(beta)
    negative_x_vector = [-x, -y, -z]
    _,current_position = sim.simxGetObjectPosition(clientID,tip_handle,-1,sim.simx_opmode_blocking)
    
    target_position = [current_position[0]+negative_x_vector[0]*distance,
                       current_position[1]+negative_x_vector[1]*distance,
                       current_position[2]+negative_x_vector[2]*distance,]
    
    sim.simxSetObjectPosition(clientID, targetHandle, -1, target_position, sim.simx_opmode_blocking)        


   
    
def move_sideways(clientID, targetHandle, distance,average_normal):
    """
    向侧面随机移动，以便使机械臂末端构成连续的凸轨迹
    """
    _, currentPosition = sim.simxGetObjectPosition(clientID, targetHandle, -1, sim.simx_opmode_blocking)

    tangent_direction = get_tangent_direction(average_normal)
    # print("切向",tangent_direction)
    # 计算移动后的末端位置
    new_position = currentPosition - tangent_direction * distance

    sim.simxSetObjectPosition(clientID, targetHandle, -1, new_position, sim.simx_opmode_blocking)
    print('侧移')
    return new_position

def move_sideways_with_adjusted_distance(clientID,vector,distance):
    '''vector基于当前UR10_connection基坐标系的方向移动'''
    vector = np.array([vector[0],vector[1],0])
    _,objecthandle = sim.simxGetObjectHandle(clientID,'UR10_connection',sim.simx_opmode_blocking)
    res, Orientation = sim.simxGetObjectOrientation(clientID,objecthandle , -1, sim.simx_opmode_blocking)
    _, targetHandle = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_blocking)
# 构建旋转矩阵
    rotationMatrix = np.eye(3)
    rotationMatrix = np.dot(rotationMatrix, np.array([[np.cos(Orientation[2]), -np.sin(Orientation[2]), 0],
                                                    [np.sin(Orientation[2]), np.cos(Orientation[2]), 0],
                                                    [0, 0, 1]]))
    rotationMatrix = np.dot(rotationMatrix, np.array([[np.cos(Orientation[1]), 0, np.sin(Orientation[1])],
                                                    [0, 1, 0],
                                                    [-np.sin(Orientation[1]), 0, np.cos(Orientation[1])]]))
    rotationMatrix = np.dot(rotationMatrix, np.array([[1, 0, 0],
                                                    [0, np.cos(Orientation[0]), -np.sin(Orientation[0])],
                                                    [0, np.sin(Orientation[0]), np.cos(Orientation[0])]]))

    # 将向量乘以旋转矩阵来转换到世界坐标系下
    Vector_world = np.dot(rotationMatrix, vector)
    _, currentPosition = sim.simxGetObjectPosition(clientID, targetHandle, -1, sim.simx_opmode_blocking)
    new_position = currentPosition + Vector_world * distance
    move_distance = Vector_world*distance
    print('位移量------------------',move_distance)
    Vector_world[2] = 0
    num_steps = 5
    step_distance = distance / num_steps
    for i in range(num_steps):
        new_position = currentPosition + Vector_world * step_distance * (i + 1)
        sim.simxSetObjectPosition(clientID, targetHandle, -1, new_position, sim.simx_opmode_blocking)
        time.sleep(0.3)
    
     
    

from scipy.spatial.transform import Rotation as R

def align_object_with_normal_using_quaternion(clientID, targetHandle, normal_vector):
    """
    使用四元数将targetHandle的x轴与给定的normal_vector对齐
    """
    
    normal_vector = np.array(normal_vector)
    normal_vector = normal_vector / np.linalg.norm(normal_vector)
    
    _,plane_handle = sim.simxGetObjectHandle(clientID,'Plane',sim.simx_opmode_blocking)

    # _,position = sim.simxGetObjectPosition(clientID,plane_handle,targetHandle,sim.simx_opmode_blocking)
    
    # print("相对位置===",position)
    # 获取targetHandle姿态的四元数,current_rotation是四元数转化为相对于世界的旋转矩阵，apply就是将旋转矩阵应用于世界的x轴：1，0，0向量，那么旋转之后就是target的x轴在世界下的方向向量current-z
    _, current_quaternion = sim.simxGetObjectQuaternion(clientID, targetHandle, -1, sim.simx_opmode_blocking)
    current_rotation = R.from_quat(current_quaternion)

    # 如果叉积结果接近0（即两向量平行），旋转角度为0。否则得到旋转轴与旋转角度，标准化旋转轴，并计算两向量之间的角度，使用点积和arccos函数
    current_z = current_rotation.apply([1, 0, 0])
    print('current_z',current_z)  #非常接近(1,0,0)
    print('normal-vector',normal_vector)
    rot_axis = np.cross(current_z, normal_vector)
    # print(rot_axis)
    if np.linalg.norm(rot_axis) < 1e-3:  # 如果两个向量已经对齐
        rot_angle = 0
    else:
        rot_axis = rot_axis / np.linalg.norm(rot_axis)
        rot_angle = np.arccos(np.clip(np.dot(current_z, normal_vector), -1.0, 1.0))
        print("target的x方向需要旋转",rot_angle)
    # 创建四元数表示的旋转，四元数表示绕一个轴与绕该轴旋转的角度
        rotation = R.from_rotvec(rot_angle * rot_axis)
        # 组合旋转：将新旋转应用于当前旋转
        new_rotation = rotation * current_rotation
        new_quaternion = new_rotation.as_quat()

        # 应用新的四元数
        sim.simxSetObjectQuaternion(clientID, targetHandle, -1, new_quaternion.tolist(), sim.simx_opmode_blocking)
    '''以下为验证代码'''
    # _, current_quaternion = sim.simxGetObjectQuaternion(clientID, targetHandle, -1, sim.simx_opmode_blocking)
    # current_rotation = R.from_quat(current_quaternion)
    # _, current_ori = sim.simxGetObjectOrientation(clientID, targetHandle, -1, sim.simx_opmode_blocking)
    # print('对比1====',current_rotation,current_ori)
    # current_z = current_rotation.apply([1, 0, 0])
    # print('对比====',normal_vector,current_z)
    
    # print('校准后结果=',sensor_distances)
    
tangential_distances = []
CoPx = []


def main():
    base_coefficient = 10
    push_times = 0
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if clientID != -1:
        print('Connected to CoppeliaSim')

        # 获取IK目标和传感器的句柄
        _, targetHandle = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_blocking)
        sensor_handles = []
        sensor_handles = [sim.simxGetObjectHandle(clientID, f'Proximity_sensor{i-1}' if i > 0 else 'Proximity_sensor', sim.simx_opmode_blocking)[1] for i in range(5)]    
        force_sensor_handles = get_force_handle(clientID)
        while True:    
            sensor_distances, average_normal = get_sensor_data(clientID, sensor_handles)
            align_object_with_normal_using_quaternion(clientID, targetHandle, average_normal)   #只是让target handle这个dummy与average_normal对齐了，但是机械臂的tip dummy移动到target的位置还需要时间。
            print('正在对齐，五个输出',sensor_distances)
            time.sleep(3)
            if max(sensor_distances)-min(sensor_distances) < 0.0005:
                print('对齐完成')
                break
            else:
                print("realigning")
        # threshold = 0.005
        threshold = 0.01
        while True:
            move_towards(clientID,targetHandle,0.0005,average_normal)
            # time.sleep(0.1)
            # time.sleep(0.1)
            push_times += 1
            force_list = get_all_forces(clientID,force_sensor_handles)
            # move_backwards(clientID,targetHandle,0.1,average_normal)
            vector,tangential_distance,copx = get_tangential_distance_and_vector(force_list)
            print(f'Tangential Distance: {tangential_distance}')
            print(f'Vector方向{vector}')
            tangential_distances.append(tangential_distance)
            CoPx.append(copx)
            if tangential_distance > threshold:
                print('Tangential distance exceeded threshold, adjusting...')
                # 后退并重新对齐
                move_backwards(clientID, targetHandle, -0.01, average_normal)
                time.sleep(1)
                print('后退成功,准备对齐')
                if push_times>300:
                    coefficient = 10/base_coefficient
                else:
                    coefficient = base_coefficient-push_times/150
                while True:
                    sensor_distances, average_normal = get_sensor_data(clientID, sensor_handles)
                    align_object_with_normal_using_quaternion(clientID, targetHandle, average_normal)
                    time.sleep(3)  # 等待对齐稳定
                    print('正在对齐，五个输出',sensor_distances)
                    if max(sensor_distances) - min(sensor_distances) < 0.0005:
                        print('对齐完成')
                        break
                    else:
                        print('Realigning...')
                        
                move_sideways_with_adjusted_distance(clientID, vector, tangential_distance*coefficient)
                time.sleep(1)
                print('成功侧移')
                push_times =0
                while True:
                    sensor_distances, average_normal = get_sensor_data(clientID, sensor_handles)
                    align_object_with_normal_using_quaternion(clientID, targetHandle, average_normal)
                    time.sleep(3)  # 等待对齐稳定
                    if max(sensor_distances) - min(sensor_distances) < 0.0005:
                        print('Realign complete')
                        break
                    else:
                        print('Realigning...')
                    
            else:
                print('Tangential distance within acceptable range.')
    else:
        print('Failed connecting to CoppeliaSim')

import datetime
now = datetime.datetime.now()
formatted_time = now.strftime("%Y-%m-%d")
def save_data_to_excel(filename=f'tangential_distances_{formatted_time}.xlsx'):
    # 将数据保存到 Excel 文件
    df = pd.DataFrame({'Tangential Distance': tangential_distances,'CoPx':CoPx})
    df.to_excel(filename, index=False)
    print("已保存")
    
if __name__ == '__main__':
    try:
        main()
    finally:
        save_data_to_excel()

    
