import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from scipy.spatial.transform import Rotation as R
import numpy as np
import requests

sess = requests.Session()

def get_quaternion(server_url="http://localhost:59666/quaternion"):
    try:
        # 发送GET请求
        response = sess.get(server_url)
        
        # 检查请求是否成功
        if response.status_code == 200:
            # 解析返回的四元数值
            quaternion_str = response.text
            #去掉第一位和最后一位的括号
            quaternion_str = quaternion_str[2:-2]
            quaternion = list(map(float, quaternion_str.split(",")))
            return quaternion
        else:
            print(f"Failed to get quaternion. Status code: {response.status_code}")
            return None
    except requests.exceptions.RequestException as e:
        print(f"An error occurred: {e}")
        return None

# 定义正方体的顶点和面
initial_vertices = np.array([
    [1, 1, -1],
    [1, -1, -1],
    [-1, -1, -1],
    [-1, 1, -1],
    [1, 1, 1],
    [1, -1, 1],
    [-1, -1, 1],
    [-1, 1, 1]
], dtype=np.float32)

edges = [
    [0, 1],
    [1, 2],
    [2, 3],
    [3, 0],
    [4, 5],
    [5, 6],
    [6, 7],
    [7, 4],
    [0, 4],
    [1, 5],
    [2, 6],
    [3, 7]
]

surfaces = [
    [0, 1, 2, 3],
    [3, 2, 6, 7],
    [7, 6, 5, 4],
    [4, 5, 1, 0],
    [5, 6, 2, 1],
    [7, 4, 0, 3]
]

colors = [
    [1, 0, 0],  # 红色
    [0, 1, 0],  # 绿色
    [0, 0, 1],  # 蓝色
    [1, 1, 0],  # 黄色
    [1, 0, 1],  # 紫色
    [0, 1, 1]   # 青色
]

def draw_cube(transformed_vertices):
    glBegin(GL_QUADS)
    for i, surface in enumerate(surfaces):
        glColor3fv(colors[i])
        for vertex in surface:
            glVertex3fv(transformed_vertices[vertex])
    glEnd()

    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(transformed_vertices[vertex])
    glEnd()


def transform_vertices(vertices, quaternion):
    # 将四元数转换为旋转矩阵
    rotation_matrix = R.from_quat([quaternion[1], quaternion[2], quaternion[3], quaternion[0]]).as_matrix()

    # 对顶点应用旋转
    transformed_vertices = np.dot(vertices, rotation_matrix.T)  # 旋转矩阵需要转置
    return transformed_vertices

def main():
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -10)  # 将相机向后移动，确保正方体在视野内

    # 启用深度测试
    glEnable(GL_DEPTH_TEST)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        # 获取最新的四元数
        quaternion = get_quaternion()

        # 对顶点进行变换
        transformed_vertices = transform_vertices(initial_vertices, quaternion)

        # 清除颜色和深度缓冲区
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # 绘制立方体
        draw_cube(transformed_vertices)

        # 刷新显示
        pygame.display.flip()
        pygame.time.wait(10)

if __name__ == "__main__":
    main()