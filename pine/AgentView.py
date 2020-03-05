from SpoolMobile import *
from marvelmind import MarvelmindHedge
from tkinter import *
from PIL import Image, ImageTk, ImageDraw
from numpy import *
import math
import pygame
from pygame.locals import *
from threading import Thread
import datetime
import time

args = sys.argv

dataname_weight = '1b1w.txt'  # 連結されたノード間の重み付け。移動コストによるコース選択判定に利用する。
dataname_node = '1b1n.txt'  # 各ノードの座標データ
dataname_rrmap = 'RRMap2.txt'  # landscape（152×120画像）の各画素ごとの走行抵抗値（転がり摩擦係数）
dataname_beacon = '1b1b.txt'  # Stationary beacon 4基の箱庭座標系における設置位置 (in studs)


# 入力画像である箱庭倉庫俯瞰図のサイズ
num_of_gridx = 152
num_of_gridy = 120

# AI入力画像のサイズ
vision_size_x = 160
vision_size_y = 120
grid_size = 1

debug = 0


# ノード間Legの重み付けを読み込む
def init_weight_data(dataname):
    f = open(dataname)
    lines = f.readlines()
    datalen = len(lines)
    f.close()
    data = np.zeros((datalen, datalen))
    for j in range (datalen):
        for i in range (datalen):
            sp = lines[i].split()
            data[i,j] = sp[j+1]
    return data

# ノード座標データを読み込む
def init_node_data(dataname):
    f = open(dataname)
    lines = f.readlines()
    datalen = len(lines)
    f.close()
    data = np.zeros((datalen, 2))
    for j in range (2):
        for i in range (datalen):
            sp = lines[i].split()
            data[i,j] = sp[j+1]
    return data


# 箱庭倉庫俯瞰図のピクセル毎の走行抵抗（転がり抵抗）を読み込む
def init_resistance_data(dataname, num):
    f = open(dataname)
    lines = f.readlines()
    datalen = len(lines)
    f.close()
    data = np.zeros((datalen, num))
    for j in range (num):
        for i in range (datalen):
            sp = lines[i].split()
            data[i,j] = sp[j+1]
    return data

# 箱庭倉庫に設置されたstationary beacon 4基の箱庭座標系での設置位置座標を読み込む
def init_beacon_data(dataname):
    f = open(dataname)
    lines = f.readlines()
    datalen = len(lines)
    f.close()
    data = np.zeros((datalen, 3))
    data0 = []
    for j in range (3):
        for i in range (datalen):
            sp = lines[i].split()
            if j==0:
                data0.append(sp[0])
            data[i,j] = sp[j+1]
    return data0, data


resistance_list = init_resistance_data(dataname_rrmap, num_of_gridx)  # 走行抵抗リスト
weight_list = init_weight_data(dataname_weight)  # 重み付けリスト
node_list = init_node_data(dataname_node)  # ノードリスト
beacon_address, beacon_position = init_beacon_data(dataname_beacon)  # 各Stationary beaconの座標情報

master = Tk()


def get_torch_view(overall_view, torch_radius, pos_x, pos_y, rotate_angle, v_scale, view_width, view_height, offset_x, offset_y):
    # mobileの位置を中心として、半径rのアルファチャンネルを作る
    im_a = Image.new("L", overall_view.size, 0)
    draw = ImageDraw.Draw(im_a)
    pos = [(pos_x + offset_x) * v_scale - torch_radius, (pos_y + offset_y) * v_scale - torch_radius, (pos_x + offset_x) * v_scale + torch_radius, (pos_y + offset_y) * v_scale + torch_radius]
    # print(pos)
    draw.ellipse(pos, fill=255)
    im_rgba = overall_view.copy()
    im_rgba.putalpha(im_a)
    im_rgba_crop = im_rgba.crop(pos)  # R120
    #　mobileのangleで回転
    rotate_angle = (360 - rotate_angle) % 360 - 180
    img = im_rgba_crop.rotate(rotate_angle)

    # 入力画像サイズ(160, 120)のバックグラウンドに張り付ける
    torch_view = Image.new("RGB", (view_width, view_height), 0)
    torch_view.paste(img, (int((view_width - torch_radius * 2) / 2), int((view_height - torch_radius * 2) / 2)))

    return torch_view


def course_line(node_numbers, node_list):
    coordinates = []
    mysterious_offset = [12, 12]
    for i in range(len(node_numbers)):
        coordinates.append(node_list[node_numbers[i] - 1] + mysterious_offset)
    return np.array(coordinates)


# 以下、AI_Pilotテスト用の周回コースのノードリスト ------------------------------------
# course_node_numbersは、周回路に含まれるコーナーノード　⇒　主にシミュレータのAuto_Pilot用
# course_node_numbers_nodesは、course_node_numbersに加えてコーナーノード間のすべてのノードを含む　⇒　主にローダーへの走路指示用
# 内周/時計回り
course_node_numbers = [25, 37, 33, 29, 25, 37]  # inner clockwise
course_node_numbers_nodes = [25, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 37]  # inner with existing nodes
# 内周/反時計回り
# course_node_numbers = [25, 29, 33, 37, 25, 29]  # inner counter-clockwise
# course_node_numbers_nodes = [25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 25, 26]  # inner with existing nodes
# Ｕターン
# course_node_numbers = [25, 37, 33, 12, 7, 2, 25, 37]  # inner clockwise
# course_node_numbers_nodes = [25, 40, 39, 38, 37, 36, 35, 34, 33, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 25, 40]  # inner with existing nodes
# Iターン 超信地旋回が必要
# course_node_numbers = [25, 37, 33, 37, 25, 37]  # inner clockwise
# course_node_numbers_nodes = [25, 40, 39, 38, 37, 36, 35, 34, 33, 34, 35, 36, 37, 38, 39, 40, 25, 40]
# 凸：時計回り
# course_node_numbers = [25, 37, 36, 9, 11, 34, 33, 29, 25, 37]
# course_node_numbers_nodes = [25,  40, 39, 38, 37, 36, 9, 10, 11, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 40]
# big凸：時計回り
# course_node_numbers = [1, 4, 39, 37, 33, 31, 16, 19, 1, 4]
# course_node_numbers_nodes = [1, 2, 3, 4, 39, 38, 37, 36, 35, 34, 33, 32, 31, 16, 17, 18, 19, 20, 21, 22, 23, 24, 1, 2]
# 内周＋車線変更　（凸x2：時計回り）
# course_node_numbers = [25, 37, 36, 9, 11, 34, 33, 32, 15, 17, 30, 29, 25, 37]
# course_node_numbers_nodes = [25,  40, 39, 38, 37, 36, 9, 10, 11, 34, 33, 32, 15, 16, 17, 30, 29, 28, 27, 26, 25, 40]
# 内周＋車線変更
# course_node_numbers = [25, 37, 35, 10, 13, 19, 22, 27, 25, 37]
# course_node_numbers_nodes = [25,  40, 39, 38, 37, 36, 35, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 27, 26, 25, 40]
# 内外周＋車線変更
# course_node_numbers = [1, 4, 39, 37, 35, 10, 13, 16, 31, 29, 27, 22, 1, 4]
# course_node_numbers_nodes = [1, 2, 3, 4, 39, 38, 37, 36, 35, 10, 11, 12, 13, 14, 15, 16, 31, 30, 29, 28, 27, 22, 23, 24, 1, 2]
# course_node_numbers = [1, 7, 13, 19, 1, 7] # outer clockwise
# course_node_numbers_nodes = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 1, 2]
# course_node_numbers = [25, 37, 14, 18, 25, 37]  # inner 右に横長 clockwise
# course_node_numbers_nodes = [25, 40, 39, 38, 37, 36, 35, 34, 33, 14, 15, 16, 17, 18, 29, 28, 27, 26, 25, 40]
# course_node_numbers = [24, 37, 14, 19, 24, 37]  # inner & outer clockwise
# course_node_numbers_nodes = [24, 25, 40, 39, 38, 37, 36, 35, 34, 33, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25]
# course_node_numbers_01 = [25, 37, 36, 9, 11, 34, 33, 29, 25, 37]
# course_node_numbers = [25, 37, 36, 9, 11, 34, 33, 29, 28, 21, 23, 26, 25, 37] # tate clockwise
# course_node_numbers_nodes = [25,  40, 39, 38, 37, 36, 9, 10, 11, 34, 33, 32, 31, 30, 29, 28, 21, 22, 23, 26, 25, 40] # tate with nodes
# course_node_numbers = [25, 40, 3, 5, 36, 37, 38, 32, 15, 17, 30, 29, 25, 40] # yoko clockwise
# course_node_numbers_nodes = [25, 40, 3, 4, 5, 38, 37, 36, 35, 34, 33, 32, 15, 16, 17, 30, 29, 28, 27, 26, 25, 40]
# course_node_numbers = [25, 40, 3, 4, 39, 37, 35, 10, 11, 34, 33, 32, 15, 17, 30, 29, 28, 21, 23, 26, 25, 40]
# ----------------------------------------------------------------------

# vision用の走行抵抗マップ（バックグラウンド）とりあえず緑無地
outfilepath_vision_background = 'out_box/vision_map.jpg'
color_list_green = {0.001: '#00984C', 0.3: '#00984C', 0.002: '#00984C'}
rrmap_vision_img = ResistanceMap(dataname_rrmap, color_list_green).generateImageFile(outfilepath_vision_background)

# フルサイズのビジョンイメージ
base_margin = 4  # 基礎エリアのオフセット
ladder_margin = 8  # ラダーエリアのオフセット
vision_scale = 4  # 入力画面として作成する画像の拡大率（整数倍）。1倍で152×120。
# テスト表示用のキャンバスの余白
vision_margin_x = 10
vision_margin_y = 10

vision_img_org = Landscape(rrmap_vision_img, weight_list, node_list, base_margin, ladder_margin, 1).create_1x1_landscape_vision_img_wide4(vision_margin_x, vision_margin_y, course_line(course_node_numbers_nodes, node_list), vision_scale)
vision = vision_img_org.copy()

# 松明ビジョンのテスト表示用のキャンバス
w = Canvas(master, width=vision_size_x + vision_margin_x * 2, height=vision_size_y + vision_margin_y * 2)
w.pack()

# エージェント生成
mobile = SpoolMobile(6, 28, 92, 180, 0.05, 0, 0, 0, 0, 0, 0, 10.625, 13.78, 1, 0, -6, 1000, 0, 8, resistance_list, 1, 0, 0, num_of_gridx, num_of_gridy, grid_size, None, vision_scale)

# ビジョン初期画面の生成
next_vision = vision.copy()
mobile.draw(canvas=None, vision=next_vision) # 引数canvasはシミュレータ用の画面を生成する場合に指定する
next_vision_cropped_resized = get_torch_view(next_vision, 60, mobile.loader[2].position[0, 0], mobile.loader[2].position[1, 0], mobile.loader[2].angle, vision_scale, 160, 120, vision_margin_x, vision_margin_y)
# 貼り付け用の画像イメージに変換
next_vision_img = ImageTk.PhotoImage(next_vision_cropped_resized)
w.delete("vision1")
w.create_image(vision_margin_x, vision_margin_y, image=next_vision_img, anchor=NW, tag="vision1")
master.update()


def init(root):
    root.update()
    root.lift()


# リアル倉庫からの実車の計測値（位置、向き）によってローダーを描画する
# 尾輪（キャスター）は固定（初期値0度）
def draw():
    global vision
    global next_vision
    global hedge_head_address, hedge_head_position
    global hedge_tail_address, hedge_tail_position
    if debug == 1:
        print('head(%d:%f:%f) tail(%d:%f:%f)' % (
            hedge_head_address, hedge_head_position[0, 0], hedge_head_position[1, 0], hedge_tail_address,
            hedge_tail_position[0, 0], hedge_tail_position[1, 0]))

    # angle
    y_length = hedge_head_position[1, 0] - hedge_tail_position[1, 0]
    x_length = hedge_head_position[0, 0] - hedge_tail_position[0, 0]
    angle = math.degrees(math.atan2(y_length, x_length))
    if debug == 1:
        print('atan2:(%f,%f,%f)' % (y_length, x_length, angle))

    if debug == 1:
        print("ビーコン＃{:d}".format(hedge_head_address))
        print(hedge_head_position)
        print("ビーコン＃{:d}".format(hedge_tail_address))
        print(hedge_tail_position)

    pos_x = hedge_head_position[0, 0]
    pos_y = hedge_head_position[1, 0]
    angle = 90 - angle
    if debug == 1:
        print('pos_x=%f, pos_y=%f, angle=%f' % (pos_x, pos_y, angle))
    if pos_x != np.nan and pos_y != np.nan and angle != np.nan:
        # 切り出していないフルサイズのビジョン画面にエージェントを描画する
        next_vision = vision.copy()
        for b in mobile.loader:
            b.position[0, 0], b.position[1, 0] = pos_x, pos_y
            b.angle = angle
            b.draw(vision=next_vision)

        # エージェントを描画したフルサイズのビジョン画面をそのときのエージェント位置に合わせて切り出し、入力画像サイズに成形する
        # これがローダーカメラ入力の画像
        next_vision_cropped_resized = get_torch_view(next_vision, 60, mobile.loader[2].position[0, 0],
                                                     mobile.loader[2].position[1, 0], mobile.loader[2].angle,
                                                     vision_scale, 160, 120, vision_margin_x, vision_margin_y)

        # テスト表示画面用に変換する
        next_vision_img = ImageTk.PhotoImage(next_vision_cropped_resized)

        # テスト表示キャンバスに張替
        w.delete("vision1")
        w.create_image(vision_margin_x, vision_margin_y, image=next_vision_img, anchor=NW, tag="vision1")
        master.update()
    else:
        print('!!!!! position or angle is nan !!!!!')


# Multilateration(三角測量) scenario -----------------------------------------------------------------------------------
# ビーコン座標系における座標の三角測量および箱庭座標系への座標変換
garden_axis = []
garden_axis.append(0)
garden_axis.append(0)
garden_axis.append(-1*41)
if debug == 1:
    print("箱庭座標系原点オフセット x:{:f} y:{:f} z:{:f}".format(garden_axis[0], garden_axis[1], garden_axis[2]))

def updatedMobileBeaconPosition(distances):
    # Chose three of four distance data
    # for i in (1,len(beacon_address)):
    # distances[2*i] = distances[2*i]/8  # in studs
    shortest_distance = math.inf
    for i in range(1, len(beacon_address) + 1):
        if distances[2 * i] < shortest_distance:
            shortest_distance = distances[2 * i]
            excluded_address = distances[2 * i - 1]
    if debug == 1:
        print("B{:d}:{:f} excluded".format(excluded_address, shortest_distance))

    # Rearrange the three distances of beacon addresses along clockwise
    tri_distances = []
    tri_beacon_address = []
    tri_beacon_position = []
    for j in range(len(beacon_address)):
        for i in range(1, len(beacon_address) + 1):
            if distances[2 * i - 1] != excluded_address:
                if distances[2 * i - 1] == int(beacon_address[j]):
                    # print("beacon_address[{:d}] = {:s}".format(j, beacon_address[j]))
                    tri_distances.append(distances[2 * i - 1])
                    # tri_beacon_address.append(distances[2*i - 1])
                    tri_distances.append(distances[2 * i] * 1000 / 8)
                    tri_beacon_address.append(beacon_address[j])
                    # print(beacon_position[j])
                    tri_beacon_position.append([beacon_position[j, 0], beacon_position[j, 1], beacon_position[j, 2]])

    if debug == 1:
        print("Multilateration: B{:d}:{:.3f}, B{:d}:{:.3f}, B{:d}:{:.3f}".format(tri_distances[0], tri_distances[1],
                                                                                 tri_distances[2], tri_distances[3],
                                                                                 tri_distances[4], tri_distances[5]))
        print(tri_beacon_address)
        print(tri_beacon_position)

    # Cal
    Leg_01 = math.sqrt((tri_beacon_position[1][0] - tri_beacon_position[0][0]) ** 2 + (
                tri_beacon_position[1][1] - tri_beacon_position[0][1]) ** 2 + (
                                   tri_beacon_position[1][2] - tri_beacon_position[0][2]) ** 2)
    Leg_02 = math.sqrt((tri_beacon_position[2][0] - tri_beacon_position[0][0]) ** 2 + (
                tri_beacon_position[2][1] - tri_beacon_position[0][1]) ** 2 + (
                                   tri_beacon_position[2][2] - tri_beacon_position[0][2]) ** 2)
    Leg_12 = math.sqrt((tri_beacon_position[2][0] - tri_beacon_position[1][0]) ** 2 + (
                tri_beacon_position[2][1] - tri_beacon_position[1][1]) ** 2 + (
                                   tri_beacon_position[2][2] - tri_beacon_position[1][2]) ** 2)
    if debug == 1:
        print("Leg_01:{:3f} Leg_02:{:3f} Leg_12:{:3f}".format(Leg_01, Leg_02, Leg_12))

    COS_C0 = (Leg_01 ** 2 + Leg_02 ** 2 - Leg_12 ** 2) / (2 * Leg_01 * Leg_02)
    U = Leg_01
    VX = Leg_02 * COS_C0
    VY = math.sqrt(Leg_02 ** 2 - VX ** 2)
    V = math.sqrt(VX ** 2 + VY ** 2)
    R0 = tri_distances[1]
    R1 = tri_distances[3]
    R2 = tri_distances[5]
    if debug == 1:
        print("U:{:f} VX:{:f} VY:{:f} V:{:f} R0:{:f} R1:{:f} R2:{:f}".format(U, VX, VY, V, R0, R1, R2))
    P = []
    PX = (R0 ** 2 - R1 ** 2 + U ** 2) / (2 * U)
    PY = (R0 ** 2 - R2 ** 2 + V ** 2 - 2 * VX * PX) / (VY * 2)
    # PZ cannot calculate when the z-distance is too small. it needs more than 300mm.
    # PZ = math.sqrt(R0**2 - PX**2 - PY**2)
    if R0 ** 2 - PX ** 2 - PY ** 2 <= 0:
        PZ = 100
    else:
        PZ = math.sqrt(R0 ** 2 - PX ** 2 - PY ** 2)
    if debug == 1:
        print("PX:{:3f} PY:{:3f} PZ:{:3f}".format(PX, PY, PZ))
    P = [PX, PY, PZ]
    if debug == 1:
        print(P)

    # 座標変換
    def make_rot_mat(positions):
        # x = positions[1][0] - positions[0][0]
        # y = positions[1][1] - positions[0][1]

        x = positions[1][0] - positions[0][0]
        y = positions[1][1] - positions[0][1]
        leg = math.sqrt(x ** 2 + y ** 2)
        if debug == 1:
            print("x:{:f} y:{:f} leg:{:f} sin(y/leg):{:f} cos(x/leg):{:f}".format(x, y, leg, np.sin(y / leg),
                                                                                  np.cos(x / leg)))
        # 採用されたベースライン（ベクトル）とビーコン座標系のｘ軸（ベクトル）とのなす角度をｚ軸に関して逆回転
        # rot_matrix = [[np.cos(x/leg), np.sin(y/leg), 0], [-1*np.sin(y/leg), np.cos(x/leg), 0], [0, 0, 1]]
        rot_matrix = [[x / leg, y / leg, 0], [-1 * y / leg, x / leg, 0], [0, 0, 1]]
        rot_matrix = np.array(rot_matrix)
        total_rot_matrix = rot_matrix

        return total_rot_matrix

    rotaion_matrix = make_rot_mat(tri_beacon_position)
    if debug == 1:
        print(rotaion_matrix)
    baseline_point = np.zeros((3, 1))
    baseline_point[0, 0], baseline_point[1, 0], baseline_point[2, 0] = PX, PY, PZ  # in studs
    if debug == 1:
        print(baseline_point)

    # 逆回転
    rotated_baseline_point = rotaion_matrix @ baseline_point
    if debug == 1:
        print("逆回転後")
        print(rotated_baseline_point)

    x180_matrix = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
    x180_matrix = np.array(x180_matrix)
    if debug == 1:
        print("180 now")
    rotated_baseline_point_180 = x180_matrix @ rotated_baseline_point
    if debug == 1:
        print(rotated_baseline_point_180)

    # 平行移動距離:Baseline座標系原点(tri_beacon_position)と箱庭座標系原点()の距離
    Para = np.zeros((3, 1))
    # for i in range(3):
    #    Para[i,0] = garden_axis[i] - tri_beacon_position[0][i]
    # Para[2,0] = 0

    Para[0, 0], Para[1, 0], Para[2, 0] = garden_axis[0] - tri_beacon_position[0][0], garden_axis[1] - \
                                         tri_beacon_position[0][1], garden_axis[2] - tri_beacon_position[0][2]

    # Para[0,0], Para[1,0], Para[2,0] = garden_axis[0] - tri_beacon_position[0][0], tri_beacon_position[0][1] - garden_axis[1], 0
    if debug == 1:
        print("平行移動距離")
        print(Para)
    if debug == 1:
        print("平行移動後")
    updated_garden_point = np.zeros((3, 1))
    updated_garden_point = rotated_baseline_point_180 - Para
    if debug == 1:
        print(updated_garden_point)
    # print("モバイルビーコン{:d}：x:{:f} y:{:f} z:{:f}".format(distances[0], updated_garden_point[0,0], updated_garden_point[1,0],updated_garden_point[2,0]))
    # updated_point_list=[]
    # for i in range(3):
    #    updated_point_list.append(garden_point[i,0])
    return distances[0], updated_garden_point


hedge_head_address = 6
hedge_tail_address = 5
hedge_head_position = np.zeros((3, 1))
hedge_tail_position = np.zeros((3, 1))

# ベクトルの終点となるHead（６番のモバイルビーコンを想定）について
# ビーコン座標系での三角測定および箱庭座標系への座標変換をする
def update_head_position():
    global hedge_head
    global hedge_head_address, hedge_head_position

    hedge_head_address, hedge_head_position = updatedMobileBeaconPosition(hedge_head.distances())
    hedge_head_position[0, 0] = round(hedge_head_position[0, 0], 1)
    hedge_head_position[1, 0] = round(hedge_head_position[1, 0], 1)
    hedge_head_position[2, 0] = round(hedge_head_position[2, 0], 1)
    # hedge_tail_address, hedge_tail_position = updatedMobileBeaconPosition(hedge_tail.distances())
    if debug == 1:
        print("ヘッドビーコン＃{:d}".format(hedge_head_address))
        print(hedge_head_position)
        # print("ビーコン＃{:d}".format(hedge_tail_address))
        # print(hedge_tail_position)

# ベクトルの始点となるTail（５番のモバイルビーコンを想定）について
# ビーコン座標系での三角測定および箱庭座標系への座標変換をする
def update_tail_position():
    global hedge_tail
    global hedge_tail_address, hedge_tail_position

    if debug == 1:
        hedge_tail.print_distances()
    # hedge_head_address, hedge_head_position = updatedMobileBeaconPosition(hedge_head.distances())
    hedge_tail_address, hedge_tail_position = updatedMobileBeaconPosition(hedge_tail.distances())
    hedge_tail_position[0, 0] = round(hedge_tail_position[0, 0], 1)
    hedge_tail_position[1, 0] = round(hedge_tail_position[1, 0], 1)
    hedge_tail_position[2, 0] = round(hedge_tail_position[2, 0], 1)
    if debug == 1:
        # print("ビーコン＃{:d}".format(hedge_head_address))
        # print(hedge_head_position)
        print("テイルビーコン＃{:d}".format(hedge_tail_address))
        print(hedge_tail_position)


# 以下　リアル倉庫モニター　---------------------------------------------------
# 指定されたcallback_intervalでcallback1を呼び出すだけのスレッド
class RealAgentMonitor(Thread):
    def __init__(self, dead_zone, callback_interval, callback1=None, callback2=None, callback3=None, callback4=None, callback5=None):
        Thread.__init__(self)
        self.callback1 = callback1
        self.callback2 = callback2
        self.callback3 = callback3
        self.callback4 = callback4
        self.callback5 = callback5
        self.dead_zone = dead_zone # 未使用
        self.callback_interval = callback_interval

    def run(self):
        while 1:
            # Δt（0.05秒）間隔でジョイスティック入力を反映する
            t = time.time()
            self.callback1()  # angle計算と描画
            calc_time_required = time.time() - t
            if calc_time_required > self.callback_interval: # callback(move)計算がΔtを超えた場合は通知
                print('Calculation time %f exceeds the callback period %f.' % (calc_time_required, self.callback_interval))
            else: # callback(move)計算がΔt以内の場合は残り時間を待機
                time.sleep(self.callback_interval - calc_time_required)


# Mobile beacon x 2でのローダー位置、向き測定
# ttyACM0にID=6のモバイルビーコンを期待している
# ベクトルの終点となるheadの座標update_tail_positionを呼び出す
hedge_head = MarvelmindHedge(tty = "/dev/ttyACM0", adr=None, recieveUltrasoundRawDataCallback=update_head_position, debug=False)
hedge_head.start()
# ttyACM１にID=５のモバイルビーコンを期待している
# ベクトルの始点となるTailの座標計算update_tail_positionを呼び出す
hedge_tail = MarvelmindHedge(tty = "/dev/ttyACM1", adr=None, recieveUltrasoundRawDataCallback=update_tail_position, debug=False)
hedge_tail.start()

# 最速20Hz(0.05毎秒）の周期で松明ビジョン画像を更新する
monitor = RealAgentMonitor(0.05, 0.05, callback1=draw)
monitor.start()

mainloop()
