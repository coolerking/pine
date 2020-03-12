# -*- coding: utf-8 -*-
"""
RATF倉庫情報関連ユーティリティ関数群を提供するモジュール。
"""
import numpy as np

def get_color_dict():
    """
    vision用の走行抵抗マップの色指定（辞書型）を取得する。
    引数：
        なし
    戻り値：
        vision用の走行抵抗マップの色指定（辞書型）
    """
    # 緑無地
    return {0.001: '#00984C', 0.3: '#00984C', 0.002: '#00984C'}

def get_course_node_data(course_type):
    """
    config.py/myconfig.py オブジェクトに定義されたCOURSE_TYPEに従って
    コースデータを作成する。
    引数：
        course_type                 COURSE_TYPE値（コースタイプ）
    戻り値：
        course_node_numbers         周回路に含まれるコーナーノードリスト
        course_node_numbers_nodes   course_node_numbersに加えてコーナーノード間のすべてのノードを含むリスト
    例外：
        ValueError                  COURSE_TYPE値がサポート外文字列であった場合
    """
    # 以下、AI_Pilotテスト用の周回コースのノードリスト ------------------------------------
    # course_node_numbersは、周回路に含まれるコーナーノード　⇒　主にシミュレータのAuto_Pilot用
    # course_node_numbers_nodesは、course_node_numbersに加えてコーナーノード間のすべてのノードを含む　⇒　主にローダーへの走路指示用
    if course_type is None or course_type == 'INNER_CLOCKWISE':
        # 内周/時計回り
        course_node_numbers = [25, 37, 33, 29, 25, 37]  # inner clockwise
        course_node_numbers_nodes = [25, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 37]  # inner with existing nodes
    elif course_type == 'INNER_COUNTER_CLOCKWISE':
        # 内周/反時計回り
        course_node_numbers = [25, 29, 33, 37, 25, 29]  # inner counter-clockwise
        course_node_numbers_nodes = [25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 25, 26]  # inner with existing nodes
    elif course_type == 'U_TURN':
        # Ｕターン
        course_node_numbers = [25, 37, 33, 12, 7, 2, 25, 37]  # inner clockwise
        course_node_numbers_nodes = [25, 40, 39, 38, 37, 36, 35, 34, 33, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 25, 40]  # inner with existing nodes
    elif course_type == 'I_TURN':
        # Iターン 超信地旋回が必要
        course_node_numbers = [25, 37, 33, 37, 25, 37]  # inner clockwise
        course_node_numbers_nodes = [25, 40, 39, 38, 37, 36, 35, 34, 33, 34, 35, 36, 37, 38, 39, 40, 25, 40]
    elif course_type == 'CONVEX_CLOCKWISE':
        # 凸：時計回り
        course_node_numbers = [25, 37, 36, 9, 11, 34, 33, 29, 25, 37]
        course_node_numbers_nodes = [25,  40, 39, 38, 37, 36, 9, 10, 11, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 40]
    elif course_type == 'BIG_CONVEX_CLOCKWISE':
        # big凸：時計回り
        course_node_numbers = [1, 4, 39, 37, 33, 31, 16, 19, 1, 4]
        course_node_numbers_nodes = [1, 2, 3, 4, 39, 38, 37, 36, 35, 34, 33, 32, 31, 16, 17, 18, 19, 20, 21, 22, 23, 24, 1, 2]
    elif course_type == 'CONVEX_INNER_CLOCKWISE':
        # 内周＋車線変更　（凸x2：時計回り）
        course_node_numbers = [25, 37, 36, 9, 11, 34, 33, 32, 15, 17, 30, 29, 25, 37]
        course_node_numbers_nodes = [25,  40, 39, 38, 37, 36, 9, 10, 11, 34, 33, 32, 15, 16, 17, 30, 29, 28, 27, 26, 25, 40]
    elif course_type == 'CONVEX_INNER_CLOCKWISE2':
        # 内周＋車線変更
        course_node_numbers = [25, 37, 35, 10, 13, 19, 22, 27, 25, 37]
        course_node_numbers_nodes = [25,  40, 39, 38, 37, 36, 35, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 27, 26, 25, 40]
    elif course_type == 'CONVEX':
        # 内外周＋車線変更
        course_node_numbers = [1, 4, 39, 37, 35, 10, 13, 16, 31, 29, 27, 22, 1, 4]
        course_node_numbers_nodes = [1, 2, 3, 4, 39, 38, 37, 36, 35, 10, 11, 12, 13, 14, 15, 16, 31, 30, 29, 28, 27, 22, 23, 24, 1, 2]
    elif course_type == 'CONVEX_OUTER_CLOCKWISE':
        course_node_numbers = [1, 7, 13, 19, 1, 7] # outer clockwise
        course_node_numbers_nodes = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 1, 2]
    elif course_type == 'INNER_CLOCKWISE_LEFT_LONG':
        course_node_numbers = [25, 37, 14, 18, 25, 37]  # inner 右に横長 clockwise
        course_node_numbers_nodes = [25, 40, 39, 38, 37, 36, 35, 34, 33, 14, 15, 16, 17, 18, 29, 28, 27, 26, 25, 40]
    elif course_type == 'CLOCKWISE':
        course_node_numbers = [24, 37, 14, 19, 24, 37]  # inner & outer clockwise
        course_node_numbers_nodes = [24, 25, 40, 39, 38, 37, 36, 35, 34, 33, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25]
    elif course_type == 'CLOCKWISE_HEIGHT':
        # self.course_node_numbers_01 = [25, 37, 36, 9, 11, 34, 33, 29, 25, 37]
        course_node_numbers = [25, 37, 36, 9, 11, 34, 33, 29, 28, 21, 23, 26, 25, 37] # tate clockwise
        course_node_numbers_nodes = [25,  40, 39, 38, 37, 36, 9, 10, 11, 34, 33, 32, 31, 30, 29, 28, 21, 22, 23, 26, 25, 40] # tate with nodes
    elif course_type == 'CLOCKWISE_WIDTH':
        course_node_numbers = [25, 40, 3, 5, 36, 37, 38, 32, 15, 17, 30, 29, 25, 40] # yoko clockwise
        course_node_numbers_nodes = [25, 40, 3, 4, 5, 38, 37, 36, 35, 34, 33, 32, 15, 16, 17, 30, 29, 28, 27, 26, 25, 40]
        # self.course_node_numbers = [25, 40, 3, 4, 39, 37, 35, 10, 11, 34, 33, 32, 15, 17, 30, 29, 28, 21, 23, 26, 25, 40]
    else:
        raise ValueError('[get_course_node_data] course_type={} not supported'.format(str(course_type)))
    return course_node_numbers, course_node_numbers_nodes

def init_weight_data(dataname):
    """
    ノード間Legの重み付けを読み込む
    引数：
        dataname ノード間Legの重み付けファイルへのパス
    戻り値：
        ノード間Legの重み付けデータ
    """
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

def init_node_data(dataname):
    """
    ノード座標データを読み込む
    引数：
        dataname    ノード座標データファイル
    戻り値：
        ノード座標データ
    """
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

def init_resistance_data(dataname, num):
    """
    箱庭倉庫俯瞰図のピクセル毎の走行抵抗（転がり抵抗）を読み込む
    引数：
        dataname    箱庭倉庫俯瞰図のピクセル毎の走行抵抗（転がり抵抗）データファイルへのパス
        num         X軸のグリッド数
    戻り値：
        箱庭倉庫俯瞰図のピクセル毎の走行抵抗（転がり抵抗）データ
    """
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

def init_beacon_data(dataname):
    """
    箱庭倉庫に設置されたstationary beacon 4基の箱庭座標系での設置位置座標を読み込む
    引数：
        dataname    箱庭倉庫に設置されたstationary beacon 4基の箱庭座標系での
                    設置位置座標データファイルへのパス
    戻り値：
        箱庭倉庫に設置されたstationary beacon 4基の箱庭座標系での設置位置座標データ
    """
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

def init_garden_axis():
    """
    garden_axis の取得。
    引数：
        なし
    戻り値：
        なし
    """
    return [0, 0, -1*41]

def course_line(node_numbers, node_list):
    """
    走行経路ノードリストを取得する。
    引数：
        node_numbers        周回路に含まれるコーナーノードリスト
        node_list           コース上の全ノードのリスト
    戻り値：
        走行経路ノードリスト(np.array型)
    """
    coordinates = []
    mysterious_offset = [12, 12]
    for i in range(len(node_numbers)):
        coordinates.append(node_list[node_numbers[i] - 1] + mysterious_offset)
    return np.array(coordinates)