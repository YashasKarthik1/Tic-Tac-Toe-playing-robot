#!/usr/bin/env python3
import cv2
import numpy as np
import time
import serial
import serial.tools.list_ports

from pymycobot.mycobot import MyCobot

IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"


class Object_detect():

    def __init__(self, camera_x=160, camera_y=15):
        self.frame_count = 2
        super(Object_detect, self).__init__()
        self.mc = None

        self.plist = [
            str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()

        ]

        self.move_angles = [
            [0.61, 45.87, -92.37, -41.3, 2.02, 9.58],  # init the point
            [18.8, -7.91, -54.49, -23.02, -0.79, -14.76],  # point to grab
        ]

        self.x1 = self.x2 = self.y1 = self.y2 = 0
        self.cache_x = self.cache_y = 0
        self.sum_x1 = self.sum_x2 = self.sum_y2 = self.sum_y1 = 0
        self.camera_x, self.camera_y = camera_x, camera_y
        self.c_x, self.c_y = 0, 0
        self.ratio = 0
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def run(self):
        self.mc = MyCobot(self.plist[0], 115200)
        self.mc.send_angles([0.61, 45.87, -92.37, -41.3, 2.02, 9.58], 20)
        time.sleep(2.5)

    def transform_frame(self, frame):
        fx = 1.5
        fy = 1.5
        frame = cv2.resize(frame, (0, 0), fx=fx, fy=fy,
                           interpolation=cv2.INTER_CUBIC)
        if self.x1 != self.x2:
            frame = frame[int(self.y2*0.78):int(self.y1*1.1),
                          int(self.x1*0.86):int(self.x2*1.08)]
        return frame

    def get_calculate_params(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectImaPoint = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        """
        Two Arucos must be present in the picture and in the same order.
        There are two Arucos in the Corners, and each aruco contains the pixels of its four corners.
        Determine the center of the aruco by the four corners of the aruco.
        """
        if len(corners) > 0:
            if ids is not None:
                if len(corners) <= 1 or ids[0] == 1:
                    return None
                x1 = x2 = y1 = y2 = 0
                point_11, point_21, point_31, point_41 = corners[0][0]
                x1, y1 = int((point_11[0] + point_21[0] + point_31[0] + point_41[0]) / 4.0), int(
                    (point_11[1] + point_21[1] + point_31[1] + point_41[1]) / 4.0)
                point_1, point_2, point_3, point_4 = corners[1][0]
                x2, y2 = int((point_1[0] + point_2[0] + point_3[0] + point_4[0]) / 4.0), int(
                    (point_1[1] + point_2[1] + point_3[1] + point_4[1]) / 4.0)

                return x1, x2, y1, y2
        return None

    def draw_marker(self, img, x, y):
        cv2.rectangle(
            img,
            (x - 20, y - 20),
            (x + 20, y + 20),
            (0, 255, 0),
            thickness=2,
            lineType=cv2.FONT_HERSHEY_COMPLEX,
        )
        cv2.putText(img, "({},{})".format(x, y), (x, y),
                    cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (243, 0, 0), 2,)

    def set_params(self, c_x, c_y, ratio):
        self.c_x = c_x
        self.c_y = c_y
        self.ratio = 220.0/ratio

    def set_cut_params(self, x1, y1, x2, y2):
        self.x1 = int(x1)
        self.y1 = int(y1)
        self.x2 = int(x2)
        self.y2 = int(y2)

    def pump_on(self):
        self.mc.set_basic_output(2, 0)
        self.mc.set_basic_output(5, 0)

    def pump_off(self):
        self.mc.set_basic_output(2, 1)
        self.mc.set_basic_output(5, 1)


def tictactoeCenters(row, col):
    square_centers = {
        (0, 0): [220.5, 58.5],
        (0, 1): [220.1, 8.4],
        (0, 2): [220.1, -48.7],
        (1, 0): [170.5, 52.9],
        (1, 1): [170.2, 0.1],
        (1, 2): [170.2, -49.8],
        (2, 0): [115.5, 56.3],
        (2, 1): [115.3, 8.6],
        (2, 2): [115.1, -46.5]
    }
    return square_centers.get((row, col), None)


def detect_color(frame, lower_color, upper_color, color_name, min_rect_size):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    color_mask = cv2.inRange(hsv, lower_color, upper_color)

    contours, _ = cv2.findContours(
        color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    color_centers = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)

        if w > min_rect_size and h > min_rect_size:
            center_x = x + w // 2
            center_y = y + h // 2
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            color_centers.append([center_x, center_y])
    return frame, color_centers


COLS = 3
ROWS = 3


def initialize_board():
    return [[' ' for _ in range(COLS)] for _ in range(ROWS)]


def draw_board(frame):
    height, width, _ = frame.shape
    cell_width = width // COLS
    cell_height = height // ROWS
    range_x = [0, cell_width, cell_width * 2, cell_width * 3]
    range_y = [0, cell_height, cell_height * 2, cell_height * 3]

    for i in range(1, ROWS):
        cv2.line(frame, (0, i * cell_height),
                 (width, i * cell_height), (0, 0, 0), 1)

    for j in range(1, COLS):
        cv2.line(frame, (j * cell_width, 0),
                 (j * cell_width, height), (0, 0, 0), 1)

    return range_x, range_y


def draw_move(frame, row, col, player):
    height, width, _ = frame.shape
    cell_width = width // COLS
    cell_height = height // ROWS

    x = col * cell_width
    y = row * cell_height
    ratio = 30
    if player == 'X':
        cv2.line(frame, (x + ratio, y+ratio), (x + cell_width -
                 ratio, y + cell_height-ratio), (0, 0, 0), 2)
        cv2.line(frame, (x + cell_width - ratio, y+ratio),
                 (x + ratio, y + cell_height-ratio), (0, 0, 0), 2)
    elif player == 'O':
        center = (x + cell_width // 2, y + cell_height // 2)
        radius = min(cell_width, cell_height) // 2 - 5
        cv2.circle(frame, center, radius-5, (0, 0, 0), 2)
    else:
        pass


def get_grid_indices(x, y, x_list, y_list):
    xVal = next((i for i, val in enumerate(
        x_list) if val >= x), len(x_list) - 1)
    yVal = next((i for i, val in enumerate(
        y_list) if val >= y), len(y_list) - 1)

    return xVal-1, yVal-1


def print_board(board):
    for row in board:
        print(" | ".join(row))
        print("---------")


def move(detect, poseX, poseY):

    detect.mc.send_angles(detect.move_angles[1], 20)
    time.sleep(3)

    grabPose = [170.8, -170.0, 84.4, 179.87, -3.78, -62.75]
    aboveGrabPose = [156.2, -162.1, 188.8, 179.87, -3.78, -62.75]
    detect.mc.send_coords(aboveGrabPose, 40, 1)
    time.sleep(4)
    detect.mc.send_coords(grabPose, 40, 1)
    time.sleep(4)
    detect.pump_on()
    time.sleep(2)
    detect.mc.send_coords(aboveGrabPose, 40, 1)
    time.sleep(4)
    x, y = tictactoeCenters(poseX, poseY)
    detect.mc.send_coords([x, y, 103+50, 179.87, -3.78, -62.75], 40, 1)
    time.sleep(4)
    detect.mc.send_coords([x, y, 103, 179.87, -3.78, -62.75], 40, 1)
    time.sleep(5)
    detect.pump_off()
    time.sleep(2)
    detect.mc.send_coords([x, y, 103+50, 179.87, -3.78, -62.75], 40, 1)
    time.sleep(4)

    detect.mc.send_angles(detect.move_angles[0], 25)
    time.sleep(4.5)


def play_tic_tac_toe_with_camera(detect, cap):
    board = initialize_board()
    player_turn = True
    print("You can start off with the first move")
    print("#####################################\n")
    while True:
        ret, frame = cap.read()
        frame = detect.transform_frame(frame)
        frame = cv2.flip(frame, -1)
        range_x, range_y = draw_board(frame)
        if not ret:
            print("Error: Failed to capture frame.")
            break
        key = cv2.waitKey(1) & 0xFF

        if key == 27:
            break

        for i in range(ROWS):
            for j in range(COLS):
                if board[i][j] != ' ':
                    draw_move(frame, i, j, board[i][j])

        if key == ord(' ') and player_turn:

            image = frame
            cv2.waitKey(5)
            result_image_blue, blue_centers = detect_color(
                image, lower_red, upper_red, "Blue", min_rect_size)
            for center in blue_centers:
                x, y = center
                xVal, yVal = get_grid_indices(x, y, range_x, range_y)
                if board[yVal][xVal] == ' ':
                    board[yVal][xVal] = 'X'
                    print("Players move -- ")

                    print_board(board)

                    cv2.imshow("Tic Tac Toe", image)
                    player_turn = not player_turn

        for i in range(ROWS):
            for j in range(COLS):
                if board[i][j] != ' ':
                    draw_move(frame, i, j, board[i][j])

        if not player_turn:
            best_move = find_best_move(board)
            if best_move:
                row, col = best_move
                board[row][col] = 'O'
                player_turn = not player_turn
            print("Computers move -- ")
            print_board(board)

            move(detect, row, col)
        cv2.imshow("Tic Tac Toe", frame)

        for i in range(ROWS):
            for j in range(COLS):
                if board[i][j] != ' ':
                    draw_move(frame, i, j, board[i][j])

        cv2.imshow("Tic Tac Toe", frame)
        if check_winner(board, 'X'):
            print("Player wins!")
            time.sleep(3)
            break
        elif check_winner(board, 'O'):
            print("Computer wins!")
            time.sleep(3)
            break
        elif is_board_full(board):
            print("It's a draw!")
            time.sleep(3)
            break

    cap.release()
    cv2.destroyAllWindows()


def check_winner(board, player):
    for i in range(ROWS):
        if all(board[i][j] == player for j in range(COLS)) or all(board[j][i] == player for j in range(ROWS)):
            return True

    if all(board[i][i] == player for i in range(ROWS)) or all(board[i][ROWS - 1 - i] == player for i in range(ROWS)):
        return True

    return False


def is_board_full(board):
    return all(board[i][j] != ' ' for i in range(ROWS) for j in range(COLS))


def get_move_from_keyboard():
    print("Enter row and column (separated by space):")
    move = input().split()
    row, col = map(int, move)
    return row-1, col-1


def get_available_moves(board):
    return [(i, j) for i in range(ROWS) for j in range(COLS) if board[i][j] == ' ']


def find_best_move(board):
    best_val = float('-inf')
    best_move = None

    for move in get_available_moves(board):
        row, col = move
        board[row][col] = 'O'
        move_val = minimax(board, 0, False)
        board[row][col] = ' '

        if move_val > best_val:
            best_move = move
            best_val = move_val

    return best_move


def minimax(board, depth, is_maximizing):
    if check_winner(board, 'O'):
        return 1
    elif check_winner(board, 'X'):
        return -1
    elif is_board_full(board):
        return 0

    if is_maximizing:
        max_eval = float('-inf')
        for move in get_available_moves(board):
            row, col = move
            board[row][col] = 'O'
            eval = minimax(board, depth + 1, False)
            board[row][col] = ' '
            max_eval = max(max_eval, eval)
        return max_eval
    else:
        min_eval = float('inf')
        for move in get_available_moves(board):
            row, col = move
            board[row][col] = 'X'
            eval = minimax(board, depth + 1, True)
            board[row][col] = ' '
            min_eval = min(min_eval, eval)
        return min_eval


if __name__ == "__main__":
    import platform
    if platform.system() == "Windows":
        cap_num = 1
        cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)

        if not cap.isOpened():
            cap.open(1)
    elif platform.system() == "Linux":
        cap_num = 4
        cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
        if not cap.isOpened():
            cap.open()

    detect = Object_detect()
    detect.run()
    detect.pump_off()

    _init_ = 20
    init_num = 0
    nparams = 0
    arukoOK = 0
    lower_green = np.array([35, 43, 35])
    upper_green = np.array([90, 255, 255])

    lower_blue = np.array([100, 43, 46])
    upper_blue = np.array([130, 255, 255])

    lower_red = np.array([0, 43, 46])
    upper_red = np.array([10, 255, 255])

    min_rect_size = 50

    while arukoOK == 0:
        _, frame = cap.read()
        frame = detect.transform_frame(frame)

        cv2.imshow("Tic Tac Toe", frame)
        cv2.waitKey(5)
        if _init_ > 0:
            _init_ -= 1
            continue

        if init_num < 20:
            if detect.get_calculate_params(frame) is None:
                continue
            else:
                x1, x2, y1, y2 = detect.get_calculate_params(frame)
                detect.draw_marker(frame, x1, y1)
                detect.draw_marker(frame, x2, y2)
                detect.sum_x1 += x1
                detect.sum_x2 += x2
                detect.sum_y1 += y1
                detect.sum_y2 += y2
                init_num += 1
                continue
        elif init_num == 20:
            detect.set_cut_params(
                (detect.sum_x1)/20.0-20,
                (detect.sum_y1)/20.0,
                (detect.sum_x2)/20.0,
                (detect.sum_y2)/20.0-20,
            )
            detect.sum_x1 = detect.sum_x2 = detect.sum_y1 = detect.sum_y2 = 0
            init_num += 1
            continue

        if nparams < 10:
            if detect.get_calculate_params(frame) is None:
                continue
            else:
                x1, x2, y1, y2 = detect.get_calculate_params(frame)
                detect.draw_marker(frame, x1, y1)
                detect.draw_marker(frame, x2, y2)
                detect.sum_x1 += x1
                detect.sum_x2 += x2
                detect.sum_y1 += y1
                detect.sum_y2 += y2
                nparams += 1
                continue
        elif nparams == 10:
            nparams += 1
            detect.set_params(
                (detect.sum_x1+detect.sum_x2)/20.0,
                (detect.sum_y1+detect.sum_y2)/20.0,
                abs(detect.sum_x1-detect.sum_x2)/10.0 +
                abs(detect.sum_y1-detect.sum_y2)/10.0
            )
            print("ok")
            arukoOK = 1
            continue
    play_tic_tac_toe_with_camera(detect, cap)
