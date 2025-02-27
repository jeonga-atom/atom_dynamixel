
# import cv2
# import numpy as np

# from ultralytics import YOLO

# # YOLO 모델 로드
# model = YOLO("yolov8n.pt")

# # 카메라 캡처 시작
# cap = cv2.VideoCapture(4)

# # 현재 선택된 물병 인덱스
# selected_bottle_idx = 0


# def find_closest_box(current_box, bottles, direction):
#     """ 방향(←↑→↓)에 따라 가장 가까운 물병 선택 """
#     if not bottles:
#         return 0  # 감지된 물병이 없으면 첫 번째 인덱스 반환

#     x1, y1, x2, y2 = map(int, current_box)
#     cx, cy = (x1 + x2) // 2, (y1 + y2) // 2  # 중심 좌표

#     min_dist = float("inf")
#     closest_idx = selected_bottle_idx

#     for i, box in enumerate(bottles):
#         if i == selected_bottle_idx:
#             continue  # 현재 선택된 물병은 제외

#         bx1, by1, bx2, by2 = map(int, box)
#         bx, by = (bx1 + bx2) // 2, (by1 + by2) // 2  # 다른 물병 중심 좌표

#         dx, dy = bx - cx, by - cy  # 현재 물병과의 거리

#         # 방향에 맞는 물병 찾기
#         if direction == "left" and dx < 0 and abs(dx) > abs(dy):  # 왼쪽에 있는 물병
#             dist = abs(dx)
#         elif direction == "right" and dx > 0 and abs(dx) > abs(dy):  # 오른쪽
#             dist = abs(dx)
#         elif direction == "up" and dy < 0 and abs(dy) > abs(dx):  # 위쪽
#             dist = abs(dy)
#         elif direction == "down" and dy > 0 and abs(dy) > abs(dx):  # 아래쪽
#             dist = abs(dy)
#         else:
#             continue

#         # 더 가까운 물병이 있으면 선택
#         if dist < min_dist:
#             min_dist = dist
#             closest_idx = i

#     return closest_idx


# while cap.isOpened():
#     ret, frame = cap.read()
#     if not ret:
#         break

#     # YOLO를 사용한 객체 감지
#     results = model(frame)

#     bottles = []  # 물병 바운딩 박스 저장

#     for result in results:
#         if result.boxes is None or len(result.boxes) == 0:
#             continue

#         for box, cls in zip(result.boxes.xyxy, result.boxes.cls):
#             class_id = int(cls)
#             class_name = model.names[class_id]  # 클래스 이름 가져오기

#             if class_name == "bottle":  # 물병만 선택
#                 bottles.append(box)

#     # 선택된 물병 인덱스 보정
#     if bottles:
#         selected_bottle_idx %= len(bottles)
#         selected_box = bottles[selected_bottle_idx]

#         for i, box in enumerate(bottles):
#             x1, y1, x2, y2 = map(int, box)

#             if i == selected_bottle_idx:
#                 color = (0, 255, 0)  # 초록색 (선택된 물병)
#                 thickness = 3
#             else:
#                 color = (100, 100, 100)  # 회색 (비활성 물병)
#                 thickness = 1

#             cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)

#     # 화면 출력
#     cv2.imshow("YOLO Bottle Selector", frame)

#     # 키 입력 대기
#     key = cv2.waitKey(1) & 0xFF
#     if key == ord('q'):  # 'q' 키를 누르면 종료
#         break
#     elif key == 81 and bottles:  # 왼쪽 화살표 (←)
#         selected_bottle_idx = find_closest_box(selected_box, bottles, "left")
#     elif key == 83 and bottles:  # 오른쪽 화살표 (→)
#         selected_bottle_idx = find_closest_box(selected_box, bottles, "right")
#     elif key == 82 and bottles:  # 위쪽 화살표 (↑)
#         selected_bottle_idx = find_closest_box(selected_box, bottles, "up")
#     elif key == 84 and bottles:  # 아래쪽 화살표 (↓)
#         selected_bottle_idx = find_closest_box(selected_box, bottles, "down")

# # 종료
# cap.release()
# cv2.destroyAllWindows()


#물체 여러 개 인식하고 class 입력 후 화살표 키로 물체 선택할 수 있음
import cv2
import numpy as np
from ultralytics import YOLO

# YOLO 모델 로드
model = YOLO("yolov8n.pt")

# 카메라 캡처 시작
cap = cv2.VideoCapture(4)

# 현재 선택된 물체 인덱스
selected_obj_idx = 0

# 필터링할 클래스 ID (기본값: None = 모든 물체 표시)
filter_class_id = None
input_number = ""  # 키보드로 입력받은 숫자 저장


def find_closest_box(current_box, objects, direction):
    """ 방향(←↑→↓)에 따라 가장 가까운 물체 선택 """
    if not objects:
        return 0  # 감지된 물체가 없으면 첫 번째 인덱스 반환

    x1, y1, x2, y2 = map(int, current_box)
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2  # 중심 좌표

    min_dist = float("inf")
    closest_idx = selected_obj_idx

    for i, box in enumerate(objects):
        if i == selected_obj_idx:
            continue  # 현재 선택된 물체는 제외

        bx1, by1, bx2, by2 = map(int, box)
        bx, by = (bx1 + bx2) // 2, (by1 + by2) // 2  # 다른 물체 중심 좌표

        dx, dy = bx - cx, by - cy  # 현재 물체와의 거리

        # 방향에 맞는 물체 찾기
        if direction == "left" and dx < 0 and abs(dx) > abs(dy):  # 왼쪽에 있는 물체
            dist = abs(dx)
        elif direction == "right" and dx > 0 and abs(dx) > abs(dy):  # 오른쪽
            dist = abs(dx)
        elif direction == "up" and dy < 0 and abs(dy) > abs(dx):  # 위쪽
            dist = abs(dy)
        elif direction == "down" and dy > 0 and abs(dy) > abs(dx):  # 아래쪽
            dist = abs(dy)
        else:
            continue

        # 더 가까운 물체가 있으면 선택
        if dist < min_dist:
            min_dist = dist
            closest_idx = i

    return closest_idx


while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # YOLO를 사용한 객체 감지
    results = model(frame)

    objects = []  # 감지된 물체 바운딩 박스 저장
    object_classes = []  # 물체의 클래스 ID 저장

    for result in results:
        if result.boxes is None or len(result.boxes) == 0:
            continue

        for box, cls, conf in zip(result.boxes.xyxy, result.boxes.cls, result.boxes.conf):
            class_id = int(cls)

            # 신뢰도(conf) 0.5 이상인 물체만 인식
            if conf >= 0.5:
                # 특정 ID의 물체만 필터링 (filter_class_id가 None이면 모든 물체 표시)
                if filter_class_id is None or class_id == filter_class_id:
                    objects.append(box)
                    object_classes.append(class_id)

    # 선택된 물체 인덱스 보정
    if objects:
        selected_obj_idx %= len(objects)
        selected_box = objects[selected_obj_idx]

        for i, box in enumerate(objects):
            x1, y1, x2, y2 = map(int, box)
            obj_id = object_classes[i]

            if i == selected_obj_idx:
                color = (0, 255, 0)  # 초록색 (선택된 물체)
                thickness = 3
            else:
                color = (100, 100, 100)  # 회색 (비활성 물체)
                thickness = 1

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
            cv2.putText(frame, f"ID: {obj_id}", (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    # 화면 출력
    cv2.imshow("YOLO Object Selector", frame)

    # 키 입력 대기
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):  # 'q' 키를 누르면 종료
        break
    elif key == ord('r'):  # 'r' 키를 누르면 전체 물체 인식 모드로 복귀
        filter_class_id = None
        input_number = ""
    elif key in range(48, 58):  # 숫자 키(0~9) 입력 처리
        input_number += chr(key)  # 입력한 숫자를 문자열에 추가
    elif key == 13 and input_number:  # Enter(13) 입력 시 특정 ID 필터링
        filter_class_id = int(input_number)
        input_number = ""
    elif key == 81 and objects:  # 왼쪽 화살표 (←)
        selected_obj_idx = find_closest_box(selected_box, objects, "left")
    elif key == 83 and objects:  # 오른쪽 화살표 (→)
        selected_obj_idx = find_closest_box(selected_box, objects, "right")
    elif key == 82 and objects:  # 위쪽 화살표 (↑)
        selected_obj_idx = find_closest_box(selected_box, objects, "up")
    elif key == 84 and objects:  # 아래쪽 화살표 (↓)
        selected_obj_idx = find_closest_box(selected_box, objects, "down")

# 종료
cap.release()
cv2.destroyAllWindows()



