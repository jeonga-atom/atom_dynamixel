# # #1번
from ultralytics import YOLO

import numpy as np
import cv2
import supervision as sv


def main():
    model = YOLO('yolo11n.pt')
    tracker = sv.ByteTrack() #트래킹의 세밀함을 위해 supervision의 Bytetrack 사용

    box_annotator = sv.BoxAnnotator()   #바운딩 박스 시각화
    label_annotator = sv.LabelAnnotator()   #객체 라벨 시각화

    results=model.track(source=4, stream= True)

    for result in results: #단순 이미지 처리가 아닌 영상 처리에서는 for문을 반드시 사용하여 프레임 가공
        frame = result.orig_img #현재 프레임 이미지 추출
        detections = sv.Detections.from_ultralytics(result) #물체 감지를 위해 sv.Detections 명령어 활성화
        detections = tracker.update_with_detections(detections) #바이트 트랙 활성화 (트래킹 세밀화)
        
        detections = detections[detections.class_id == 67] #추가 조건: 폰만 감지-> 67로 설정
        #여러 클래스를 감지 할 수 있음 ex)detection.class_id in [67, 1, 2]
        
        #바운딩 박스 및 라벨 표시
        frame = box_annotator.annotate(scene=frame, detections=detections)
        label_annotator.annotate(scene=frame, detections=detections)

        for box, class_id in zip(result.boxes.xyxy.cpu(), result.boxes.cls.cpu()):  # result를 직접 사용
            if int(class_id) == 67:
                x1, y1, x2, y2 = box.numpy() #바운딩 박스 좌표 추출

                #바운딩 박스 중심 좌표 계산
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)

                # #바운딩 박스의 너비, 높이, 면적 계산-> 굳이 필요없음
                # width, height, area = annotator.get_bbox_dimension(box)
                # print(f"Bounding Box - Width: {width.item()}, Height: {height.item()}, Area: {area.item()}")

                #영상에 좌표 출력
                cv2.putText(frame, f"({center_x}, {center_y})", 
                        (center_x + 10, center_y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, (0, 255, 0), 1, cv2.LINE_AA)
                
                print(center_x, center_y)

        cv2.imshow("test", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
    

if __name__ == "__main__":
    main()



#2번
# from ultralytics import YOLO
# import cv2

# def main():
#     model = YOLO('yolo11n.pt')
#     results = model.track(source=0, show=True, stream=True, show_boxes=True ,show_conf=True)  # 신뢰도 표시 활성화

#     for result in results:
#         frame = result.orig_img

#         # 좌표 계산 및 표시
#         for box in result.boxes.xyxy.cpu():  # 바운딩 박스 좌표 가져오기
#             x1, y1, x2, y2 = box.numpy()

#             # 바운딩 박스 중심 좌표 계산
#             center_x = int((x1 + x2) / 2)
#             center_y = int((y1 + y2) / 2)

#             # 중심 좌표 영상에 표시
#             cv2.putText(frame, f"({center_x}, {center_y})",
#                         (center_x + 10, center_y),
#                         cv2.FONT_HERSHEY_SIMPLEX,
#                         0.5, (0, 255, 0), 1, cv2.LINE_AA)

#         cv2.imshow("test", frame)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()




#3번-라인 중간에 표시
# from ultralytics.utils.plotting import Annotator
# from ultralytics import YOLO

# import numpy as np
# import cv2
# import supervision as sv


# def main():
#     model = YOLO('yolov8n.pt')
#     tracker = sv.ByteTrack() #트래킹의 세밀함을 위해 supervision의 Bytetrack 사용

#     start, end = sv.Point(x=240, y=0), sv.Point(x=240, y=640)
#     line_zone = sv.LineZone(start=start, end=end) #라인존의 위치 지정

#     box_annotator = sv.BoxAnnotator()   #바운딩 박스 시각화
#     label_annotator = sv.LabelAnnotator()   #객체 라벨 시각화
#     line_zone_annotator = sv.LineZoneAnnotator() #라인존 주석화시 세부 설정, 여기서 thickness, scale등 조절


#     results=model.track(source=0, show=True, stream= True)

#     for result in results: #단순 이미지 처리가 아닌 영상 처리에서는 for문을 반드시 사용하여 프레임 가공
#         frame = result.orig_img #현재 프레임 이미지 추출
#         detections = sv.Detections.from_ultralytics(result) #물체 감지를 위해 sv.Detections 명령어 활성화
#         detections = tracker.update_with_detections(detections) #바이트 트랙 활성화 (트래킹 세밀화)
        
#         detections = detections[detections.class_id == 67] #추가 조건: 폰만 감지-> 67로 설정
#         #여러 클래스를 감지 할 수 있음 ex)detection.class_id in [67, 1, 2]
        
#         #바운딩 박스 및 라벨 표시
#         frame = box_annotator.annotate(scene=frame, detections=detections)
#         label_annotator.annotate(scene=frame, detections=detections)
#         line_zone_annotator.annotate(frame, line_zone)

#         #객체 좌표 및 바운딩 박스 정보 표시
#         # annotator = Annotator(frame, example=model.names) #굳이 필요가 있나-> 굳이 필요 없음

#         for box, class_id in zip(result.boxes.xyxy.cpu(), result.boxes.cls.cpu()):  # result를 직접 사용
#             if int(class_id) == 67:
#                 x1, y1, x2, y2 = box.numpy() #바운딩 박스 좌표 추출

#                 #바운딩 박스 중심 좌표 계산
#                 center_x = int((x1 + x2) / 2)
#                 center_y = int((y1 + y2) / 2)

#                 # #바운딩 박스의 너비, 높이, 면적 계산-> 굳이 필요없음
#                 # width, height, area = annotator.get_bbox_dimension(box)
#                 # print(f"Bounding Box - Width: {width.item()}, Height: {height.item()}, Area: {area.item()}")

#                 #영상에 좌표 출력
#                 cv2.putText(frame, f"({center_x}, {center_y})", 
#                         (center_x + 10, center_y), 
#                         cv2.FONT_HERSHEY_SIMPLEX, 
#                         0.5, (0, 255, 0), 1, cv2.LINE_AA)



#         cv2.imshow("test", frame)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
#     cv2.destroyAllWindows()
    

# if __name__ == "__main__":
#     main()