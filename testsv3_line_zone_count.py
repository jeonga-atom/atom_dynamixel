#line zone 내부에 들어오면 카운팅
import cv2
import supervision as sv
import numpy as np

from ultralytics import YOLO


def main():
    model=YOLO("yolov8n.pt")
    tracker = sv.ByteTrack() #트래킹의 세밀함을 위해 supervision의 Bytetrack 사용

    start, end = sv.Point(x=240, y=0), sv.Point(x=240, y=640)
    line_zone = sv.LineZone(start=start, end=end) #라인존의 위치 지정

    box_annotator = sv.BoxAnnotator()
    label_annotator = sv.LabelAnnotator()
    line_zone_annotator = sv.LineZoneAnnotator() #라인존 주석화시 세부 설정, 여기서 thickness, scale등 조절

    results=model.track(source=0, show=True, stream= True)

    for result in results: #단순 이미지 처리가 아닌 영상 처리에서는 for문을 반드시 사용하여 프레임 가공
        frame = result.orig_img
        detections = sv.Detections.from_ultralytics(result) #물체 감지를 위해 sv.Detections 명령어 활성화
        detections = tracker.update_with_detections(detections) #바이트 트랙 활성화 (트래킹 세밀화)
        
        detections = detections[detections.class_id == 67] #추가 조건: 폰만 감지하여 카운트 하기 위해 67로 설정
        
        line_zone.trigger(detections=detections) #라인존 내부의 감지체들만 트리거 되도록 설정 #최종적인 감지의 조건, 라인존 내부의 감지체들만 감지하고 카운트함.

        frame = box_annotator.annotate(scene=frame, detections=detections)
        label_annotator.annotate(scene=frame, detections=detections)
        line_zone_annotator.annotate(frame, line_zone)

        cv2.imshow("test", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
    
    

if __name__ == "__main__":
    main()



