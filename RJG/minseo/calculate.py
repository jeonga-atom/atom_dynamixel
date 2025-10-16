import cv2
import numpy as np


class ObjectManipulator:

    @staticmethod
    def mask_centroid_moments(mask, contours):
        """
        중심좌표 계산
        컨투어 기반 모멘트 방법: cv2.findContours + cv2.moments
        외곽 컨투어들만 찾은 뒤, 가장 큰 컨투어 하나를 골라 cv2.moments(contour) 로 중심을 계산.
        """
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None
        cx = float(M['m10'] / M['m00'])
        cy = float(M['m01'] / M['m00'])

        return cx, cy, largest

    @staticmethod
    def simple_clean_mask(mask, kernel_size=5):
        """
        간단한 노이즈 제거: closing -> opening
        """
        kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kern, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern, iterations=1)
        
        return mask

    @staticmethod
    def calculate_contour_angle_square(contour):
        """
        컨투어의 최소 면적 사각형을 사용해 x축과의 기울기 각도를 계산.
        """
        if contour is None or len(contour) < 3:
            return None
        rect = cv2.minAreaRect(contour)
        angle = rect[2]
        if angle < 0:
            angle += 180

        return float(angle)

    @staticmethod
    def calculate_contour_angle_cross(contour):
        """
        컨투어의 최소 면적 사각형을 사용해 x축과의 기울기 각도를 계산. -> 팔이 아닌 대각선으로 기준으로 계산됨.
        """
        if contour is None or len(contour) < 3:
            return None
        rect = cv2.minAreaRect(contour)
        angle = rect[2]
        angle -= 45

        if angle < 0:
            angle += 180

        return float(angle)

    @staticmethod
    # 십자형은 허프+PCA 기반 정밀 각도 추정
    def angle_cross_precise(mask_bin,
                            canny1=50, canny2=150,  # Canny 엣지 검출 임곗값. 작을수록 선이 많이 생김(노이즈↑), 클수록 선이 끊길 수 있음.
                            hough_thresh=50,        # cv2.HoughLinesP에서 선분으로 인정받기 위한 누적 투표 임계값.
                            min_line_frac=0.12,     # 최소 선분 길이 비율 (W * 0.12)
                            max_line_gap=10,
                            cluster_bw_deg=12.0     # 대표 각도 주변 군집 폭 (±도)
                            ):
        """
        십자형의 팔 방향을 허프+PCA로 정밀 추정.
        mask_bin: 원본 크기 이진 마스크 (0/1 or 0/255)
        반환값: angle_deg [0,90) 또는 None
        """
        H, W = mask_bin.shape[:2]
        edges = cv2.Canny((mask_bin * 255).astype(np.uint8), canny1, canny2)

        min_len = int(max(10, W * min_line_frac))   
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=hough_thresh,
                                minLineLength=min_len, maxLineGap=max_line_gap)
        if lines is None or len(lines) == 0:
            return None

        segs = []
        for x1, y1, x2, y2 in lines[:, 0, :]:
            dx, dy = (x2 - x1), (y2 - y1)
            L = float(np.hypot(dx, dy))
            if L < 1.0:
                continue
            a = np.degrees(np.arctan2(dy, dx))
            if a < 0: a += 180.0
            segs.append((x1, y1, x2, y2, a, L))
        if not segs:
            return None

        # 90° 접기
        angs_fold = np.array([(a if a < 90 else a - 90) for *_, a, _ in segs], dtype=np.float32)
        hist, bins = np.histogram(angs_fold, bins=36, range=(0.0, 90.0))
        k = int(np.argmax(hist))
        angle0 = float((bins[k] + bins[k + 1]) * 0.5)  # 대표 팔 방향(초안)

        def ang_dist(a, c):
            d = abs((a if a < 90 else a - 90) - c)
            return d

        inA, inB = [], []
        for (x1, y1, x2, y2, a, L) in segs:
            dA = ang_dist(a, angle0)
            dB = ang_dist((a + 90.0) % 180.0, angle0)
            if dA <= cluster_bw_deg:
                inA.append((x1, y1, x2, y2, a, L))
            elif dB <= cluster_bw_deg:
                inB.append((x1, y1, x2, y2, a, L))

        sumA = sum(s[-1] for s in inA)
        sumB = sum(s[-1] for s in inB)
        chosen = inA if sumA >= sumB else inB
        if not chosen:
            return angle0

        pts = []
        for (x1, y1, x2, y2, a, L) in chosen:
            pts.append([x1, y1])
            pts.append([x2, y2])
        P = np.array(pts, dtype=np.float32)
        mean, eigenvecs = cv2.PCACompute(P, mean=None)[:2]
        vx, vy = eigenvecs[0]
        ang = np.degrees(np.arctan2(vy, vx))
        if ang < 0:
            ang += 180.0
        if ang >= 90.0:
            ang -= 90.0
        return float(ang)
    
    @staticmethod
    def angle_square_precise(mask_bin,
                         canny1=40, canny2=120,
                         hough_thresh=50,
                         min_line_frac=0.10,   # 이미지 폭의 10% 이상
                         max_line_gap=8,
                         cluster_bw_deg=10.0   # 대표각 ±10°
                         ):
        """
        정사각형 회전각 정밀 추정. 반환 각도는 [0, 90) 도.
        mask_bin: 0/1 또는 0/255 이진 마스크 (원본 크기)
        실패 시 None
        """
        H, W = mask_bin.shape[:2]
        edges = cv2.Canny((mask_bin.astype(np.uint8) * 255), canny1, canny2)

        min_len = int(max(8, W * min_line_frac))    # 허프 선분 검출 -> 너무 짧은 조각은 버리기 위해 minLineLength 적용.
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=hough_thresh,
                                minLineLength=min_len, maxLineGap=max_line_gap)
        if lines is None or len(lines) == 0:
            return None

        segs = []  # (a_fold, L)
        for x1, y1, x2, y2 in lines[:, 0, :]:
            dx, dy = float(x2 - x1), float(y2 - y1)
            L = (dx*dx + dy*dy) ** 0.5
            if L < 1.0: 
                continue
            a = np.degrees(np.arctan2(dy, dx))  # [-180,180)
            if a < 0: a += 180.0                # [0,180)
            a_fold = a if a < 90.0 else a - 90.0  # [0,90), 정사각형의 90° 대칭성 때문에 90° 주기로 폴딩
            segs.append((a_fold, L))
        if not segs:
            return None

        angs = np.array([a for a, _ in segs], dtype=np.float32)
        hist, bins = np.histogram(angs, bins=36, range=(0.0, 90.0))
        k = int(np.argmax(hist))
        angle0 = float((bins[k] + bins[k+1]) * 0.5) # 중심값을 angle0로 취함.

        sel = [(a, L) for (a, L) in segs if abs(a - angle0) <= cluster_bw_deg]
        if not sel:
            sel = segs

        num = sum(a * L for a, L in sel)
        den = sum(L for _, L in sel) + 1e-6
        ang = num / den
        if ang < 0: ang += 90.0
        if ang >= 90.0: ang -= 90.0
        
        return float(ang)




