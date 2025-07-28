import pyrealsense2 as rs
import numpy as np
import cv2

# 1. 필터 객체 생성
decimation    = rs.decimation_filter()     # 해상도 축소
depth2disp    = rs.disparity_transform(True)  # Depth → Disparity
spatial       = rs.spatial_filter()        # 공간적 노이즈 제거
temporal      = rs.temporal_filter()       # 시간적 노이즈 제거
disp2depth    = rs.disparity_transform(False) # Disparity → Depth

# (선택) 필터 파라미터 조정 예시
# decimation.set_option(rs.option.filter_magnitude, 2)
# spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
# spatial.set_option(rs.option.filter_smooth_delta, 20)
# temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
# temporal.set_option(rs.option.filter_smooth_delta, 20)

# 2. 파이프라인 구성
pipeline = rs.pipeline()
config   = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)
profile  = pipeline.start(config)

# align: Depth → Color 정합용
align = rs.align(rs.stream.color)

try:
    while True:
        # 3. 프레임 가져오기 & 정렬
        frames        = pipeline.wait_for_frames()
        aligned       = align.process(frames)
        depth_frame   = aligned.get_depth_frame()
        color_frame   = aligned.get_color_frame()

        # 4. 원본과 필터링된 깊이 프레임 얻기
        # 원본
        depth_orig = depth_frame

        # 필터링: Decimation → Depth→Disparity → Spatial → Temporal → Disparity→Depth
        filtered = decimation.process(depth_frame)
        filtered = depth2disp.process(filtered)
        filtered = spatial.process(filtered)
        filtered = temporal.process(filtered)
        filtered = disp2depth.process(filtered)

        # 5. 시각화용 컬러맵 변환
        colorizer = rs.colorizer()
        depth_orig_color = np.asanyarray(colorizer.colorize(depth_orig).get_data())
        depth_filt_color = np.asanyarray(colorizer.colorize(filtered).get_data())
        color_image      = np.asanyarray(color_frame.get_data())

        # 6. 화면 합치기 & 출력
        top = np.hstack((color_image, color_image))
        bot = np.hstack((depth_orig_color, depth_filt_color))
        canvas = np.vstack((top, bot))

        cv2.namedWindow('Post-Processing: [L]orig vs [R]filtered', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Post-Processing: [L]orig vs [R]filtered', canvas)

        if cv2.waitKey(1) == 27:  # ESC 키로 종료
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
