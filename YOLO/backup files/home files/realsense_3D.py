#realsense 카메라를 사용하여 실시간으로 3D점군 데이터를 시각화하는 프로그램
#->카메라로 부터 깊이이미지와 컬러 이미지를 받아와서 3D공간에 점군을 그려주는 것

"""
Mouse: 
    Drag with left button to rotate around pivot (thick small axes), 
    with right button to translate and the wheel to zoom.
Keyboard: 
    [p]     Pause
    [r]     Reset View
    [d]     Cycle through decimation values
    [z]     Toggle point scaling
    [c]     Toggle color source
    [s]     Save PNG (./out.png)
    [e]     Export points to ply (./out.ply)
    [q\ESC] Quit
"""

import math     #math: 수학적 함수(각도 변환 등)를 위한 라이브러리
import time     #time: 시간관력 함수(프레임 시간 계산)를 위한 라이브러리
import cv2      #cv2: Open CV라이브러리, 이미지 처리
import numpy as np  #numpy: 배열 및 행렬 연산을 위한 라이브러리
import pyrealsense2 as rs   #pyrealsense2: realsense카메라와 통신하고 데이터를 처리하는 라이브러리

class AppState:     #AppState 클래스: 카메라 뷰와 상태를 관리하는 클래스-> 카메라의 위치, 회전, 스케일 등 상태를 추적

    def __init__(self, *args, **kwargs):
        self.WIN_NAME = 'RealSense'     #WIN_NAME: 창의 이름
        self.pitch, self.yaw = math.radians(-10), math.radians(-15)   #pitch, yaw: 카메라의 회전 각도
        self.translation = np.array([0, 0, -1], dtype=np.float32)     #translation: 카메라의 위치, 객체의 위치를 나타내는 백터,(x,y,z)->3D좌표
        self.distance = 2           #distance: 카메라의 거리
        self.prev_mouse = 0, 0      #prev_mouse: 이전 마우스 위치
        self.mouse_btns = [False, False, False]     #mouse_btns: 마우스 버튼 상태
        self.paused = False         #paused: 일시 정지 상태
        self.decimate = 1           #decimate: 포인트 클라우드의 축소 비율
        self.scale = True           #scale: 포인트 클라우드의 스케일링 여부
        self.color = True           #color: 색상 이미지 사용 여부

    def reset(self):                #reset메서드는 상태를 초기값으로 재설정
        self.pitch, self.yaw, self.distance = 0, 0, 2
        self.translation[:] = 0, 0, -1

    @property
    def rotation(self):             #rotation: 카메라의 회전 행렬을 계산, cv2.Rodrigues를 사용하여 회전 벡터->회전 행렬로 변환하는 OpenCV 함수, 3X3 크기의 회전행렬로 변환
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))   #pitch, yaw는 회전벡터를 나나내는 값, self.pitch는 X축에 대한 회전(피치각도)을 의미
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))     #self.yaw는 Y축에 대한 회전(요 각도)을 의미, 이 벡터들을 cv2.Rodrigues로 변환하여 Rx와 Ry라는 회전 행렬을 얻음
        return np.dot(Ry, Rx).astype(np.float32)    #np.dot(Ry, Rx)는 두 회전 행렬 Ry와 Rx의 곱을 계산, 두 회전 행렬을 곱하면, Y축 회전과 X축 회전을 결합한 새로운 회전 행렬을 얻음
                                                    #.astype(np.float32)는 계산된 결과를 np.float32 형으로 변환-> 계산 속도나 메모리 최적화를 위해 사용

    @property
    def pivot(self):    #pivot은 카메라의 회전 중심을 계산하는 속성, self.translation: 객체의 위치를 나타내는 벡터, (x,y,z)->3D좌표
        return self.translation + np.array((0, 0, self.distance), dtype=np.float32)     #객체가 self.translation 위치를 기준으로 회전한다고 했을 때, self.translation에서 z축 방향으로 self.distance만큼 이동한 위치가 회전 중심


state = AppState()      #AppState() 클래스의 인스턴스를 생성하여 상태를 초기화

# Configure depth and color streams
pipeline = rs.pipeline()    #카메라 스트리밍 파이프라인을 생성
config = rs.config()        #카메라 스트림 설정을 초기화

#카메라 장치와 파이프라인 프로필을 설정하여 카메라 장치 정보를 얻음->  RealSense 카메라와 연결하여 해당 카메라의 제품 라인 정보(예: D400 시리즈, L500 시리즈 등)를 추출하는 과정을 구현
pipeline_wrapper = rs.pipeline_wrapper(pipeline)    #rs.pipeline_wrapper는 RealSense SDK에서 사용하는 객체로, pipeline 객체를 감싸는 래퍼(wrapper) 역할, pipeline을 rs.pipeline_wrapper로 감싸 pipeline_wrapper라는 변수에 저장
pipeline_profile = config.resolve(pipeline_wrapper) #config.resolve()는 파이프라인을 실행할 때 사용할 설정 프로파일을 반환, config는 RealSense 카메라의 스트리밍 옵션을 설정하는 객체
device = pipeline_profile.get_device()              #pipeline_profile에서 연결된 카메라 장치(device)를 가져옴

# device_product_line = str(device.get_info(rs.camera_info.product_line))

#카메라에 RGB 센서가 있는지 확인하고, 없다면 프로그램을 종료
found_rgb = False   #RGB 카메라 센서를 찾았는지 여부, 처음: False, 카메라 찾으면 True
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

#깊이 및 색상 스트림을 30fps로 설정, 깊이는 z16 포맷, 색상은 bgr8 포맷을 사용
# 이 줄은 깊이 스트림을 설정, 깊이 스트림: realsense 카메라가 제공하는 3D 깊이 정보를 실시간으로 캡처하는 데이터 스트림으로 카메라와 물체사이의 거리를 측정하여 각 픽셀에 대한 깊이 값을 제공
# rs.stream.depth: 깊이 카메라 스트림을 활성화
# 640, 480: 카메라 해상도를 640x480 픽셀로 설정
# rs.format.z16: 깊이 데이터를 16비트 형식으로 설정, 각 픽셀의 값은 거리 정보를 의미함.
# 30: 초당 30프레임으로 스트리밍 설정
# config.enable_stream(rs.stream.depth, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, rs.format.z16, 30)
config.enable_stream(rs.stream.color, rs.format.bgr8, 30)


# 카메라 스트리밍 시작
pipeline.start(config)

# 카메라 스트림의 프로필을 얻고, 깊이 스트림의 내부 파라미터를 가져와 해상도를 추출
profile = pipeline.get_active_profile()     #활성화된 realsense 파이프라인 프로파일 가져옴 
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))    #profile.get_stream(rs.stream.depth): 활성화된 프로파일에서 깊이 스트림을 가져옴, 즉, depth_profile은 깊이 스트림에 대한 프로파일 객체
depth_intrinsics = depth_profile.get_intrinsics()   #depth_profile.get_intrinsics()는 깊이 카메라의 **내부 파라미터(intrinsics)**를 가져옴, 내부 파라미터는 카메라의 초점 거리, 주점, 왜곡계수 등을 포함, 이 벙보는 카메라의 3D 데이터를 2D 이미지로 투영하는데 필요한 정보
w, h = depth_intrinsics.width, depth_intrinsics.height  #w: 가로, h: 세로, 깊이 카메라로 캡처되는 이미지의 해상도를 의미

# Processing blocks
pc = rs.pointcloud()    #point cloud:3D 공간에서 각 점이 XYZ 좌표로 표현된 데이터, 이 줄은 객체는 깊이 데이터를 XYZ죄표로 변환하며 RGB데이터를 포함하면 색상이 입혀진 3D 포인트 클라우드를 생성할 수 있음
decimate = rs.decimation_filter()   #깊이 이미지의 축소 필터, 깊이 이미지의 해상도를 줄여 데이터의 크기를 감소 시키고 처리 속도를 높일 수 있음
decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)    #2 ** state.decimate는 디시메이션 필터의 축소 배율을 계산하는 부분, state.decimate가 1이면 2 ** 1 = 2, 즉 해상도가 2배 감소
colorizer = rs.colorizer()      #rs.colorizer()는 컬러라이저(Colorizer) 객체를 초기화


def mouse_cb(event, x, y, flags, param):    #mouse_cb 함수는 마우스 이벤트를 처리하여 카메라의 이동, 회전 및 줌 기능을 구현

    if event == cv2.EVENT_LBUTTONDOWN:
        state.mouse_btns[0] = True

    if event == cv2.EVENT_LBUTTONUP:
        state.mouse_btns[0] = False

    if event == cv2.EVENT_RBUTTONDOWN:
        state.mouse_btns[1] = True

    if event == cv2.EVENT_RBUTTONUP:
        state.mouse_btns[1] = False

    if event == cv2.EVENT_MBUTTONDOWN:
        state.mouse_btns[2] = True

    if event == cv2.EVENT_MBUTTONUP:
        state.mouse_btns[2] = False

    if event == cv2.EVENT_MOUSEMOVE:

        h, w = out.shape[:2]
        dx, dy = x - state.prev_mouse[0], y - state.prev_mouse[1]

        if state.mouse_btns[0]:
            state.yaw += float(dx) / w * 2
            state.pitch -= float(dy) / h * 2

        elif state.mouse_btns[1]:
            dp = np.array((dx / w, dy / h, 0), dtype=np.float32)
            state.translation -= np.dot(state.rotation, dp)

        elif state.mouse_btns[2]:
            dz = math.sqrt(dx**2 + dy**2) * math.copysign(0.01, -dy)
            state.translation[2] += dz
            state.distance -= dz

    if event == cv2.EVENT_MOUSEWHEEL:
        dz = math.copysign(0.1, flags)
        state.translation[2] += dz
        state.distance -= dz

    state.prev_mouse = (x, y)


#OpenCV 창을 생성하고, 창크기를 설정한 후 마우스 이벤트를 처리하는 콜백을 등록
cv2.namedWindow(state.WIN_NAME, cv2.WINDOW_AUTOSIZE)
cv2.resizeWindow(state.WIN_NAME, w, h)
cv2.setMouseCallback(state.WIN_NAME, mouse_cb)


def project(v):
    """project 3d vector array to 2d"""
    h, w = out.shape[:2]
    view_aspect = float(h)/w

    # ignore divide by zero for invalid depth
    with np.errstate(divide='ignore', invalid='ignore'):
        proj = v[:, :-1] / v[:, -1, np.newaxis] * \
            (w*view_aspect, h) + (w/2.0, h/2.0)

    # near clipping
    znear = 0.03
    proj[v[:, 2] < znear] = np.nan
    return proj


def view(v):
    """apply view transformation on vector array"""
    return np.dot(v - state.pivot, state.rotation) + state.pivot - state.translation


def line3d(out, pt1, pt2, color=(0x80, 0x80, 0x80), thickness=1):
    """draw a 3d line from pt1 to pt2"""
    p0 = project(pt1.reshape(-1, 3))[0]
    p1 = project(pt2.reshape(-1, 3))[0]
    if np.isnan(p0).any() or np.isnan(p1).any():
        return
    p0 = tuple(p0.astype(int))
    p1 = tuple(p1.astype(int))
    rect = (0, 0, out.shape[1], out.shape[0])
    inside, p0, p1 = cv2.clipLine(rect, p0, p1)
    if inside:
        cv2.line(out, p0, p1, color, thickness, cv2.LINE_AA)


def grid(out, pos, rotation=np.eye(3), size=1, n=10, color=(0x80, 0x80, 0x80)):
    """draw a grid on xz plane"""
    pos = np.array(pos)
    s = size / float(n)
    s2 = 0.5 * size
    for i in range(0, n+1):
        x = -s2 + i*s
        line3d(out, view(pos + np.dot((x, 0, -s2), rotation)),
               view(pos + np.dot((x, 0, s2), rotation)), color)
    for i in range(0, n+1):
        z = -s2 + i*s
        line3d(out, view(pos + np.dot((-s2, 0, z), rotation)),
               view(pos + np.dot((s2, 0, z), rotation)), color)


def axes(out, pos, rotation=np.eye(3), size=0.075, thickness=2):
    """draw 3d axes"""
    line3d(out, pos, pos +
           np.dot((0, 0, size), rotation), (0xff, 0, 0), thickness)
    line3d(out, pos, pos +
           np.dot((0, size, 0), rotation), (0, 0xff, 0), thickness)
    line3d(out, pos, pos +
           np.dot((size, 0, 0), rotation), (0, 0, 0xff), thickness)


def frustum(out, intrinsics, color=(0x40, 0x40, 0x40)):
    """draw camera's frustum"""
    orig = view([0, 0, 0])
    w, h = intrinsics.width, intrinsics.height

    for d in range(1, 6, 2):
        def get_point(x, y):
            p = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], d)
            line3d(out, orig, view(p), color)
            return p

        top_left = get_point(0, 0)
        top_right = get_point(w, 0)
        bottom_right = get_point(w, h)
        bottom_left = get_point(0, h)

        line3d(out, view(top_left), view(top_right), color)
        line3d(out, view(top_right), view(bottom_right), color)
        line3d(out, view(bottom_right), view(bottom_left), color)
        line3d(out, view(bottom_left), view(top_left), color)


def pointcloud(out, verts, texcoords, color, painter=True):
    """draw point cloud with optional painter's algorithm"""
    if painter:
        # Painter's algo, sort points from back to front

        # get reverse sorted indices by z (in view-space)
        # https://gist.github.com/stevenvo/e3dad127598842459b68
        v = view(verts)
        s = v[:, 2].argsort()[::-1]
        proj = project(v[s])
    else:
        proj = project(view(verts))

    if state.scale:
        proj *= 0.5**state.decimate

    h, w = out.shape[:2]

    # proj now contains 2d image coordinates
    j, i = proj.astype(np.uint32).T

    # create a mask to ignore out-of-bound indices
    im = (i >= 0) & (i < h)
    jm = (j >= 0) & (j < w)
    m = im & jm

    cw, ch = color.shape[:2][::-1]
    if painter:
        # sort texcoord with same indices as above
        # texcoords are [0..1] and relative to top-left pixel corner,
        # multiply by size and add 0.5 to center
        v, u = (texcoords[s] * (cw, ch) + 0.5).astype(np.uint32).T
    else:
        v, u = (texcoords * (cw, ch) + 0.5).astype(np.uint32).T
    # clip texcoords to image
    np.clip(u, 0, ch-1, out=u)
    np.clip(v, 0, cw-1, out=v)

    # perform uv-mapping
    out[i[m], j[m]] = color[u[m], v[m]]


out = np.empty((h, w, 3), dtype=np.uint8)

while True:
    # Grab camera data-> 여기 부분이 많이 다름
    
    if not state.paused:

        #####################
        frames = pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        # if not depth_frame or not color_frame:
        #     continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow('RealSense', images)
        ###########################


        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_frame = decimate.process(depth_frame)

        # Grab new intrinsics (may be changed by decimation)
        depth_intrinsics = rs.video_stream_profile(
            depth_frame.profile).get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_colormap = np.asanyarray(
            colorizer.colorize(depth_frame).get_data())

        if state.color:
            mapped_frame, color_source = color_frame, color_image
        else:
            mapped_frame, color_source = depth_frame, depth_colormap

        points = pc.calculate(depth_frame)
        pc.map_to(mapped_frame)

        # Pointcloud data to arrays
        v, t = points.get_vertices(), points.get_texture_coordinates()
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
        texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

    # Render
    now = time.time()

    out.fill(0)

    grid(out, (0, 0.5, 1), size=1, n=10)
    frustum(out, depth_intrinsics)
    axes(out, view([0, 0, 0]), state.rotation, size=0.1, thickness=1)

    if not state.scale or out.shape[:2] == (h, w):
        pointcloud(out, verts, texcoords, color_source)
    else:
        tmp = np.zeros((h, w, 3), dtype=np.uint8)
        pointcloud(tmp, verts, texcoords, color_source)
        tmp = cv2.resize(
            tmp, out.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)
        np.putmask(out, tmp > 0, tmp)

    if any(state.mouse_btns):
        axes(out, view(state.pivot), state.rotation, thickness=4)

    dt = time.time() - now

    cv2.setWindowTitle(
        state.WIN_NAME, "RealSense (%dx%d) %dFPS (%.2fms) %s" %
        (w, h, 1.0/dt, dt*1000, "PAUSED" if state.paused else ""))

    cv2.imshow(state.WIN_NAME, out) #내가 추가한거-> 이거 하니까 됨 근데 포인트 클라우드랑 뭐가 다른겨
    key = cv2.waitKey(1)

    #####
    if key == ord("w"):
        cv2.imshow(state.WIN_NAME, out)
    #####
    if key == ord("e"):
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)


    if key == ord("r"):
        state.reset()

    if key == ord("p"):
        state.paused ^= True

    if key == ord("d"):
        state.decimate = (state.decimate + 1) % 3
        decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)

    if key == ord("z"):
        state.scale ^= True

    if key == ord("c"):
        state.color ^= True

    if key == ord("s"):
        cv2.imwrite('./out.png', out)

    if key == ord("e"):
        points.export_to_ply('./out.ply', mapped_frame)

    if key in (27, ord("q")) or cv2.getWindowProperty(state.WIN_NAME, cv2.WND_PROP_AUTOSIZE) < 0:
        break

# Stop streaming
pipeline.stop()