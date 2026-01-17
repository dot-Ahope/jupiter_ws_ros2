#!/usr/bin/env python3
# encoding: utf-8
import cv2 as cv
import time
import numpy as np

# Astra 카메라 초기화
capture = cv.VideoCapture(0)  # RGB 카메라
capture.set(6, cv.VideoWriter.fourcc('M','J','P','G'))
capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

# 뎁스 카메라 초기화 (Astra는 일반적으로 1번 인덱스가 뎁스)
depth_capture = cv.VideoCapture(1)
depth_capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
depth_capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

print("RGB Camera FPS:", capture.get(cv.CAP_PROP_FPS))
print("Depth Camera FPS:", depth_capture.get(cv.CAP_PROP_FPS))

def process_depth_image(depth_frame):
    # 뎁스 이미지를 시각화하기 위한 정규화
    # Astra의 뎁스 범위는 일반적으로 0-4096
    depth_frame_norm = cv.normalize(depth_frame, None, 0, 255, cv.NORM_MINMAX, dtype=cv.CV_8U)
    depth_colormap = cv.applyColorMap(depth_frame_norm, cv.COLORMAP_JET)
    return depth_colormap

while capture.isOpened() and depth_capture.isOpened():
    start = time.time()
    
    # RGB 프레임 읽기
    ret, frame = capture.read()
    if not ret:
        break
        
    # 뎁스 프레임 읽기
    ret_depth, depth_frame = depth_capture.read()
    if not ret_depth:
        break
        
    # 뎁스 이미지 처리
    if depth_frame is not None:
        depth_colormap = process_depth_image(depth_frame)
        
        # 중앙 픽셀의 뎁스 값 표시
        center_y, center_x = depth_frame.shape[0] // 2, depth_frame.shape[1] // 2
        center_depth = depth_frame[center_y, center_x]
        cv.circle(depth_colormap, (center_x, center_y), 4, (0, 0, 255), -1)
        cv.putText(depth_colormap, f"Depth: {center_depth}mm", 
                  (20, 60), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
    
    end = time.time()
    fps = 1 / (end - start)
    
    # FPS 표시
    text = f"FPS: {int(fps)}"
    cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
    
    # 영상 표시
    cv.imshow('RGB', frame)
    if depth_frame is not None:
        cv.imshow('Depth', depth_colormap)
    
    # q 키를 누르면 종료
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# 리소스 해제
capture.release()
depth_capture.release()
cv.destroyAllWindows()
