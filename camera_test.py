import cv2

for idx, name in [(0, "camera_A"), (2, "camera_B")]:
    cap = cv2.VideoCapture(idx)
    ret, frame = cap.read()
    if ret:
        path = f"{name}_video{idx}.jpg"
        cv2.imwrite(path, frame)
        print(f"{name} (/dev/video{idx}): 拍照成功 -> {path}")
    else:
        print(f"{name} (/dev/video{idx}): 拍照失败")
    cap.release()   