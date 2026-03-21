import cv2
import depthai as dai
import numpy as np

def main():
    pipeline = dai.Pipeline()

    # Cameras
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)

    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_left = pipeline.create(dai.node.XLinkOut)
    xout_right = pipeline.create(dai.node.XLinkOut)
    xout_depth = pipeline.create(dai.node.XLinkOut)

    xout_rgb.setStreamName("rgb")
    xout_left.setStreamName("left")
    xout_right.setStreamName("right")
    xout_depth.setStreamName("depth")

    # Configure RGB
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setPreviewSize(640, 480)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    # Configure mono
    mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    # Configure stereo depth
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(False)

    # Link nodes
    cam_rgb.preview.link(xout_rgb.input)
    mono_left.out.link(xout_left.input)
    mono_right.out.link(xout_right.input)

    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)
    stereo.depth.link(xout_depth.input)

    print("Looking for OAK device...")

    with dai.Device(pipeline) as device:
        print("Connected to:", device.getDeviceName())
        print("MX ID:", device.getMxId())

        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        q_left = device.getOutputQueue(name="left", maxSize=4, blocking=False)
        q_right = device.getOutputQueue(name="right", maxSize=4, blocking=False)
        q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        print("Press q to quit.")

        while True:
            in_rgb = q_rgb.get()
            in_left = q_left.get()
            in_right = q_right.get()
            in_depth = q_depth.get()

            frame_rgb = in_rgb.getCvFrame()
            frame_left = in_left.getCvFrame()
            frame_right = in_right.getCvFrame()
            frame_depth = in_depth.getFrame()  # depth in mm

            # Normalize depth for display
            depth_vis = cv2.normalize(frame_depth, None, 0, 255, cv2.NORM_MINMAX)
            depth_vis = np.uint8(depth_vis)
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

            cv2.imshow("RGB", frame_rgb)
            cv2.imshow("Left Mono", frame_left)
            cv2.imshow("Right Mono", frame_right)
            cv2.imshow("Depth", depth_vis)

            key = cv2.waitKey(1)
            if key == ord("q"):
                break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()