#!/usr/bin/env python3

import cv2
import depthai as dai
import contextlib
import os
from pathlib import Path
import datetime
from vidgear.gears import WriteGear

width_main = 1280
height_main = 720
width_side = 750
height_side = 450
# Replace line below with choice of these: 18443010D1B09B0F00,18443010E1BD9B0F00,  1844301041A78E0E00
main_cam = "18443010F1A1A20F00"


def createPipeline(main=True):
    # Start defining a pipeline
    pipeline = dai.Pipeline()
    # Define a source - color camera
    camRgb = pipeline.create(dai.node.ColorCamera)

    if main:
        camRgb.setPreviewSize(width_main,height_main)
    else:
        camRgb.setPreviewSize(width_side,height_side)
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)

    # Create output
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutRgb.setStreamName("rgb")
    camRgb.preview.link(xoutRgb.input)

    return pipeline


with contextlib.ExitStack() as stack:
    deviceInfos = dai.Device.getAllAvailableDevices()
    usbSpeed = dai.UsbSpeed.SUPER
    openVinoVersion = dai.OpenVINO.Version.VERSION_2021_4

    qRgbMap = []
    devices = []
    dir_name = "/videos"
    Path(dir_name).mkdir(parents=True, exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    file_format = "Day_%Y_%m_%d-Time_%H_%M_%S"
    start_stamp = datetime.datetime.now()
    file_name = start_stamp.strftime(file_format)
    vMap = {}
    # output_params = {"-r":10}

    for deviceInfo in deviceInfos:
        deviceInfo: dai.DeviceInfo
        device: dai.Device = stack.enter_context(dai.Device(openVinoVersion, deviceInfo, usbSpeed))
        devices.append(device)
        print("===Connected to ", deviceInfo.getMxId())
        mxId = device.getMxId()
        eepromData = device.readCalibration2().getEepromData()  # 18443010D1B09B0F00,18443010E1BD9B0F00,  1844301041A78E0E00
        if mxId != main_cam:
            pipeline = createPipeline(False)
        else:
            pipeline = createPipeline()
        device.startPipeline(pipeline)
        
        # Output queue will be used to get the rgb frames from the output defined above
        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        stream_name = f"rgb-{mxId}-{eepromData.productName}"
        Path(os.path.join(dir_name,stream_name)).mkdir(parents=True, exist_ok=True)
        qRgbMap.append((q_rgb, stream_name))
        if stream_name !=  f"rgb-{main_cam}-{eepromData.productName}":
            pass
        else:
            out = cv2.VideoWriter(os.path.join(dir_name,stream_name,f"Main_Feed_{file_name}.mp4"), fourcc, 24.0, (width_main,  height_main))
            vMap[stream_name] = out

    
    

    while True:
        t_stamp = datetime.datetime.now()
        for q_rgb, stream_name in qRgbMap:
            if q_rgb.has():
                frame = q_rgb.get().getCvFrame()
                # cv2.imshow(stream_name,frame) # Show frame in viewer
                if stream_name in vMap:
                    vMap[stream_name].write(frame) # Save frame

        if t_stamp - start_stamp >= datetime.timedelta(minutes=10):
            start_stamp = datetime.datetime.now()
            file_name = start_stamp.strftime(file_format)
            print(file_name)
            for i in vMap.keys():
                vMap[i].release()       
                if i !=  f"rgb-{main_cam}-{eepromData.productName}":
                    pass
                else:
                    vMap[i] = cv2.VideoWriter(os.path.join(dir_name,i,f"Main_Feed_{file_name}.mp4"), fourcc, 15.0, (width_main,  height_main))

        if cv2.waitKey(1) == ord('q'):
            for i in vMap.keys():
                vMap[i].release()
            cv2.destroyAllWindows()
            break
