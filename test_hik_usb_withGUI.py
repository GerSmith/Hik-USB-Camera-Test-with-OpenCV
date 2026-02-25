import sys
import time
import ctypes
import numpy as np
import cv2
from ctypes import *

from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QLabel,
    QVBoxLayout,
    QWidget,
    QPushButton,
    QHBoxLayout,
    QMessageBox,
)
from PySide6.QtCore import QThread, Signal, Qt
from PySide6.QtGui import QImage, QPixmap

# Импорт Hik SDK
from MvCameraControl_class import *

# Импорт констант пиксельных форматов
try:
    from PixelType_header import *  # PixelType_Gvsp_*
except ImportError as e:
    print("Error importing PixelType_header:", e)
    print("Copy PixelType_header.py to current folder from MVS\\Samples\\Python")
    sys.exit(1)


class VideoThread(QThread):
    """
    Поток для захвата кадров с Hik камеры без блокировки GUI
    """

    change_pixmap_signal = Signal(np.ndarray)  # сигнал с кадром (numpy BGR)
    error_signal = Signal(str)  # сигнал ошибки

    def __init__(self):
        super().__init__()
        self._run_flag = True
        self.cam = None
        self.pData = None
        self.stFrameInfo = MV_FRAME_OUT_INFO_EX()
        memset(byref(self.stFrameInfo), 0, sizeof(self.stFrameInfo))

        # Для FPS
        self.frame_count = 0
        self.start_time = time.time()
        self.current_fps = 0.0

    def run(self):
        # Инициализация камеры
        deviceList = MV_CC_DEVICE_INFO_LIST()
        tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
        ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
        if ret != 0 or deviceList.nDeviceNum == 0:
            self.error_signal.emit(f"Enum devices fail or no devices! ret[0x{ret:x}")
            return

        usb_device_index = -1
        for i in range(deviceList.nDeviceNum):
            mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            if mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
                serial = (
                    bytes(mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber)
                    .split(b"\x00")[0]
                    .decode("utf-8", errors="ignore")
                )
                user_name = (
                    bytes(mvcc_dev_info.SpecialInfo.stUsb3VInfo.chUserDefinedName)
                    .split(b"\x00")[0]
                    .decode("utf-8", errors="ignore")
                )
                print(f"Found USB camera: Serial [{serial}], UserDefinedName [{user_name}]")
                usb_device_index = i
                break

        if usb_device_index == -1:
            self.error_signal.emit("No USB camera found!")
            return

        self.cam = MvCamera()
        stDeviceList = cast(
            deviceList.pDeviceInfo[usb_device_index], POINTER(MV_CC_DEVICE_INFO)
        ).contents

        ret = self.cam.MV_CC_CreateHandle(stDeviceList)
        if ret != 0:
            self.error_signal.emit(f"Create handle fail! ret[0x{ret:x}")
            return

        ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            self.error_signal.emit(f"Open device fail! ret[0x{ret:x}")
            return

        # Настройки
        self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)

        ret = self.cam.MV_CC_SetEnumValue("PixelFormat", PixelType_Gvsp_BayerRG8)
        if ret != 0:
            print(f"Set PixelFormat BayerRG8 fail! ret[0x{ret:x}")
            self.cam.MV_CC_SetEnumValue("PixelFormat", PixelType_Gvsp_Mono8)
            print("Fallback to Mono8")

        self.cam.MV_CC_SetIntValue("Width", 3072)
        self.cam.MV_CC_SetIntValue("Height", 2048)

        ret = self.cam.MV_CC_StartGrabbing()
        if ret != 0:
            self.error_signal.emit(f"Start grabbing fail! ret[0x{ret:x}")
            return

        # Буфер
        max_buffer_size = 3072 * 2048 * 3
        self.pData = (c_ubyte * max_buffer_size)()

        print("Grabbing started in thread...")

        while self._run_flag:
            ret = self.cam.MV_CC_GetOneFrameTimeout(
                self.pData, max_buffer_size, self.stFrameInfo, 1000
            )
            if ret == 0:
                self.frame_count += 1

                img_buff = np.frombuffer(self.pData, dtype=np.uint8)
                try:
                    raw_img = img_buff[: self.stFrameInfo.nWidth * self.stFrameInfo.nHeight].reshape(
                        (self.stFrameInfo.nHeight, self.stFrameInfo.nWidth)
                    )

                    if self.stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG8:
                        img = cv2.cvtColor(raw_img, cv2.COLOR_BayerBG2BGR)
                    elif self.stFrameInfo.enPixelType == PixelType_Gvsp_Mono8:
                        img = cv2.cvtColor(raw_img, cv2.COLOR_GRAY2BGR)
                    else:
                        img = raw_img.reshape((self.stFrameInfo.nHeight, self.stFrameInfo.nWidth, 3))

                    # Расчёт и отрисовка FPS на кадре
                    elapsed = time.time() - self.start_time
                    if elapsed >= 1.0:
                        self.current_fps = self.frame_count / elapsed
                        self.frame_count = 0
                        self.start_time = time.time()

                    fps_text = f"FPS: {self.current_fps:.1f}"
                    cv2.putText(
                        img,
                        fps_text,
                        (10, 40),  # Левый верхний угол (x=10, y=40)
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.2,  # Размер шрифта
                        (0, 255, 0),  # Зелёный цвет
                        3,  # Толщина
                        cv2.LINE_AA,
                    )

                    # Отправляем кадр с FPS в GUI
                    self.change_pixmap_signal.emit(img)

                except Exception as e:
                    print("Image processing error:", e)

            else:
                print(f"Get frame timeout or error [0x{ret:x}]")

        # Cleanup в потоке
        if self.cam:
            self.cam.MV_CC_StopGrabbing()
            self.cam.MV_CC_CloseDevice()
            self.cam.MV_CC_DestroyHandle()
        print("Grabbing stopped")

    def stop(self):
        self._run_flag = False
        self.wait()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hik USB Camera Test with GUI")
        self.resize(1000, 800)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Лейбл для видео
        self.image_label = QLabel()
        self.image_label.setScaledContents(True)
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(800, 600)
        layout.addWidget(self.image_label)

        # Панель кнопок
        btn_layout = QHBoxLayout()
        self.btn_start_stop = QPushButton("Старт захвата")
        self.btn_start_stop.clicked.connect(self.toggle_capture)
        btn_layout.addWidget(self.btn_start_stop)

        btn_exit = QPushButton("Выход")
        btn_exit.clicked.connect(self.close)
        btn_layout.addWidget(btn_exit)

        layout.addLayout(btn_layout)

        # Поток
        self.thread = None
        self.is_capturing = False

    def toggle_capture(self):
        if not self.is_capturing:
            self.thread = VideoThread()
            self.thread.change_pixmap_signal.connect(self.update_image)
            self.thread.error_signal.connect(self.show_error)
            self.thread.start()
            self.btn_start_stop.setText("Стоп захвата")
            self.is_capturing = True
        else:
            if self.thread:
                self.thread.stop()
            self.btn_start_stop.setText("Старт захвата")
            self.is_capturing = False
            self.image_label.clear()

    def update_image(self, cv_img):
        """OpenCV BGR → QImage RGB → QPixmap"""
        h, w, ch = cv_img.shape
        bytes_per_line = ch * w
        qt_img = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
        qt_img = qt_img.rgbSwapped()  # BGR → RGB
        pixmap = QPixmap.fromImage(qt_img)
        self.image_label.setPixmap(
            pixmap.scaled(self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        )

    def show_error(self, msg):
        QMessageBox.critical(self, "Ошибка", msg)
        if self.is_capturing:
            self.toggle_capture()  # Авто-стоп при ошибке

    def closeEvent(self, event):
        if self.thread and self.thread.isRunning():
            self.thread.stop()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
