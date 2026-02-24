import sys
import time
import ctypes
import numpy as np
import cv2
from ctypes import *

# ── Импорт классов и функций Hikvision SDK ───────────────────────────────
try:
    from MvCameraControl_class import *
except ImportError as e:
    print("❌ Ошибка импорта MvCameraControl_class:", e)
    print("   Убедитесь, что файл MvCameraControl_class.py находится в текущей папке.")
    print("   Обычно он лежит в: MVS\\Samples\\Python\\MvCameraControl_class.py")
    sys.exit(1)

# ── Импорт констант пиксельных форматов ──────────────────────────────────
try:
    from PixelType_header import *
except ImportError as e:
    print("❌ Ошибка импорта PixelType_header:", e)
    print("   Скопируйте PixelType_header.py в текущую папку из MVS\\Samples\\Python")
    sys.exit(1)


# Константы отображения
TARGET_WIDTH = 800
TARGET_HEIGHT = 600
DISPLAY_FPS = True
FPS_UPDATE_INTERVAL = 1.0  # через сколько секунд пересчитывать FPS


def find_usb_camera(deviceList):
    """Поиск первой USB-камеры в списке устройств"""
    print("🔍 Поиск USB-камеры...")
    for i in range(deviceList.nDeviceNum):
        dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        if dev_info.nTLayerType == MV_USB_DEVICE:
            serial = (
                bytes(dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber)
                .split(b"\x00")[0]
                .decode("utf-8", errors="ignore")
            )
            user_name = (
                bytes(dev_info.SpecialInfo.stUsb3VInfo.chUserDefinedName)
                .split(b"\x00")[0]
                .decode("utf-8", errors="ignore")
            )
            print(f"   ✅ Найдена: Serial [{serial}] | Name [{user_name}] (индекс {i})")
            return i
    print("❌ USB-камера не найдена!")
    return -1


def configure_camera(cam):
    """Настройка основных параметров камеры"""
    print("⚙️ Настройка камеры...")

    # Отключаем внешний триггер → свободный (continuous) режим
    ret = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
    print(f"   TriggerMode → OFF  {'✅' if ret == 0 else f'❌ [0x{ret:x}]'}")

    # Пробуем цветной Bayer RG8 → если не получилось, переключаемся на монохром
    ret = cam.MV_CC_SetEnumValue("PixelFormat", PixelType_Gvsp_BayerRG8)
    if ret == 0:
        print("   PixelFormat → BayerRG8  ✅")
    else:
        print(f"   BayerRG8 не поддерживается [0x{ret:x}] → пробуем Mono8")
        ret = cam.MV_CC_SetEnumValue("PixelFormat", PixelType_Gvsp_Mono8)
        print(f"   PixelFormat → Mono8  {'✅' if ret == 0 else f'❌ [0x{ret:x}]'}")

    # Фиксированное разрешение (можно потом вынести в параметры)
    cam.MV_CC_SetIntValue("Width", 3072)
    cam.MV_CC_SetIntValue("Height", 2048)
    print("   Разрешение → 3072 × 2048  ✅")

    # Отключаем авто-баланс белого
    cam.MV_CC_SetEnumValue("BalanceWhiteAuto", 0)  # 0 = Off
    # Или поставь Once для одноразовой подстройки:
    # cam.MV_CC_SetEnumValue("BalanceWhiteAuto", 1)  # 1 = Once

    # Отключаем авто-экспозицию
    cam.MV_CC_SetEnumValue("ExposureAuto", 0)  # Off
    print("   Авто-баланс белого и авто-экспозиция → отключены  ⚠️")


def process_frame(pData, stFrameInfo):
    """
    Преобразование сырого буфера в BGR-изображение OpenCV
    Возвращает None при ошибке
    """
    n_pixels = stFrameInfo.nWidth * stFrameInfo.nHeight
    img_buff = np.frombuffer(pData, dtype=np.uint8)

    if len(img_buff) < n_pixels:
        print("⚠️  Буфер меньше ожидаемого размера!")
        return None

    # Берём только нужное количество байт и формируем 2D-массив
    raw_img = img_buff[:n_pixels].reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))

    # Конвертация в зависимости от текущего формата пикселей
    if stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG8:
        # return cv2.cvtColor(raw_img, cv2.COLOR_BayerRG2BGR)
        return cv2.cvtColor(raw_img, cv2.COLOR_BayerBG2BGR)
    elif stFrameInfo.enPixelType == PixelType_Gvsp_Mono8:
        return cv2.cvtColor(raw_img, cv2.COLOR_GRAY2BGR)
    else:
        print(f"⚠️  Неизвестный формат пикселей: {stFrameInfo.enPixelType}")
        return None


def calculate_fps(frame_count, start_time, interval=FPS_UPDATE_INTERVAL):
    """
    Вычисляет актуальный FPS и обновляет счётчики
    Возвращает (новый_fps, новый_frame_count, новое_start_time)
    """
    current_time = time.time()
    elapsed = current_time - start_time

    if elapsed >= interval:
        fps = frame_count / elapsed
        return fps, 0, current_time
    else:
        return None, frame_count, start_time


def display_resized_with_fps(img, fps):
    """Масштабирование с сохранением пропорций + центрирование + наложение FPS"""
    h, w = img.shape[:2]
    scale = min(TARGET_WIDTH / w, TARGET_HEIGHT / h)
    new_w, new_h = int(w * scale), int(h * scale)
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)

    # Чёрный фон целевого размера
    display_img = np.zeros((TARGET_HEIGHT, TARGET_WIDTH, 3), dtype=np.uint8)
    x_offset = (TARGET_WIDTH - new_w) // 2
    y_offset = (TARGET_HEIGHT - new_h) // 2
    display_img[y_offset : y_offset + new_h, x_offset : x_offset + new_w] = resized

    if DISPLAY_FPS and fps is not None:
        cv2.putText(
            display_img,
            f"FPS: {fps:.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 100),  # чуть ярче-зелёный
            2,
            cv2.LINE_AA,
        )

    return display_img


def main():
    print("🚀 Тест Hik USB камеры (MV-CS060-10UC-PRO) запущен!\n")

    print("SDK версия:", hex(MvCamera.MV_CC_GetSDKVersion()))

    # ── 1. Поиск устройств ───────────────────────────────────────
    deviceList = MV_CC_DEVICE_INFO_LIST()
    ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, deviceList)
    if ret != 0 or deviceList.nDeviceNum == 0:
        print(f"❌ Ошибка перечисления устройств или устройств нет → [0x{ret:x}]")
        sys.exit()

    print(f"Найдено устройств: {deviceList.nDeviceNum} шт.")

    # ── 2. Выбор USB-камеры ──────────────────────────────────────
    usb_index = find_usb_camera(deviceList)
    if usb_index == -1:
        sys.exit()

    # ── 3. Инициализация камеры ──────────────────────────────────
    cam = MvCamera()
    stDevice = cast(deviceList.pDeviceInfo[usb_index], POINTER(MV_CC_DEVICE_INFO)).contents

    ret = cam.MV_CC_CreateHandle(stDevice)
    if ret != 0:
        print(f"❌ Ошибка создания хэндла → [0x{ret:x}]")
        sys.exit()

    ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    if ret != 0:
        print(f"❌ Ошибка открытия устройства → [0x{ret:x}]")
        sys.exit()

    print("🎉 Камера успешно открыта")

    # ── 4. Настройки ─────────────────────────────────────────────
    configure_camera(cam)

    # ── 5. Запуск захвата ────────────────────────────────────────
    ret = cam.MV_CC_StartGrabbing()
    if ret != 0:
        print(f"❌ Ошибка запуска захвата → [0x{ret:x}]")
        sys.exit()

    print("📹 Захват кадров запущен. Нажмите 'q' для выхода\n")

    # Подготовка буфера и информации о кадре
    stFrameInfo = MV_FRAME_OUT_INFO_EX()
    memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
    max_buffer_size = 3072 * 2048 * 3
    pData = (c_ubyte * max_buffer_size)()

    # Переменные для FPS
    frame_count = 0
    start_time = time.time()
    current_fps = 0.0

    try:
        while True:
            ret = cam.MV_CC_GetOneFrameTimeout(pData, max_buffer_size, stFrameInfo, 1000)
            if ret == 0:
                frame_count += 1

                img = process_frame(pData, stFrameInfo)
                if img is None:
                    continue

                # Расчёт FPS (обновляется не каждый кадр)
                new_fps, frame_count, start_time = calculate_fps(frame_count, start_time)
                if new_fps is not None:
                    current_fps = new_fps

                # Подготовка изображения для показа
                display_img = display_resized_with_fps(img, current_fps)
                cv2.imshow("Hik USB Camera", display_img)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("\n👋 Выход по клавише 'q'")
                    break
            else:
                print(f"⏳ Таймаут/ошибка кадра [0x{ret:x}]")

    except KeyboardInterrupt:
        print("\n🛑 Прервано Ctrl+C")

    finally:
        print("\n🧹 Остановка и очистка...")
        cam.MV_CC_StopGrabbing()
        cam.MV_CC_CloseDevice()
        cam.MV_CC_DestroyHandle()
        cv2.destroyAllWindows()
        print("Готово! 👌")


if __name__ == "__main__":
    main()
