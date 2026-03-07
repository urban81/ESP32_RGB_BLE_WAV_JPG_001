"""
BLE Image Sender для ESP32‑S3 RGB Display 400×960 (портрет)
Отправляет JPEG (сжатый) вместо сырых RGB565 для ускорения передачи.
Полная версия с GUI на Kivy.
Исправлено: принудительное завершение процесса при закрытии окна.
"""

import asyncio
import os
import struct
import math
import threading
import sys
import time
from pathlib import Path
from typing import List, Optional

# Kivy для GUI
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.image import Image as KivyImage
from kivy.uix.filechooser import FileChooserListView
from kivy.uix.popup import Popup
from kivy.uix.scrollview import ScrollView
from kivy.uix.gridlayout import GridLayout
from kivy.core.window import Window
from kivy.clock import Clock
from kivy.properties import StringProperty, NumericProperty, DictProperty

# Обработка изображений
from PIL import Image
import io

# BLE
from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

# ========== НАСТРОЙКИ ДИСПЛЕЯ ==========
TFT_WIDTH = 400
TFT_HEIGHT = 960
TARGET_SIZE = (TFT_WIDTH, TFT_HEIGHT)

# Параметры JPEG
JPEG_QUALITY = 85  # можно менять (80–95)

PACKET_SIZE = 200
HEADER_SIZE = 5
PACKET_TYPE_IMAGE_START = 0x01
PACKET_TYPE_IMAGE_DATA = 0x02
PACKET_TYPE_IMAGE_END = 0x03

# UUID сервиса и характеристики (little-endian, как передаёт ESP32)
SERVICE_UUID = "4b9131c3-c9c5-cc8f-9e45-b51f01c2af4f"
CHAR_TX_UUID = "a8261b36-07ea-f5b7-8846-e1363e48b5be"

# ========== ОБРАБОТКА ИЗОБРАЖЕНИЯ ==========
def process_image_to_jpeg(image_path: str) -> bytes:
    """Загружает, ресайзит до 400×960 и возвращает JPEG в памяти."""
    print(f"📷 Обработка: {Path(image_path).name}")
    img = Image.open(image_path).convert('RGB')
    img = img.resize(TARGET_SIZE, Image.Resampling.LANCZOS)
    print(f"  📐 Размер после ресайза: {img.width} x {img.height}")

    output = io.BytesIO()
    img.save(output, format='JPEG', quality=JPEG_QUALITY, optimize=True, progressive=False)
    jpeg_bytes = output.getvalue()
    print(f"  📊 Размер JPEG: {len(jpeg_bytes)} байт")
    return jpeg_bytes

def create_image_packets(image_data: bytes) -> List[bytes]:
    """Разбивает данные на пакеты с заголовками."""
    packets = []
    MAX_DATA = PACKET_SIZE - HEADER_SIZE  # 195 байт
    total_packets = math.ceil(len(image_data) / MAX_DATA)

    # START пакет
    start = struct.pack('<BHH', PACKET_TYPE_IMAGE_START, 0, total_packets)
    packets.append(start)

    # DATA пакеты
    for i in range(total_packets):
        start_idx = i * MAX_DATA
        end_idx = min((i+1) * MAX_DATA, len(image_data))
        chunk = image_data[start_idx:end_idx]
        packet = struct.pack('<BHH', PACKET_TYPE_IMAGE_DATA, i+1, total_packets) + chunk
        packets.append(packet)

    # END пакет
    end = struct.pack('<BHH', PACKET_TYPE_IMAGE_END, total_packets+1, total_packets)
    packets.append(end)

    print(f"📦 Создано пакетов: {len(packets)}")
    return packets

# ========== МЕНЕДЖЕР ASYNCIO ==========
class AsyncManager:
    def __init__(self):
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.thread: Optional[threading.Thread] = None

    def start_loop(self):
        if self.loop is None or self.loop.is_closed():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.thread = threading.Thread(target=self._run_loop, daemon=True)
            self.thread.start()

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_forever()
        finally:
            self.loop.close()
            self.loop = None

    def run_coroutine(self, coro, timeout=60):
        if self.loop is None or self.loop.is_closed():
            self.start_loop()
        future = asyncio.run_coroutine_threadsafe(coro, self.loop)
        return future.result(timeout=timeout)

    def stop(self):
        if self.loop and not self.loop.is_closed():
            for task in asyncio.all_tasks(self.loop):
                task.cancel()
            self.loop.call_soon_threadsafe(self.loop.stop)
            if self.thread:
                self.thread.join(timeout=5)

async_manager = AsyncManager()

# ========== GUI НА KIVY ==========
class DeviceButton(Button):
    device_info = DictProperty({})

class ProgressPopup(Popup):
    progress = NumericProperty(0)
    message = StringProperty("")

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.title = "Отправка изображения"
        self.size_hint = (0.8, 0.3)
        self.auto_dismiss = False

class BLEImageSenderApp(App):
    status_text = StringProperty("Готов к работе")
    connected_device = StringProperty("Не подключено")
    selected_image = StringProperty("")

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.ble_client: Optional[BleakClient] = None
        self.selected_file: Optional[str] = None
        self.selected_device: Optional[dict] = None
        self.tx_characteristic = None
        self.is_connected = False
        self.progress_popup: Optional[ProgressPopup] = None
        self.scanning = False
        self._is_closing = False

    def build(self):
        self.title = f"BLE JPEG Sender для {TFT_WIDTH}x{TFT_HEIGHT}"
        Window.size = (800, 600)
        Window.bind(on_request_close=self.on_request_close)

        main_layout = BoxLayout(orientation='vertical', padding=10, spacing=10)

        # Статусная строка
        status_layout = BoxLayout(size_hint=(1, 0.1), spacing=10)
        self.status_label = Label(text=self.status_text, font_size='16sp', halign='left', size_hint=(0.7,1))
        self.status_label.bind(size=self.status_label.setter('text_size'))
        self.size_label = Label(text=f"Дисплей: {TFT_WIDTH}x{TFT_HEIGHT}", font_size='12sp', halign='right', color=(0.8,0.6,0.2,1), size_hint=(0.3,1))
        self.device_label = Label(text=self.connected_device, font_size='14sp', halign='right', color=(0.2,0.8,0.2,1), size_hint=(0.3,1))
        status_layout.add_widget(self.status_label)
        status_layout.add_widget(self.size_label)
        status_layout.add_widget(self.device_label)

        # Центральная область
        center_layout = BoxLayout(orientation='horizontal', spacing=10, size_hint=(1,0.7))

        # Левая панель с предпросмотром
        image_panel = BoxLayout(orientation='vertical', size_hint=(0.6,1))
        image_label = Label(text=f"Предпросмотр (будет изменено до {TFT_WIDTH}x{TFT_HEIGHT}):", size_hint=(1,0.1), font_size='14sp')
        self.preview_image = KivyImage(source="", size_hint=(1,0.9), allow_stretch=True, keep_ratio=True)
        image_panel.add_widget(image_label)
        image_panel.add_widget(self.preview_image)

        # Правая панель с кнопками
        button_panel = BoxLayout(orientation='vertical', size_hint=(0.4,1), spacing=10)

        self.select_button = Button(text="📁 Выбрать изображение", font_size='14sp', size_hint=(1,0.15), background_color=(0.2,0.6,0.8,1))
        self.select_button.bind(on_press=self.select_image)

        self.scan_button = Button(text="🔍 Сканировать BLE", font_size='14sp', size_hint=(1,0.15), background_color=(0.8,0.6,0.2,1))
        self.scan_button.bind(on_press=self.start_scan)

        self.send_button = Button(text="📤 Отправить JPEG", font_size='14sp', size_hint=(1,0.15), background_color=(0.2,0.8,0.4,1), disabled=True)
        self.send_button.bind(on_press=self.send_image)

        self.test_button = Button(text="🧪 Тестовый JPEG", font_size='14sp', size_hint=(1,0.15), background_color=(0.8,0.2,0.8,1), disabled=True)
        self.test_button.bind(on_press=self.send_test_image)

        devices_label = Label(text="Найденные устройства BLE:", size_hint=(1,0.1), font_size='12sp')

        self.devices_scroll = ScrollView(size_hint=(1,0.3))
        self.devices_layout = GridLayout(cols=1, spacing=5, size_hint_y=None)
        self.devices_layout.bind(minimum_height=self.devices_layout.setter('height'))
        self.devices_scroll.add_widget(self.devices_layout)

        button_panel.add_widget(self.select_button)
        button_panel.add_widget(self.scan_button)
        button_panel.add_widget(self.send_button)
        button_panel.add_widget(self.test_button)
        button_panel.add_widget(devices_label)
        button_panel.add_widget(self.devices_scroll)

        center_layout.add_widget(image_panel)
        center_layout.add_widget(button_panel)

        # Нижняя панель информации
        info_layout = BoxLayout(orientation='vertical', size_hint=(1,0.2))
        self.info_label = Label(text=f"Выберите изображение для отправки на TFT {TFT_WIDTH}x{TFT_HEIGHT}", font_size='12sp', halign='center', valign='middle')
        self.info_label.bind(size=self.info_label.setter('text_size'))
        info_layout.add_widget(self.info_label)

        main_layout.add_widget(status_layout)
        main_layout.add_widget(center_layout)
        main_layout.add_widget(info_layout)

        return main_layout

    def select_image(self, instance):
        content = BoxLayout(orientation='vertical', spacing=10)
        filechooser = FileChooserListView(filters=['*.png', '*.jpg', '*.jpeg', '*.bmp'], size_hint=(1,0.8))
        button_layout = BoxLayout(size_hint=(1,0.2), spacing=10)
        cancel_button = Button(text="Отмена")
        select_button = Button(text="Выбрать")

        def select_file(btn):
            if filechooser.selection:
                self.selected_file = filechooser.selection[0]
                self.preview_image.source = self.selected_file
                self.selected_image = Path(self.selected_file).name
                self.info_label.text = f"Выбрано: {self.selected_image}"
                if self.is_connected:
                    self.send_button.disabled = False
                    self.test_button.disabled = False
            popup.dismiss()

        def cancel(btn):
            popup.dismiss()

        select_button.bind(on_press=select_file)
        cancel_button.bind(on_press=cancel)

        button_layout.add_widget(cancel_button)
        button_layout.add_widget(select_button)

        content.add_widget(filechooser)
        content.add_widget(button_layout)

        popup = Popup(title="Выберите изображение", content=content, size_hint=(0.9,0.9))
        popup.open()

    def start_scan(self, instance):
        if self.scanning:
            return
        self.status_text = "Сканирование BLE... (10 сек)"
        self.scan_button.disabled = True
        self.scan_button.text = "🔍 Сканирование..."
        self.devices_layout.clear_widgets()
        self.scanning = True
        threading.Thread(target=self.run_scan, daemon=True).start()

    def run_scan(self):
        try:
            devices = async_manager.run_coroutine(self.scan_ble_devices())
            Clock.schedule_once(lambda dt: self.update_devices_list(devices))
        except Exception as e:
            print(f"❌ Ошибка сканирования: {e}")
            Clock.schedule_once(lambda dt: self.show_error(f"Ошибка сканирования: {str(e)}"))
        finally:
            Clock.schedule_once(lambda dt: self.finish_scan())

    async def scan_ble_devices(self):
        print("🔍 Начало сканирования BLE...")
        devices = []
        try:
            scanner = BleakScanner()
            scanned_devices = await scanner.discover(timeout=10.0, return_adv=True)
            for device, advertisement_data in scanned_devices.values():
                name = device.name or "Без имени"
                rssi = advertisement_data.rssi if advertisement_data.rssi else -100
                devices.append({'name': name, 'address': device.address, 'rssi': rssi})
                print(f"📱 Найдено: {name} ({device.address}) RSSI: {rssi}")
        except Exception as e:
            print(f"❌ Ошибка сканирования: {e}")
        return devices

    def update_devices_list(self, devices):
        self.devices_layout.clear_widgets()
        if not devices:
            no_devices_label = Label(
                text="Устройства не найдены\n(Проверьте включен ли ESP32)",
                size_hint_y=None, height=60, color=(0.8,0.2,0.2,1)
            )
            self.devices_layout.add_widget(no_devices_label)
            return

        for device in devices:
            device_name = device['name'] if device['name'] else "Без имени"
            button_text = f"{device_name}\n{device['address']}\nRSSI: {device['rssi']}"
            device_button = Button(
                text=button_text,
                size_hint_y=None, height=80, font_size='11sp', halign='left',
                background_color=(0.3,0.3,0.3,1)
            )
            device_button.bind(on_press=lambda btn, d=device: self.select_device(d))
            self.devices_layout.add_widget(device_button)

    def select_device(self, device_info):
        self.selected_device = device_info
        self.info_label.text = f"Выбрано: {device_info['name']}"
        self.show_connect_popup(device_info)

    def show_connect_popup(self, device_info):
        content = BoxLayout(orientation='vertical', spacing=10, padding=10)
        message = Label(
            text=f"Подключиться к:\n{device_info['name']}\n{device_info['address']}?",
            size_hint=(1,0.6), font_size='14sp', halign='center'
        )
        button_layout = BoxLayout(size_hint=(1,0.4), spacing=10)
        cancel_button = Button(text="Отмена")
        connect_button = Button(text="Подключиться", background_color=(0.2,0.8,0.2,1))

        def connect(btn):
            popup.dismiss()
            threading.Thread(target=self.connect_to_device, args=(device_info['address'],), daemon=True).start()

        def cancel(btn):
            popup.dismiss()

        connect_button.bind(on_press=connect)
        cancel_button.bind(on_press=cancel)

        button_layout.add_widget(cancel_button)
        button_layout.add_widget(connect_button)

        content.add_widget(message)
        content.add_widget(button_layout)

        popup = Popup(title="Подключение к устройству", content=content, size_hint=(0.7,0.4))
        popup.open()

    def connect_to_device(self, address):
        self.status_text = f"Подключение к {address}..."
        try:
            connected = async_manager.run_coroutine(self.ble_connect(address))
            if connected:
                Clock.schedule_once(lambda dt: self.on_device_connected(address))
            else:
                Clock.schedule_once(lambda dt: self.show_error("Не удалось подключиться"))
        except Exception as e:
            print(f"❌ Ошибка подключения: {e}")
            Clock.schedule_once(lambda dt: self.show_error(f"Ошибка подключения: {str(e)}"))

    async def ble_connect(self, address):
        try:
            print(f"🔗 Подключение к {address}...")
            self.ble_client = BleakClient(address)
            await self.ble_client.connect(timeout=10.0)
            print("✅ Успешно подключено")

            for service in self.ble_client.services:
                print(f"📋 Сервис: {service.uuid}")
                for char in service.characteristics:
                    print(f"   🔹 Характеристика: {char.uuid}")
                    if char.uuid.lower() == CHAR_TX_UUID.lower():
                        self.tx_characteristic = char
                        print(f"✅ Найдена характеристика: {char.uuid}")
                        break
                if self.tx_characteristic:
                    break

            if not self.tx_characteristic:
                print("❌ Характеристика не найдена!")
                return False

            return True
        except Exception as e:
            print(f"❌ Ошибка подключения: {e}")
            return False

    def on_device_connected(self, address):
        self.status_text = "Подключено"
        self.connected_device = f"{address[:17]}..."
        self.device_label.color = (0.2,0.8,0.2,1)
        self.info_label.text = "Устройство подключено. Выберите изображение."
        self.is_connected = True
        self.send_button.disabled = False
        self.test_button.disabled = False

    def send_image(self, instance):
        if not self.selected_file:
            self.show_error("Сначала выберите изображение")
            return
        if not self.is_connected:
            self.show_error("Сначала подключитесь к устройству")
            return

        self.progress_popup = ProgressPopup()
        self.progress_popup.message = "Подготовка JPEG..."
        self.progress_popup.open()
        threading.Thread(target=self.run_send_image, daemon=True).start()

    def send_test_image(self, instance):
        if not self.is_connected:
            self.show_error("Сначала подключитесь к устройству")
            return

        print("🎨 Создание тестового JPEG...")
        from PIL import ImageDraw
        img = Image.new('RGB', TARGET_SIZE, color='black')
        draw = ImageDraw.Draw(img)
        # Рисуем несколько полос для теста
        for i in range(0, TFT_WIDTH, 50):
            draw.rectangle([i, 0, i+25, TFT_HEIGHT], fill=(255,0,0))
        for j in range(0, TFT_HEIGHT, 100):
            draw.rectangle([0, j, TFT_WIDTH, j+50], fill=(0,255,0))
        output = io.BytesIO()
        img.save(output, format='JPEG', quality=85, progressive=False)
        test_data = output.getvalue()

        self.progress_popup = ProgressPopup()
        self.progress_popup.message = "Отправка тестового JPEG..."
        self.progress_popup.open()
        threading.Thread(target=self.run_send_test_image, args=(test_data,), daemon=True).start()

    def run_send_image(self):
        try:
            Clock.schedule_once(lambda dt: self.update_progress("Ресайз и сжатие...", 10))
            jpeg_data = process_image_to_jpeg(self.selected_file)
            # Сохраняем JPEG для проверки
            with open("sent_image.jpg", "wb") as f:
                f.write(jpeg_data)
            print("💾 JPEG сохранён в sent_image.jpg для проверки")
            Clock.schedule_once(lambda dt: self.update_progress("Создание пакетов...", 30))
            packets = create_image_packets(jpeg_data)

            Clock.schedule_once(lambda dt: self.update_progress("Отправка...", 50))
            success = async_manager.run_coroutine(self.send_packets_async(packets, response=True), timeout=120)

            if success:
                Clock.schedule_once(lambda dt: self.on_image_sent_success())
            else:
                Clock.schedule_once(lambda dt: self.show_error("Ошибка отправки"))
        except Exception as e:
            print(f"❌ Ошибка: {e}")
            import traceback
            traceback.print_exc()
            error_msg = str(e)
            Clock.schedule_once(lambda dt: self.show_error(f"Ошибка: {error_msg}"))
        finally:
            Clock.schedule_once(lambda dt: self.close_progress_popup())

    def run_send_test_image(self, test_data):
        try:
            Clock.schedule_once(lambda dt: self.update_progress("Создание пакетов...", 30))
            packets = create_image_packets(test_data)

            Clock.schedule_once(lambda dt: self.update_progress("Отправка тестового JPEG...", 50))
            success = async_manager.run_coroutine(self.send_packets_async(packets, response=True), timeout=120)

            if success:
                Clock.schedule_once(lambda dt: self.on_test_image_sent_success())
            else:
                Clock.schedule_once(lambda dt: self.show_error("Ошибка отправки теста"))
        except Exception as e:
            error_msg = str(e)
            Clock.schedule_once(lambda dt: self.show_error(f"Ошибка: {error_msg}"))
        finally:
            Clock.schedule_once(lambda dt: self.close_progress_popup())

    async def send_packets_async(self, packets, response):
        try:
            if not self.ble_client or not self.ble_client.is_connected:
                print("❌ Не подключено к устройству")
                return False
            if not self.tx_characteristic:
                print("❌ Не найдена характеристика")
                return False

            print(f"📤 Начало отправки {len(packets)} пакетов (response={response})...")
            total_packets = len(packets)
            for i, packet in enumerate(packets):
                try:
                    await self.ble_client.write_gatt_char(
                        self.tx_characteristic,
                        packet,
                        response=response
                    )
                    progress = 50 + (i / total_packets) * 50
                    if i % 10 == 0 or i == total_packets-1:
                        Clock.schedule_once(lambda dt, idx=i, tot=total_packets: self.update_progress(f"Отправка {idx+1}/{tot}", progress))
                    await asyncio.sleep(0.01)
                except Exception as e:
                    print(f"❌ Ошибка пакета {i+1}: {e}")
                    return False
            print(f"✅ Отправлено {total_packets} пакетов")
            return True
        except Exception as e:
            print(f"❌ Ошибка отправки: {e}")
            return False

    def update_progress(self, message, progress):
        if self.progress_popup:
            self.progress_popup.message = message
            self.progress_popup.progress = progress

    def close_progress_popup(self):
        if self.progress_popup:
            self.progress_popup.dismiss()
            self.progress_popup = None

    def on_image_sent_success(self):
        self.status_text = "JPEG отправлен"
        filename = Path(self.selected_file).name
        self.info_label.text = f"✅ {filename} отправлен!"
        success_popup = Popup(
            title="Успех!",
            content=Label(text="Изображение отправлено на дисплей"),
            size_hint=(0.6,0.3)
        )
        success_popup.open()

    def on_test_image_sent_success(self):
        self.status_text = "Тест отправлен"
        self.info_label.text = "✅ Тестовый JPEG отправлен!"
        success_popup = Popup(
            title="Успех!",
            content=Label(text="Тестовое изображение отправлено"),
            size_hint=(0.6,0.3)
        )
        success_popup.open()

    def show_error(self, message):
        self.status_text = "Ошибка"
        self.info_label.text = message
        error_popup = Popup(title="Ошибка", content=Label(text=message), size_hint=(0.6,0.3))
        error_popup.open()

    def finish_scan(self):
        self.status_text = "Сканирование завершено"
        self.scan_button.disabled = False
        self.scan_button.text = "🔍 Сканировать BLE"
        self.scanning = False

    def on_request_close(self, *args):
        if not self._is_closing:
            self._is_closing = True
            # Запускаем очистку в отдельном потоке, чтобы не блокировать UI
            threading.Thread(target=self.cleanup_and_exit, daemon=True).start()
        return True

    def cleanup_and_exit(self):
        print("🛑 Завершение работы...")
        if self.progress_popup:
            Clock.schedule_once(lambda dt: self.progress_popup.dismiss(), 0)
        if self.ble_client and self.ble_client.is_connected:
            try:
                async_manager.run_coroutine(self.ble_client.disconnect(), timeout=5)
            except Exception as e:
                print(f"⚠️ Ошибка при отключении: {e}")
        async_manager.stop()
        time.sleep(1)  # даём время на завершение asyncio
        # Принудительное завершение процесса
        os._exit(0)

# ========== ЗАПУСК ==========
if __name__ == '__main__':
    print("=" * 60)
    print("BLE JPEG Sender для ESP32 RGB Display")
    print("=" * 60)
    print(f"📐 Размер TFT дисплея: {TFT_WIDTH}x{TFT_HEIGHT}")
    print(f"⚙️ Качество JPEG: {JPEG_QUALITY}")
    print("\n🚀 Запуск приложения...")
    print("=" * 60)
    async_manager.start_loop()
    try:
        BLEImageSenderApp().run()
    except KeyboardInterrupt:
        print("\n🛑 Приложение остановлено пользователем")
    finally:
        print("🧹 Очистка ресурсов...")
        async_manager.stop()
        print("👋 Завершение работы")
