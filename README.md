# Lab3_Đề5_Dương Kim Tùng Dương _N02_231432199
# Lab 3: FreeRTOS trên ESP32 - Hệ thống phát hiện té ngã (Đề 5)

Môn: Hệ Thống Nhúng
Sinh viên: Dương Kim Tùng Dương - MSSV: 231432199 - N02

## Liên kết
- Báo cáo:BÁO CÁO LAB 3.docx
- Web điều khiển: [https://yangyang-01288.github.io/Lab3_De5_DuongKimTungDuong_N02_231432199/](https://yangyang-01288.github.io/Lab3_De5_DuongKimTungDuong_N02_231432199/)
- Mô phỏng Wokwi:[ https://wokwi.com/projects/480202976757129217](https://wokwi.com/projects/480202976757129217)
- Video thuyết trình: [https://youtu.be/5vekJ6TRYBE](https://youtu.be/5vekJ6TRYBE)

## Tóm tắt
Hệ thống phát hiện té ngã dùng **ESP32 + cảm biến MPU6050**, chạy **FreeRTOS** với 3 Task và 2 hàng đợi, gửi dữ liệu qua **MQTT (EMQX)** lên giao diện web để theo dõi và điều khiển. Mô phỏng trên Wokwi.

**Kiến trúc FreeRTOS**
| Task | Core | Ưu tiên | Nhiệm vụ |
|---|---|---|---|
| Đọc data | 1 | 2 | Đọc MPU6050 mỗi 20 ms, tính SVM, Pitch, Roll, Tilt |
| Truyền data | 0 | 1 | Wi-Fi + MQTT, gửi dữ liệu mỗi 500 ms, nhận lệnh từ web |
| Cảnh báo | 1 | 3 | Chạy thuật toán té ngã, điều khiển LED và loa |

- 2 Queue: `dataQueue` (đọc → truyền MQTT), `dataAlert` (đọc → cảnh báo)
- Mutex (`stateMutex`, `serialMutex`) bảo vệ biến dùng chung và Serial; Binary Semaphore để 3 Task in Serial theo thứ tự.

**Thuật toán phát hiện té ngã**
1. Va chạm: SVM > 1.8 g
2. Chờ xác nhận 3 giây
3. Té ngã nếu Tilt > 60° và SVM trong khoảng 0.8–1.2 g (nằm yên); nếu không thì bỏ qua là báo động giả.

**Chế độ điều khiển**: AUTO (té ngã thì LED nhấp nháy, loa kêu) và MANUAL (bật/tắt LED, loa trên web). Nút Reset xóa cảnh báo.

**Sơ đồ chân**: MPU6050 SDA = GPIO21, SCL = GPIO22; LED = GPIO15; Buzzer = GPIO18.

**Web**: hiển thị trạng thái, gia tốc, góc nghiêng, 2 biểu đồ thời gian thực và các nút điều khiển (mqtt.js + Chart.js).

## Nội dung repository
- lab3_de5_code.ino: code ESP32 (3 Task, 2 Queue, Mutex, Semaphore)
- index.html: giao diện web (MQTT, biểu đồ, nút điều khiển)
- BÁO CÁO LAB 3.docx: File báo báo bài làm

## Thư viện cần dùng trên Wokwi
PubSubClient, Adafruit MPU6050, Adafruit Unified Sensor

## Cách chạy
1. Mở dự án Wokwi, dán code .ino, chạy mô phỏng.
2. Mở link web, đợi báo "Đã kết nối ESP32".
3. Chỉnh MPU6050 trong Wokwi để thử té ngã (xem báo cáo mục 5).
