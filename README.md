# STM32 Drone & Remote Controller System

Dự án cá nhân về thiết kế, lập trình hệ thống điều khiển Drone (Quadcopter) và Bộ điều khiển từ xa (Remote Controller) sử dụng vi điều khiển STM32. Hệ thống tích hợp khả năng đọc cảm biến dòng dữ liệu IMU, tính toán cân bằng góc bằng bộ điều khiển PID và truyền nhận dữ liệu không dây thời gian thực.

---

## 1. Thành Phần Phần Cứng Tích Hợp (Hardware Components)

Dựa trên cấu trúc mã nguồn, hệ thống tập trung vào việc xử lý các module ngoại vi cốt lõi sau:
- **Vi điều khiển chính:** STM32 (phục vụ tính toán và điều khiển tốc độ động cơ/đọc joystick).
- **Cảm biến góc (IMU):** MPU6050 (giao tiếp qua I2C để thu thập dữ liệu gia tốc và vận tốc góc).
- **Truyền nhận không dây:** NRF24L01 (giao tiếp qua SPI để truyền tín hiệu điều khiển từ Remote tới Drone với độ trễ thấp).
- **Màn hình hiển thị (phía Remote):** LCD tích hợp module chuyển đổi I2C (i2c_lcd) dùng để giám sát trạng thái, thông số PID hoặc kênh sóng.

---

## 2. Cấu Trúc Các File Trong Repository

Repository chứa toàn bộ driver ngoại vi và các thuật toán điều khiển cốt lõi được viết bằng ngôn ngữ C:

- `README.md`: Tài liệu hướng dẫn tổng quan dự án.
- `main.c` / `main.h`: Luồng chương trình chính, khởi tạo cấu hình hệ thống (Clock, GPIO, Peripherals) và vòng lặp điều khiển thời gian thực.
- `mpu6050.c` / `mpu6050.h`: Thư viện cấu hình các thanh ghi chức năng của MPU6050, đọc dữ liệu thô (Raw data) và tính toán góc nghiêng (Roll/Pitch).
- `pid.c` / `pid.h`: Thuật toán điều khiển PID (Proportional-Integral-Derivative) chịu trách nhiệm tính toán sai số góc và điều chỉnh độ ổn định cân bằng cho Drone.
- `NRF24.c` / `NRF24.h` / `NRF24_conf.h` / `NRF24_reg_addresses.h`: Bộ thư viện driver hoàn chỉnh cho module NRF24L01, định nghĩa các thanh ghi địa chỉ, cấu hình công suất, kênh tần số và thiết lập luồng dữ liệu (Payload) truyền/nhận.
- `i2c_lcd.c` / `i2c_lcd.h`: Driver điều khiển màn hình LCD qua giao tiếp I2C để hiển thị thông tin cấu hình.

---

## 3. Kiến Trúc Phần Mềm & Luồng Xử Lý (Software Flow)

Hệ thống hoạt động dựa trên sự phối hợp chặt chẽ giữa mạch thu phát (Remote) và mạch xử lý trung tâm (Drone):

1. **Thu thập dữ liệu (Data Acquisition):** 
   - Đọc dữ liệu từ cảm biến `MPU6050` định kỳ, áp dụng bộ lọc (ví dụ: Bộ lọc bù - Complementary Filter) để trích xuất góc nghiêng thực tế của Drone một cách chính xác nhất.
2. **Truyền nhận không dây (RF Communication):**
   - Mạch Remote đọc trạng thái các nút nhấn/Joystick, đóng gói dữ liệu và gửi qua `NRF24L01`.
   - Mạch Drone nhận gói tin dữ liệu, giải mã các lệnh điều khiển (Throttle, Yaw, Pitch, Roll).
3. **Vòng lặp điều khiển cân bằng (PID Control Loop):**
   - Bộ điều khiển `PID` tiếp nhận giá trị góc đặt (SetPoint từ Remote) và góc hiện tại (Feedback từ MPU6050).
   - Tính toán giá trị output thích hợp để điều chỉnh chu kỳ xung PWM cấp cho các bộ điều tốc (ESC) điều khiển động cơ.
