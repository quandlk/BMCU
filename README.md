# BMCU

#### Giới thiệu

BMCU được thiết kế với bốn kênh làm một đơn vị, hiện sử dụng vi điều khiển CH32 làm bộ điều khiển chính. Tài liệu thiết kế được tham khảo từ các nguồn công khai trên mạng và các thử nghiệm cá nhân. Chương trình được phát triển dựa trên thư viện hỗ trợ Arduino cho vi điều khiển CH32 trên nền tảng Platform IO, sử dụng thư viện CRC của robtillaart.  
**Lưu ý: Dự án này tuân theo giấy phép mã nguồn mở GPL 2.0, nhưng cần bổ sung rằng dự án này cấm sử dụng cho mục đích thương mại.**

#### Hướng dẫn sử dụng và cài đặt

**1. Tài liệu cần thiết để chế tạo xem trong tệp tích hợp BMCU**

Tệp tích hợp BMCU hiện được phát hành trong nhóm hoặc có thể tải từ phần đính kèm tại [https://oshwhub.com/bamboo-shoot-xmcu-pcb-team/bmcu](https://oshwhub.com/bamboo-shoot-xmcu-pcb-team/bmcu), tìm tệp tích hợp phiên bản V1.1.

**2. Firmware cần ghi xem trong phần release**

**3. Hướng dẫn cài đặt và sử dụng xem trong wiki (do thành viên nhóm @丸子 khởi xướng, đang được cộng đồng xây dựng)**

Địa chỉ: [https://bmcu.wanzii.cn/](https://bmcu.wanzii.cn/)

#### Kiến trúc phần mềm

Các tệp chính:

main.cpp/h:               Chịu trách nhiệm điều phối các mô-đun  
many_soft_AS5600.cpp/h:   Trình điều khiển mô phỏng IO, có thể giao tiếp đồng thời với nhiều AS5600  
Motion_control.cpp/h:     Chịu trách nhiệm điều phối phần cứng và trạng thái chuyển động  
Flash_saves.cpp/h:        Dùng để lưu dữ liệu vào bộ nhớ flash  
time64.cpp/h:             Chuyển đổi cơ số thời gian 32-bit của Arduino sang 64-bit, ngăn chặn tràn số sau vài tháng hoạt động liên tục  
Debug_log.cpp/h:          Dùng để gửi dữ liệu DEBUG qua DMA đến cổng serial 3  
BambuBus.cpp/h:           Hỗ trợ giao tiếp với máy in Bambu, sử dụng cổng serial 0  
Klipper.cpp/h:            (Đã dừng phát triển do có nhiều dòng máy Klipper) Hỗ trợ giao tiếp với Klipper  
ws2812.cpp/h:             Trình điều khiển WS2812 mô phỏng IO  

Các thư viện sử dụng:

robtillaart/CRC@^1.0.3     Dùng để tính toán kiểm tra CRC  

#### Đóng góp

Nhà thiết kế:

4061N - Lập trình viên, tối ưu hóa phần cơ khí, thiết kế PCB  
括号 (Ngoặc) - Tham gia thiết kế khung thành phần BMCU và giá đỡ từ BMCU đến máy in  

Một số thành viên nhóm khác cũng tham gia thiết kế, hiện chưa được liệt kê.  

Người thử nghiệm và cung cấp dữ liệu:

Phong Tuyết(风雪) - Cung cấp dữ liệu giao tiếp giữa máy in và AMS, tham gia một phần thử nghiệm  
Nhị Nguyệt Miêu - Cung cấp một lượng nhỏ dữ liệu giao tiếp chất lượng cao, tham gia một phần thử nghiệm  

Các thành viên khác:

Bà Lão(婆老) - Chịu trách nhiệm "lướt net" trực tuyến  
Các thành viên nhóm chưa được liệt kê - Cung cấp dữ liệu thử nghiệm và đề xuất quý giá  