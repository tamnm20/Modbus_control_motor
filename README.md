# Modbus_control_motor

Code	Chức năng	Kiểu dữ liệu	Đọc/Ghi	Đơn vị	Ví dụ dùng
0x01	Read Coils	Digital Output	Read	Bit	Relay, LED
0x02	Read Discrete Inputs	Digital Input	Read	Bit	Nút nhấn, sensor on/off
0x03	Read Holding Registers	Analog / Config	Read	16-bit	ADC, biến analog
0x04	Read Input Registers	Analog Input	Read	16-bit	Sensor analog
0x05	Write Single Coil	Digital Output	Write	Bit	Bật/tắt relay
0x06	Write Single Register	Analog Output	Write	16-bit	Ghi giá trị PWM
0x0F	Write Multiple Coils	Digital Output	Write	Bit	Ghi nhóm relay
0x10	Write Multiple Registers	Analog Output	Write	16-bit	Ghi nhóm giá trị analog


Code	Mode	Đọc/Ghi	Đơn vị	Dữ liệu	Ghi chú
0x01	Read Coils	Đọc	Bit	Digital Output	Trạng thái relay
0x02	Read Inputs	Đọc	Bit	Digital Input	Nút nhấn / sensor
0x03	Read Holding Registers	Đọc	Word (16-bit)	Analog Output / config	Có thể ghi
0x04	Read Input Registers	Đọc	Word (16-bit)	Analog Input	Chỉ đọc
0x05	Write Single Coil	Ghi	Bit	1 coil	0xFF00 = ON
0x06	Write Single Register	Ghi	Word	1 register	analog out
0x0F	Write Multiple Coils	Ghi	Bit	N coil	Packed bits
0x10	Write Multiple Registers	Ghi	Word	N registers	analog out


🧭 1️⃣ Read Coils — Function Code 0x01
📤 Master → Slave
┌────────┬──────────────┬──────────────────────┬──────────────────────┬───────────┐
│ Slave  │ Function     │ Starting Address Hi │ Starting Address Lo │ Quantity  │
│ Address│ Code = 0x01  │                    │                    │ of Coils  │
├────────┼──────────────┼──────────────────────┼──────────────────────┼───────────┤
│   1B   │     1B       │        1B           │        1B           │    2B     │
└───────────────────────────────────────────────────────────────────────────────┘
                                     ↓
                               + CRC16 (2B)

📥 Slave → Master
┌────────┬──────────────┬────────────┬──────────────────────────────┬──────────┐
│ Slave  │ Function     │ Byte Count │ Coil Status (LSB first)      │  CRC16   │
│ Address│ Code = 0x01  │            │ e.g. bit0=coil0, bit1=coil1… │          │
└────────┴──────────────┴────────────┴──────────────────────────────┴──────────┘

🧭 2️⃣ Read Discrete Inputs — Function Code 0x02
📤 Master
[ Slave ID ][ 0x02 ][ Start Addr Hi ][ Start Addr Lo ][ Quantity Hi ][ Quantity Lo ][ CRC16 ]

📥 Slave
[ Slave ID ][ 0x02 ][ Byte Count ][ Input Bits LSB-first... ][ CRC16 ]

🧭 3️⃣ Read Holding Registers — Function Code 0x03
📤 Master
┌────────┬────────┬────────┬────────┬────────┬────────┬────────┐
│Slave ID│ 0x03   │Addr Hi │Addr Lo │Qty Hi  │Qty Lo  │ CRC16  │
└────────┴────────┴────────┴────────┴────────┴────────┴────────┘

📥 Slave
┌────────┬────────┬────────────┬────────────┬────────────┬────────┐
│Slave ID│ 0x03   │Byte Count  │ Data Hi Lo │ Data Hi Lo │ CRC16  │
└────────┴────────┴────────────┴────────────┴────────────┴────────┘


📘 Ví dụ:
Master → 07 03 00 00 00 01 84 6C
Slave → 07 03 02 00 00 30 44

🧭 4️⃣ Read Input Registers — Function Code 0x04

Giống FC 0x03, chỉ khác loại thanh ghi (Input Registers thay vì Holding).

Master: [ Slave ][ 0x04 ][ Addr Hi ][ Addr Lo ][ Qty Hi ][ Qty Lo ][ CRC16 ]
Slave : [ Slave ][ 0x04 ][ Byte Count ][ Data Hi Lo ... ][ CRC16 ]

🧭 5️⃣ Write Single Coil — Function Code 0x05
📤 Master
┌────────┬────────┬────────────┬────────────┬────────────┬────────────┬────────┐
│Slave ID│ 0x05   │Addr Hi     │Addr Lo     │Value Hi    │Value Lo    │ CRC16  │
│        │        │            │            │FF 00=ON    │00 00=OFF   │        │
└────────┴────────┴────────────┴────────────┴────────────┴────────────┴────────┘

📥 Slave

Phản hồi giống hệt khung Master (echo toàn bộ).

🧭 6️⃣ Write Single Register — Function Code 0x06
Master: [ Slave ][ 0x06 ][ Addr Hi ][ Addr Lo ][ Data Hi ][ Data Lo ][ CRC16 ]
Slave : echo lại y hệt

🧭 7️⃣ Write Multiple Coils — Function Code 0x0F
📤 Master
┌────────┬────────┬────────────┬────────────┬────────────┬────────────┬────────────┬──────────┐
│Slave ID│ 0x0F   │Start Addr Hi│Start Addr Lo│Qty Coils Hi│Qty Coils Lo│Byte Count │Coil Data │
│        │        │              │              │             │             │(ceil(N/8))│(LSB first)│
└────────────────────────────────────────────────────────────────────────────────────────────┘
                                         ↓
                                       + CRC16


📘 Ví dụ:
0A 0F 00 13 00 0A 02 CD 01 BF 85

📥 Slave
[ Slave ][ 0x0F ][ Start Addr Hi ][ Start Addr Lo ][ Qty Hi ][ Qty Lo ][ CRC16 ]

🧭 8️⃣ Write Multiple Registers — Function Code 0x10
📤 Master
┌────────┬────────┬────────────┬────────────┬────────────┬────────────┬────────────┬────────────┬────────┐
│Slave ID│ 0x10   │Start Addr Hi│Start Addr Lo│Qty Reg Hi │Qty Reg Lo │Byte Count │Data Hi Lo...│ CRC16  │
└──────────────────────────────────────────────────────────────────────────────────────────────────────┘


📘 Ví dụ:
10 10 00 01 00 02 04 00 0A 01 02 92 CD

📥 Slave
[ Slave ][ 0x10 ][ Start Addr Hi ][ Start Addr Lo ][ Qty Hi ][ Qty Lo ][ CRC16 ]

🧭 9️⃣ Exception Response

Nếu Slave gặp lỗi (địa chỉ sai, giá trị sai...), byte Function Code được OR với 0x80, kèm mã lỗi 1 byte:

┌────────┬────────────────┬────────────────┬────────┐
│Slave ID│FuncCode+0x80   │Exception Code  │ CRC16  │
└────────┴────────────────┴────────────────┴────────┘

Exception Code	Ý nghĩa
0x01	Illegal Function
0x02	Illegal Data Address
0x03	Illegal Data Value
0x04	Slave Device Failure

📘 Ví dụ:
01 83 02 C0 F1 → Slave ID 1, Function 0x03 lỗi “Illegal Data Address”.