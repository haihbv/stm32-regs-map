# 🚀 STM32F1 Register-Based Driver Library

![STM32](https://img.shields.io/badge/STM32-F103C8-blue?style=for-the-badge&logo=stmicroelectronics)
![C](https://img.shields.io/badge/C-Language-orange?style=for-the-badge&logo=c)
![Keil](https://img.shields.io/badge/Keil-uVision5-red?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)

> 🔧 Thư viện driver bare-metal cho vi điều khiển STM32F103C8 sử dụng thao tác trực tiếp với thanh ghi, không phụ thuộc vào HAL/SPL.

## ✨ Tính Năng Chính

- **🔌 GPIO Driver** - Cấu hình và điều khiển các chân GPIO với nhiều chế độ
- **📊 ADC Driver** - Đọc giá trị analog từ các kênh ADC
- **📡 UART Communication** - Giao tiếp UART với hỗ trợ printf đa tham số
- **⏱️ Timer (TIM)** - Định thời, PWM và đếm xung
- **🔔 External Interrupt (EXTI)** - Xử lý ngắt từ GPIO
- **⚙️ System Clock (RCC)** - Quản lý và cấu hình clock hệ thống
- **⏰ SysTick Timer** - Delay chính xác và scheduling
- **🔗 AFIO** - Cấu hình chức năng thay thế cho GPIO
- **🔄 I2C/SPI Communication** - Giao tiếp với các thiết bị ngoại vi
- **📋 Ring Buffer** - Buffer vòng tròn hiệu quả cho dữ liệu

## 📁 Cấu Trúc Dự Án

```
📦 regs_map/
├── 📂 core/                 # Mã ứng dụng chính
│   └── main.c              # File main chính
├── 📂 drivers/             # Triển khai driver
│   ├── 📂 inc/             # File header (.h)
│   │   ├── adc.h          # ADC driver header
│   │   ├── afio.h         # AFIO driver header
│   │   ├── exti.h         # External interrupt header
│   │   ├── gpio.h         # GPIO driver header
│   │   ├── iic.h          # I2C driver header
│   │   ├── rcc.h          # Clock system header
│   │   ├── spi.h          # SPI driver header
│   │   ├── systick.h      # SysTick timer header
│   │   ├── tim.h          # Timer driver header
│   │   └── uart.h         # UART driver header
│   └── 📂 src/             # File nguồn (.c)
│       ├── adc.c          # ADC implementation
│       ├── afio.c         # AFIO implementation
│       ├── exti.c         # External interrupt implementation
│       ├── gpio.c         # GPIO implementation
│       ├── iic.c          # I2C implementation
│       ├── rcc.c          # Clock system implementation
│       ├── spi.c          # SPI implementation
│       ├── systick.c      # SysTick implementation
│       ├── tim.c          # Timer implementation
│       └── uart.c         # UART implementation
├── 📂 example/             # Ví dụ sử dụng
│   ├── ds3231_example.c   # Ví dụ sử dụng DS3231 RTC
│   ├── i2c.c              # Ví dụ I2C
│   ├── ring_buffer_example.c  # Ví dụ ring buffer
│   ├── spi_send1byte.c    # Ví dụ SPI
│   └── variadic_usart_example.c  # Ví dụ UART printf
├── 📂 modules/             # Các module bổ sung
│   ├── 📂 ds3231/         # Module DS3231 RTC
│   ├── 📂 ring_buffer/    # Ring buffer implementation
│   └── 📂 variadic_uart/  # UART với printf support
├── 📂 system/              # System headers
│   ├── peripherals.h      # Peripheral definitions
│   ├── stm32system.h      # System configuration
│   └── stm32util.h        # Utility functions
└── 📂 keil_project/        # Keil uVision project files
    ├── registor.uvprojx   # Keil project file
    └── ...                # Build outputs và config files
```

## 🚀 Bắt Đầu

### 📋 Yêu Cầu Hệ Thống

- **IDE:** Keil uVision 5 (MDK-ARM)
- **MCU:** STM32F103C8T6 (Blue Pill board)
- **Compiler:** ARM Compiler 5 hoặc 6
- **Debugger:** ST-Link V2 (khuyến nghị)

### 📥 Cài Đặt và Sử Dụng

1. **Clone hoặc download** dự án về máy
2. **Mở file project** `keil_project/registor.uvprojx` trong Keil uVision
3. **Cấu hình target device** thành STM32F103C8
4. **Build project** (F7) để biên dịch
5. **Download** code vào MCU (F8)

### 🎯 Sử Dụng Nhanh

```c
#include "stm32system.h"
#include "peripherals.h"

int main(void) {
    // Cấu hình clock hệ thống 72MHz từ HSE
    RCC_Clock72MHz_HSE();
    
    // Khởi tạo SysTick timer
    SysTick_Init();
    
    // Enable clock cho GPIOC
    RCC_APB2ClockCmd(RCC_APB2_GPIOC, ENABLE);
    
    // Cấu hình GPIO cho LED PC13
    GPIO_InitTypeDef gpio;
    gpio.Pin = GPIO_PIN_13;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_50MHZ;
    GPIO_Init(GPIOC, &gpio);
    
    while(1) {
        // Toggle LED PC13
        GPIOC->ODR.REG ^= (1 << 13);
        delay_ms(1000); // Delay 1 giây
    }
}
```

### 📡 Sử Dụng UART với Printf

```c
#include "uart_vprintf.h"

int main(void) {
    // Khởi tạo UART1 với baudrate 9600
    UART1.Init(USART_BRR_9600);
    
    // Gửi thông báo khởi tạo
    UART1.Print("STM32F103C8 Started!\r\n");
    
    uint8_t buffer[128];
    int counter = 0;
    
    while(1) {
        // Gửi số đếm
        UART1.Print("Counter: %d\r\n", counter++);
        
        // Kiểm tra dữ liệu nhận
        if (UART1.Scan(buffer)) {
            UART1.Print("Received: %s\r\n", buffer);
        }
        
        delay_ms(1000);
    }
}
```
```

## 🔧 Driver APIs

### GPIO Driver
```c
// Cấu trúc khởi tạo GPIO
typedef struct {
    uint16_t Pin;       // GPIO_PIN_0 đến GPIO_PIN_15
    GPIO_Mode_t Mode;   // GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT, etc.
    GPIO_Speed_t Speed; // GPIO_SPEED_10MHZ, GPIO_SPEED_50MHZ, etc.
} GPIO_InitTypeDef;

// Khởi tạo GPIO
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);

// Điều khiển GPIO
void GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState state);
void GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint8_t GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
```

### UART Driver (Standard)
```c
// Cấu trúc UART handle
typedef struct {
    USART_TypeDef *USARTx;  // USART1, USART2, USART3
    uint32_t BaudRate;      // USART_BRR_9600, USART_BRR_115200, etc.
    uint32_t WordLength;    // Word length configuration
    uint32_t StopBits;      // Stop bits configuration
    FunctionalState IRQ_Enable; // Interrupt enable/disable
} USART_HandleTypeDef;

// UART functions
void USART_Init(USART_HandleTypeDef *huart);
void USART_SendChar(USART_TypeDef *USARTx, char c);
void USART_SendString(USART_TypeDef *USARTx, const char *str);
```

### UART Printf Module
```c
// UART với hỗ trợ printf
typedef struct {
    void (*Init)(uint32_t baudrate);
    void (*Print)(const char *str, ...);
    uint8_t (*Scan)(uint8_t *data);
} UART_TypeDefStruct;

// Sử dụng
extern UART_TypeDefStruct UART1, UART2, UART3;
UART1.Init(USART_BRR_9600);
UART1.Print("Hello %s, Number: %d\r\n", "World", 123);
```

### System Clock (RCC)
```c
// Cấu hình clock hệ thống
void RCC_Clock72MHz_HSE(void);  // Clock 72MHz từ HSE

// Enable/Disable peripheral clock
void RCC_APB2ClockCmd(uint32_t periph_mask, FunctionalState state);
void RCC_APB1ClockCmd(uint32_t periph_mask, FunctionalState state);

// Peripheral masks
// RCC_APB2_GPIOA, RCC_APB2_GPIOB, RCC_APB2_GPIOC
// RCC_APB2_USART1, RCC_APB1_USART2, RCC_APB1_I2C1, etc.
```

### SysTick Timer
```c
// SysTick functions
void SysTick_Init(void);        // Khởi tạo SysTick
void delay_ms(uint32_t ms);     // Delay mili giây
uint32_t millis(void);          // Lấy thời gian từ khởi động (ms)
```

## 📖 Ví Dụ Sử Dụng

Kiểm tra thư mục `📂 example/` để xem các ví dụ chi tiết:

### 🔄 Ring Buffer Example
```bash
example/ring_buffer_example.c
```
- Quản lý dữ liệu UART hiệu quả
- Buffer vòng tròn với FIFO

### 📡 UART Printf Example  
```bash
example/variadic_usart_example.c
```
- Sử dụng printf với UART
- Debug và logging dễ dàng

### 🕐 DS3231 RTC Example
```bash
example/ds3231_example.c
```
- Giao tiếp I2C với DS3231
- Đọc thời gian thực

### 📊 SPI Communication
```bash
example/spi_send1byte.c
```
- Gửi dữ liệu qua SPI
- Giao tiếp với sensor/display

## 🛠️ Tính Năng Nâng Cao

### 📡 I2C Communication
- Master mode communication
- Multi-byte read/write
- Clock stretching support

### 🔄 SPI Interface  
- Full-duplex communication
- Configurable clock polarity/phase
- Hardware NSS management

### ⏱️ Timer Functions
- PWM generation
- Input capture
- Output compare
- Encoder interface

### 🔔 Interrupt Handling
- External interrupts (EXTI)
- Timer interrupts  
- UART interrupts
- Nested interrupt support

## 🎯 Roadmap

- [ ] **CAN Bus Driver** - Giao tiếp CAN 2.0
- [ ] **USB CDC Driver** - USB Communication  
- [ ] **DMA Support** - Direct Memory Access
- [ ] **Low Power Modes** - Sleep/Stop/Standby
- [ ] **RTC Driver** - Real-time clock
- [ ] **Flash Memory** - Internal flash programming

## 🤝 Đóng Góp

Chúng tôi hoan nghênh mọi đóng góp! Hãy:

1. **Fork** dự án này
2. **Tạo branch** cho feature mới (`git checkout -b feature/AmazingFeature`)
3. **Commit** thay đổi (`git commit -m 'Add some AmazingFeature'`)
4. **Push** lên branch (`git push origin feature/AmazingFeature`)
5. **Mở Pull Request**

## 📞 Liên Hệ & Hỗ Trợ

- 📧 **Email:** your-email@example.com
- 🐛 **Issues:** [GitHub Issues](https://github.com/your-username/regs_map/issues)
- � **Discussions:** [GitHub Discussions](https://github.com/your-username/regs_map/discussions)

## �📄 Giấy Phép

Dự án này được phân phối dưới giấy phép **MIT License**. Xem file `LICENSE` để biết thêm chi tiết.

```
MIT License

Copyright (c) 2025 STM32F103C8 Register Driver Library

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
```

---

<div align="center">

### 🌟 **Nếu dự án này hữu ích, hãy cho chúng tôi một ⭐ star!** 🌟

[![GitHub stars](https://img.shields.io/github/stars/your-username/regs_map?style=social)](https://github.com/your-username/regs_map/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/your-username/regs_map?style=social)](https://github.com/your-username/regs_map/network)
[![GitHub watchers](https://img.shields.io/github/watchers/your-username/regs_map?style=social)](https://github.com/your-username/regs_map/watchers)

**Made with ❤️ for STM32 Community**

*Happy Coding! 🚀*

</div>
