
# ROS2_WS_2026

Bu depo, otonom su altı aracımızın (AUV) ROS2 tabanlı algı, kontrol ve görev icra mimarisini barındırır. Sistem; sensör verilerinin toplanması, akıntı telafili konum tutma (station keeping) ve açık döngü waypoint navigasyonu ile tam otonom hareket kabiliyeti sağlayacak şekilde modüler olarak tasarlanmıştır.

---

## 🏗️ Mimari ve Düğümler (Nodes)

Sistem, ROS2 topic'leri üzerinden haberleşen 4 ana bileşenden oluşur:

1. **`sensor_node.py` (Algı Katmanı):**
* **IMU (BNO085 - UART-RVC):** UART-RVC protokolü ile verileri okur. Node her başladığında ilk yaw değerini bias olarak alarak `/auv/sensors/heading` bilgisini $0^\circ$'ye sıfırlar.
* **Basınç Sensörü (MS5837):** I2C hattı üzerinden anlık derinlik ve sıcaklık verilerini yayınlar.
* **DVL (WaterLinked A50):** TCP üzerinden bağlanarak yerel konum (`dvl_pos_x`, `dvl_pos_y`), gövde hızı ve yaw bilgilerini ROS2 ağına aktarır.


2. **`auv_controller.py` (Kontrol ve Akıntı Telafisi):**
* Hedef derinlik ve heading koruması sağlar.
* DVL verilerini kullanarak akıntıya karşı istasyon tutma (*station keeping*) ve rota takip (*cross-track*) düzeltmeleri üretir.


3. **`mission_straight_dvl.py` (Görev Yöneticisi):**
* Açık döngü waypoint navigasyonunu yönetir.
* Belirlenen leg geçişlerinde Heading PID'ini bypass ederek doğrudan `cmd_vel` ile sola tam tur (`CIRCLE`) manevrasını gerçekleştirir.


4. **`arduino_bridge.py` (Donanım Sürücüsü):**
* `/auv/motor_pwm` topic'inden gelen 8 motor PWM komutunu seri port (`/dev/ttyACM0`) üzerinden alt seviye kartına aktarır.



---

## 🔌 Donanım Bağlantıları ve Pin Dağılımı

### 1. Basınç Sensörü (MS5837)

* Basınç sensörü doğrudan **Jetson'ın I2C pinlerine** (kod tarafında varsayılan olarak I2C bus 7 üzerinden) fiziksel olarak bağlıdır.

### 2. IMU (BNO085 - UART-RVC Modu) ve USB-UART Dönüştürücü Bağlantısı

BNO085 sensörünü UART-RVC modunda çalıştırmak ve bilgisayara/Jetson'a bağlamak için pin konfigürasyonu şu şekildedir:

| BNO085 Pin | USB-UART Adaptör / Jetson Bağlantısı | Açıklama |
| --- | --- | --- |
| **PS0** | `3.3V` | UART-RVC mod seçimi |
| **PS1** | `GND` | UART-RVC mod seçimi |
| **SDA (TX)** | `USB-UART Adaptör RX` | Sensör veri çıkışı (Dönüştürücü RX pinine girer) |
| **3V3** | `USB-UART Adaptör 3.3V` | Besleme voltajı |
| **GND** | `USB-UART Adaptör GND` | Ortak toprak hattı |
| **SCL, RST, INT** | *Bağlantı Yok* | Bu modda kullanılmıyor |

* **Haberleşme Testi:** USB-UART dönüştürücü ile `/dev/ttyUSB0` portuna bağlı olan IMU'yu test etmek için terminalde şu komut kullanılabilir:
```bash
sudo minicom -D /dev/ttyUSB0 -b 115200

```



---

## 🚀 Kurulum ve Başlangıç

### 1. DVL Ağ Yapılandırması

DVL sensörüyle haberleşebilmek için Ethernet arabirimine uygun IP adresinin eklenmesi gerekmektedir:

```bash
sudo ip addr add 192.168.194.90/24 dev enx000ec89fc30

```

### 2. Auv Controller Offset Kullanımı

Aracın bırakılma yönüne göre başlangıç heading offset'ini ayarlamak için aşağıdaki parametre komutu kullanılabilir:

```bash
ros2 run controller auv_controller_offset --ros-args -p initial_heading_offset:=360.0

```

* *Not:* Araç sağa doğru bırakılırsa negatif offset (`-90.0`), sola doğru bırakılmak istenirse pozitif offset (`+90.0`) verilebilir.

---

## 📦 Kullanılan Harici Paketler ve Klonlama

Projede altyapı olarak yararlanılan harici bileşenler ve kaynak depoları aşağıdadır:

* **IMU Sürücüsü (BNO085 UART-RVC):**
[GitHub - alcad1us/bno085-ros2-uart-rvc](https://github.com/alcad1us/bno085-ros2-uart-rvc/blob/main/README.md)
*Klonlama komutu:*
```bash
git clone https://github.com/alcad1us/bno085-ros2-uart-rvc.git

```


* **Basınç Sensörü Kütüphanesi (MS5837):**
[GitHub - RobTillaart/MS5837](https://github.com/RobTillaart/MS5837)
*Klonlama komutu:*
```bash
git clone https://github.com/RobTillaart/MS5837.git

```
