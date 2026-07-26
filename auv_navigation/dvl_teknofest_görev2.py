#!/usr/bin/env python3
"""
mission_straight_dvl.py — Açık Döngü (Open-Loop) DVL Waypoint Navigasyonu
                            + Leg 1 Sonrası Sola Tam Tur (CIRCLE)
----------------------------------------------------------------------
YÖNTEM: Bir Kere Dön, Düz Git (Turn-Once-Then-Go)

mission_waypoint_dvl.py (pure pursuit) ve mission_los_dvl.py (LOS)
dosyalarından farkı: bu node yol boyunca AÇIYI YENİDEN HESAPLAMAZ.
Sadece bir kere hesaplar, IMU ile o açıya döner, sonra sabit o yönde
gider. DVL SADECE "ne kadar yol aldım" ölçmek için kullanılır — sapma
düzeltmesi YOK.

Akış (2 waypoint + araya sıkıştırılmış tam tur):
  1) TURN state'inde: leg başlangıcındaki PLANLANAN (ideal) noktadan
     hedefe olan açı (atan2) BİR KERE hesaplanır, /auv/heading_setpoint'e
     basılır. Araç dönerken ilerlemez (cmd_vel.linear.x = 0).
  2) Dönüş bitti sayılır: ölçülen heading (sensor_node'dan gelen ham
     /auv/sensors/heading, controller'ın kullandığı ters çevrilmiş
     kuralla) hedef açıya HEADING_TOLERANCE_DEG içine girince.
  3) STRAIGHT state'ine geçilir: heading_setpoint bir daha DEĞİŞMEZ,
     sabit tutulur. cmd_vel.linear.x sabit ileri verilir. Mesafe
     SAYACI, bu leg'e GERÇEKTEN başladığı andaki DVL konumundan
     (dist_ref_x/y) ölçülür — atalet/momentum kayması bir sonraki
     leg'e "bedava ilerleme" olarak yansımasın diye.
  4) 1. waypoint'e varınca (leg 1 bitince): DOĞRUDAN 2. waypoint'e
     dönmek yerine CIRCLE state'ine girilir (bkz. aşağıdaki YENİ ÖZELLİK).
  5) CIRCLE bitince: 2. waypoint için normal TURN/STRAIGHT akışı devam
     eder — açı ve mesafe yine PLANLANAN (2,-2)→(-1,-1) koordinatlarından
     hesaplanır, CIRCLE'ın gerçekte ne kadar kaydığı bu hesaba KARIŞMAZ
     (ideal geometri mantığı, drift görmezden gelinir).

YENİ ÖZELLİK (LEG 1 SONRASI SOLA TAM TUR — mission_ileri_kategori.py'den
uyarlandı):
  mission_ileri_kategori.py'deki CIRCLE mekanizması buraya taşındı:
    - Heading PID'e/heading_setpoint'e GÜVENİLMİYOR (o mekanizma daire
      için güvenilmez çıkmıştı — bkz. mission_ileri_kategori.py'nin
      REVİZYON NOTLARI, madde 8).
    - Bunun yerine DOĞRUDAN sabit bir cmd_vel (ileri + açısal hız)
      komutu gönderiliyor — bu, controller'ın heading PID'ini TAMAMEN
      bypass ediyor (aynı mekanizma /auv/heading_setpoint yerine
      doğrudan ang_z komutuyla dönen TURN state'lerinde de kullanılır).
    - Dönüşün bittiği, GERÇEK ölçülen accumulated_yaw (ardışık IMU
      okumaları arasındaki farkların toplamı) ile, CIRCLE_CONFIRM_TICKS
      ardışık tick boyunca debounce'lu şekilde doğrulanıyor — tek bir
      anlık sıçramayla yanlışlıkla "tamamlandı" denmesin diye.
  TEK FARK — YÖN: mission_ileri_kategori.py'de daire SAĞA dönerek
  atılıyordu (ang_z NEGATİF = SAĞA). Burada bilerek TERSİ yapıldı:
  ang_z POZİTİF gönderiliyor → SOLA döner. (Bu işaret kuralı, bu
  mission'ın kullandığı aynı controller ailesinde — ang_z pozitif iken
  sağ taraf motorları hızlanıp sol taraf yavaşlıyor, bu da aracı SOLA
  döndürüyor — hem holonomik mikserli hem klasik diferansiyel mikserli
  controller sürümlerinde tutarlı.)
  Daire, tam bir devir (~355°, tolerans debounce ile) tamamlanınca
  aracı yaklaşık aynı konum ve AÇIDA bırakır (ileri hız + sabit açısal
  hız kombinasyonu gerçek bir daire yörüngesi çizdiği için) — devam
  eden leg'in açı/mesafe hesabına etkisi yoktur (ideal geometri).

CONTROLLER TARAFINDA DEĞİŞİKLİK GEREKMİYOR:
  Bu mission, kendi auv_controller.py'nizin (Manuel Override + "Hakem
  Bırakma Yönü Telafisi Kaldırılmış" sürüm) ZATEN sahip olduğu şu
  mekanizmayı kullanıyor:
      elif self.ang_z != 0.0:
          pid_out = int(self.ang_z * self.base_speed)   # heading PID BYPASS
          ...
      elif self._prev_ang_z != 0.0 and self.ang_z == 0.0:
          self.reference_heading = self.current_heading  # çıkışta OTOMATİK kilitleme
          ...
  Yani cmd_vel ile doğrudan ang_z basıldığında controller heading PID'i
  devre dışı bırakıp o açısal hızı doğrudan motorlara uyguluyor, ve
  ang_z=0'a döndüğünde de mevcut açıyı otomatik referans olarak
  kilitliyor. Bu davranış zaten mevcut, hiçbir satır eklemenize/
  değiştirmenize gerek YOK. Mission bu CIRCLE state'i boyunca sadece
  cmd_vel'e (lin_x, ang_z) basıyor, controller'ınız bunu zaten doğru
  işliyor.

Kullanılan mevcut mimari — pure pursuit / LOS versiyonlarıyla aynı
topic seti, ek olarak ham heading da dinleniyor (TURN/CIRCLE state'i
için):
  Subscribe: /auv/sensors/dvl_pos_x, /auv/sensors/dvl_pos_y,
             /auv/sensors/depth, /auv/sensors/heading (ham pusula),
             /auv/reference_heading
  Publish:   /auv/heading_setpoint, /auv/depth_setpoint, /auv/cmd_vel,
             /auv/manual_override, /auv/shutdown

NOT: /auv/sensors/heading, auv_controller.py'nin kullandığı ham pusula
değeridir (0-360, saat yönü). Mission içeride controller'la AYNI
dönüşümü uyguluyor: current_heading = (360 - raw) % 360.0 — çünkü
heading_setpoint'e bastığımız açı da atan2 (matematiksel, saat yönü
tersi) referansında, controller'ın iç current_heading'i ile aynı
referans sisteminde karşılaştırılmalı.

Görev akışı (state machine):
  WAIT_CALIBRATION → WAIT_DEPTH → TURN → STRAIGHT (leg 1)
    → CIRCLE (sola tam tur) → TURN → STRAIGHT (leg 2) → DONE
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
import math
import time

# ── Görev parametreleri ──────────────────────────────────────────────────
# 2 waypoint'lik rota. Her waypoint, bir önceki waypoint'in PLANLANAN
# koordinatından hesaplanan açı ve mesafeyle hedeflenir (ideal geometri).
WAYPOINTS = [
    (2.0, -2.0),   # 1. waypoint  (başlangıçtan buraya — bu leg'den sonra CIRCLE var)
    (1.0, -4.0),  # 2. waypoint  (1. waypoint'in PLANLANAN koordinatından buraya)
]

TARGET_DEPTH          = 0.60   # m — dalış hedefi
HEADING_TOLERANCE_DEG = 5.0    # bu toleransa girince dönüş tamamlandı sayılır
FORWARD_SPEED         = 0.6    # cmd_vel.linear.x oranı (düz giderken sabit)
ARRIVAL_MARGIN        = 0.30   # m — bu kadar yakına gelince "vardı" say
DEPTH_ARRIVAL_MARGIN  = 0.15
TURN_TIMEOUT_SEC      = 15.0   # dönüş bu süreyi geçerse yine de STRAIGHT'e geç
                                # (IMU/PID takılırsa sonsuz beklemesin diye emniyet)
SHUTDOWN_ON_FINISH    = False

# ── CIRCLE parametreleri (mission_ileri_kategori.py'den uyarlandı) ───────
# 1. waypoint'e vardıktan SONRA, 2. waypoint'e dönmeden ÖNCE atılacak
# sola tam tur için ayarlar. Mantık ve debounce yapısı ileri_kategori
# ile birebir aynı; TEK FARK yön (ang_z işareti tersine çevrildi).
CIRCLE_AFTER_LEG      = 1     # kaçıncı leg'den SONRA daire atılacak (1 = leg 1'den sonra)
CIRCLE_FORWARD_SPEED  = 0.4   # ileri_kategori'deki CIRCLE_FORWARD_SPEED ile aynı
CIRCLE_ANG_SPEED      = 0.1   # POZİTİF = SOLA (ileri_kategori'de negatifti = SAĞA)
CIRCLE_TARGET_DEG     = 355.0 # tam tur hedefi (355°, debounce toleransıyla ~360°)
CIRCLE_MAX_TIME       = 45.0  # güvenlik zaman aşımı (sn)
CIRCLE_CONFIRM_TICKS  = 3     # bu mission 10Hz çalışıyor (0.1s); 3 tick = 0.3s
                               # (ileri_kategori 20Hz'te 6 tick kullanıyordu — aynı 0.3s)


class MissionStraightDVL(Node):
    def __init__(self):
        super().__init__('mission_straight_dvl')

        # ---- Publisher'lar ----
        self.heading_sp_pub = self.create_publisher(Float32, '/auv/heading_setpoint', 10)
        self.depth_sp_pub   = self.create_publisher(Float32, '/auv/depth_setpoint',   10)
        self.cmd_vel_pub    = self.create_publisher(Twist,   '/auv/cmd_vel',          10)
        self.override_pub   = self.create_publisher(Bool,    '/auv/manual_override',  10)
        self.shutdown_pub   = self.create_publisher(Bool,    '/auv/shutdown',         10)

        # ---- Subscriber'lar ----
        self.create_subscription(Float32, '/auv/sensors/dvl_pos_x', self._pos_x_cb, 10)
        self.create_subscription(Float32, '/auv/sensors/dvl_pos_y', self._pos_y_cb, 10)
        self.create_subscription(Float32, '/auv/sensors/depth',     self._depth_cb, 10)
        self.create_subscription(Float32, '/auv/sensors/heading',   self._raw_heading_cb, 10)
        self.create_subscription(Float32, '/auv/reference_heading', self._ref_heading_cb, 10)

        # ---- Durum ----
        self.pos_x = None
        self.pos_y = None
        self.current_depth = 0.0
        self.current_heading = None   # controller'ınkiyle aynı dönüşüm uygulanmış (atan2 uyumlu)
        self.controller_calibrated = False

        # CIRCLE için: ardışık IMU okumaları arasındaki farkların toplamı
        # (mission_ileri_kategori._heading_cb ile birebir aynı mantık)
        self.accumulated_yaw = 0.0

        self.waypoints = list(WAYPOINTS)
        self.total_legs = len(self.waypoints)
        self.wp_index = 0
        self.state = 'WAIT_CALIBRATION'

        # İDEAL GEOMETRİ: her leg'in başlangıcı, bir önceki waypoint'in
        # PLANLANAN koordinatı (ilk leg için (0,0) — aracın kalkış noktası
        # kabul ediliyor). DVL'nin gerçek konumu bu listeye HİÇ karışmıyor,
        # sadece STRAIGHT fazında "ne kadar yol aldım" ölçümü için kullanılır.
        self.planned_points = [(0.0, 0.0)] + list(self.waypoints)

        # Her leg'de bir kere hesaplanan sabit değerler
        self.leg_start_x        = 0.0
        self.leg_start_y        = 0.0
        self.leg_target_heading = 0.0   # derece, bir kere hesaplanıp sabit kalır
        self.leg_length         = 0.0   # hipotenüs, m
        self.leg_initialized    = False
        self.turn_start_time    = None

        # Mesafe SAYACININ gerçek referans noktası (atalet/momentum düzeltmesi
        # — bkz. _turn_step). leg başladığında self.pos_x/pos_y'den doldurulur.
        self.dist_ref_x = 0.0
        self.dist_ref_y = 0.0

        # CIRCLE durumu
        self.circle_done       = False   # daire bir kere atıldıktan sonra True
        self._circle_confirm   = 0
        self.circle_start_time = None

        self.timer = self.create_timer(0.1, self._control_loop)  # 10 Hz

        self.get_logger().info('🚀 mission_straight_dvl başlatıldı (açık döngü — bir kere dön, düz git + sola tam tur)')
        self.get_logger().info(f'📍 Toplam {self.total_legs} waypoint: {self.waypoints}')
        self.get_logger().info(f'🔵 Leg {CIRCLE_AFTER_LEG} sonrası sola tam tur planlandı')

    # ── Callback'ler ──────────────────────────────────────────────────────
    def _pos_x_cb(self, msg: Float32):
        self.pos_x = msg.data

    def _pos_y_cb(self, msg: Float32):
        self.pos_y = msg.data

    def _depth_cb(self, msg: Float32):
        self.current_depth = msg.data

    def _raw_heading_cb(self, msg: Float32):
        # auv_controller.py'nin uyguladığı AYNI dönüşüm: current_heading = (360 - raw) % 360
        new_heading = (360.0 - msg.data) % 360.0

        # CIRCLE debounce'ı için: ardışık okumalar arasındaki işaretli farkı
        # topluyoruz (mission_ileri_kategori._heading_cb ile birebir aynı
        # mantık) — bu, "kaç derece döndüm" sorusuna 0-360 sarmalanmasından
        # etkilenmeyen, sürekli artan/azalan bir cevap verir.
        if self.current_heading is not None:
            delta = new_heading - self.current_heading
            if delta > 180.0:
                delta -= 360.0
            if delta < -180.0:
                delta += 360.0
            self.accumulated_yaw += delta

        self.current_heading = new_heading

    def _ref_heading_cb(self, msg: Float32):
        self.controller_calibrated = True

    # ── Ana kontrol döngüsü ───────────────────────────────────────────────
    def _control_loop(self):
        if self.state == 'WAIT_CALIBRATION':
            self._publish_override(True)
            if self.controller_calibrated:
                self.get_logger().info('✅ Controller kalibre oldu, dalışa geçiliyor')
                self.state = 'WAIT_DEPTH'
            return

        if self.state == 'WAIT_DEPTH':
            self._publish_override(False)
            depth_sp = Float32(); depth_sp.data = float(TARGET_DEPTH)
            self.depth_sp_pub.publish(depth_sp)
            if self.current_depth >= (TARGET_DEPTH - DEPTH_ARRIVAL_MARGIN):
                self.get_logger().info(f'✅ Hedef derinliğe ulaşıldı ({self.current_depth:.2f}m)')
                self.state = 'TURN'
            return

        if self.state == 'TURN':
            self._turn_step()
            return

        if self.state == 'STRAIGHT':
            self._straight_step()
            return

        if self.state == 'CIRCLE':
            self._circle_step()
            return

        if self.state == 'DONE':
            self._stop_cmd_vel()
            return

    # ── Dönüş fazı (bir kere açı hesapla, IMU ile dön) ────────────────────
    def _turn_step(self):
        if self.pos_x is None or self.pos_y is None or self.current_heading is None:
            self.get_logger().warn('⏳ DVL/IMU verisi bekleniyor...', throttle_duration_sec=2.0)
            self._stop_cmd_vel()
            return

        if self.wp_index >= len(self.waypoints):
            self.get_logger().info("🏁 Tüm waypoint'ler tamamlandı")
            self.state = 'DONE'
            self._stop_cmd_vel()
            if SHUTDOWN_ON_FINISH:
                sd = Bool(); sd.data = True
                self.shutdown_pub.publish(sd)
            return

        if not self.leg_initialized:
            target_x, target_y = self.waypoints[self.wp_index]
            # İDEAL GEOMETRİ: leg_start = bir önceki waypoint'in PLANLANAN
            # koordinatı (ilk leg için (0,0)). DVL'nin o anki gerçek
            # konumu (self.pos_x/self.pos_y) burada KASITLI olarak
            # kullanılmıyor — açı ve mesafe tamamen kâğıt üzerindeki
            # geometriden hesaplanıyor. CIRCLE'da gerçekte ne kadar
            # kayılmış olursa olsun bu hesaba KARIŞMAZ.
            self.leg_start_x, self.leg_start_y = self.planned_points[self.wp_index]
            dx = target_x - self.leg_start_x
            dy = target_y - self.leg_start_y
            # BİR KERE hesaplanıyor — bir daha güncellenmeyecek
            self.leg_target_heading = math.degrees(math.atan2(dy, dx)) % 360.0
            self.leg_length = math.hypot(dx, dy)

            # DÜZELTME (atalet/momentum sorunu): açı ve leg_length HÂLÂ ideal
            # plandan hesaplanıyor (yukarıda), ama mesafe SAYACININ referans
            # noktası artık planlanan koordinat DEĞİL — aracın bu leg'e
            # GERÇEKTEN başladığı andaki DVL konumu.
            self.dist_ref_x = self.pos_x
            self.dist_ref_y = self.pos_y

            self.leg_initialized = True
            self.turn_start_time = time.time()

            leg_no = self.wp_index + 1
            self.get_logger().info(
                f'📐 Leg {leg_no}/{self.total_legs} | '
                f'({self.leg_start_x:.2f},{self.leg_start_y:.2f}) → ({target_x:.2f},{target_y:.2f}) | '
                f'açı={self.leg_target_heading:.1f}° mesafe={self.leg_length:.2f}m'
            )

        # Dönüş sırasında ileri hız YOK, sadece heading_setpoint basılıyor
        self._stop_cmd_vel()
        heading_msg = Float32(); heading_msg.data = float(self.leg_target_heading)
        self.heading_sp_pub.publish(heading_msg)

        heading_err = self._angle_diff(self.leg_target_heading, self.current_heading)
        elapsed = time.time() - self.turn_start_time
        leg_no = self.wp_index + 1

        if abs(heading_err) <= HEADING_TOLERANCE_DEG:
            self.get_logger().info(
                f'✅ Leg {leg_no}/{self.total_legs} dönüş tamamlandı — '
                f'hedef={self.leg_target_heading:.1f}° şu an={self.current_heading:.1f}°, '
                f'düz gitmeye geçiliyor')
            self.state = 'STRAIGHT'
        elif elapsed > TURN_TIMEOUT_SEC:
            self.get_logger().warn(
                f'⚠️ Leg {leg_no}/{self.total_legs} dönüş zaman aşımı ({TURN_TIMEOUT_SEC}s) — '
                f'yine de düz gitmeye geçiliyor (hata={heading_err:.1f}°)')
            self.state = 'STRAIGHT'
        else:
            if int(time.time() * 2) % 2 == 0:
                self.get_logger().info(
                    f'🔄 Leg {leg_no}/{self.total_legs} dönüyor: '
                    f'hedef={self.leg_target_heading:.1f}° şu an={self.current_heading:.1f}° '
                    f'hata={heading_err:.1f}°')

    # ── Düz gitme fazı (heading sabit, sadece DVL ile mesafe ölçülüyor) ──
    def _straight_step(self):
        # heading_setpoint bir daha DEĞİŞMİYOR — sabit tutuluyor
        heading_msg = Float32(); heading_msg.data = float(self.leg_target_heading)
        self.heading_sp_pub.publish(heading_msg)

        cmd = Twist()
        cmd.linear.x = float(FORWARD_SPEED)
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

        # Mesafe SAYACI plan noktasına DEĞİL, bu leg'e GERÇEKTEN başladığı
        # andaki DVL konumuna (dist_ref) göre ölçülüyor.
        dx = self.pos_x - self.dist_ref_x
        dy = self.pos_y - self.dist_ref_y
        distance_traveled = math.hypot(dx, dy)   # sadece ÖLÇÜM, açı/hedef için kullanılmıyor
        remaining = self.leg_length - distance_traveled
        leg_no = self.wp_index + 1

        if remaining <= ARRIVAL_MARGIN:
            self.get_logger().info(
                f'✅ Leg {leg_no}/{self.total_legs} — waypoint {self.wp_index} ulaşıldı — '
                f'kat edilen={distance_traveled:.2f}m / hedef={self.leg_length:.2f}m')

            finished_leg_no = leg_no
            self.wp_index += 1
            self.leg_initialized = False   # bir sonraki waypoint için sıfırla
            self._stop_cmd_vel()

            # YENİ: bu leg CIRCLE_AFTER_LEG'e denk geliyorsa ve daire henüz
            # atılmadıysa, doğrudan TURN'e geçmek yerine önce CIRCLE'a gir.
            if finished_leg_no == CIRCLE_AFTER_LEG and not self.circle_done:
                self._start_circle()
            else:
                self.state = 'TURN'
            return

        if int(time.time() * 2) % 2 == 0:
            pct = 0.0 if self.leg_length <= 0 else min(100.0, 100.0 * distance_traveled / self.leg_length)
            self.get_logger().info(
                f'➡️  Leg {leg_no}/{self.total_legs} düz gidiyor: '
                f'kat edilen={distance_traveled:.2f}m kalan={remaining:.2f}m '
                f'(%{pct:.0f}) heading(sabit)={self.leg_target_heading:.1f}°')

    # ── CIRCLE fazı (mission_ileri_kategori.py'den uyarlandı, YÖN TERSİNE
    #    ÇEVRİLDİ: sağa yerine SOLA) ────────────────────────────────────────
    def _start_circle(self):
        self.accumulated_yaw   = 0.0
        self._circle_confirm   = 0
        self.circle_start_time = time.time()
        self.state = 'CIRCLE'
        self.get_logger().info('🔵 CIRCLE başlıyor — sola tam tur atılacak')

    def _circle_step(self):
        # Heading PID'e/heading_setpoint'e GÜVENİLMİYOR — doğrudan sabit
        # cmd_vel komutu gönderiliyor (controller'ın ang_z-bypass mekanizması
        # devreye giriyor, tıpkı TURN state'lerinde heading_setpoint yerine
        # ang_z kullanan mission'larda olduğu gibi).
        cmd = Twist()
        cmd.linear.x  = float(CIRCLE_FORWARD_SPEED)
        cmd.angular.z = float(CIRCLE_ANG_SPEED)   # POZİTİF = SOLA
        self.cmd_vel_pub.publish(cmd)

        turned  = abs(self.accumulated_yaw)
        elapsed = time.time() - self.circle_start_time

        if int(time.time() * 2) % 2 == 0:
            self.get_logger().info(
                f'🔵 CIRCLE  dönülen={turned:.1f}°  '
                f'mevcut_heading={self.current_heading:.1f}°  '
                f'elapsed={elapsed:.1f}s')

        angle_ok = turned >= (CIRCLE_TARGET_DEG - HEADING_TOLERANCE_DEG)
        timeout  = elapsed >= CIRCLE_MAX_TIME

        if angle_ok:
            self._circle_confirm += 1
        else:
            self._circle_confirm = 0

        if self._circle_confirm >= CIRCLE_CONFIRM_TICKS or timeout:
            if timeout and not angle_ok:
                self.get_logger().warn(
                    f'⚠️ CIRCLE zaman aşımına uğradı! (dönülen={turned:.1f}°, '
                    f'{CIRCLE_MAX_TIME}s doldu) Yine de devam ediliyor.')
            else:
                self.get_logger().info(
                    f'✅ CIRCLE tamamlandı (dönülen={turned:.1f}°) — '
                    f'mevcut açıya kilitleniyor: {self.current_heading:.1f}°')

            self._stop_cmd_vel()
            # Controller zaten ang_z=0'a dönünce mevcut açıyı otomatik
            # referans yapıyor (bkz. dosya başındaki CONTROLLER TARAFINDA
            # DEĞİŞİKLİK GEREKMİYOR notu) — burada ayrıca açıkça
            # heading_setpoint basmak, referansın CIRCLE çıkışındaki
            # gerçek açıyla bire bir örtüşmesini garantiliyor.
            heading_msg = Float32(); heading_msg.data = float(self.current_heading)
            self.heading_sp_pub.publish(heading_msg)

            self.circle_done = True
            self.state = 'TURN'

    def _stop_cmd_vel(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def _publish_override(self, value: bool):
        msg = Bool(); msg.data = value
        self.override_pub.publish(msg)

    @staticmethod
    def _angle_diff(target: float, current: float) -> float:
        err = target - current
        if err > 180.0: err -= 360.0
        if err < -180.0: err += 360.0
        return err


def main(args=None):
    rclpy.init(args=args)
    node = MissionStraightDVL()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 mission_straight_dvl durduruldu')
    finally:
        node._stop_cmd_vel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
