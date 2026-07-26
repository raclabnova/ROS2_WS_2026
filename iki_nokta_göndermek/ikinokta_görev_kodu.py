#!/usr/bin/env python3
"""
mission_straight_dvl.py — Açık Döngü (Open-Loop) DVL Waypoint Navigasyonu
----------------------------------------------------------------------
YÖNTEM: Bir Kere Dön, Düz Git (Turn-Once-Then-Go)

Bu node yol boyunca AÇIYI YENİDEN HESAPLAMAZ. Sadece bir kere hesaplar,
IMU ile o açıya döner, sonra sabit o yönde gider. DVL SADECE "ne kadar
yol aldım" ölçmek için kullanılır — sapma düzeltmesi YOK.

Akış (2 waypoint):
  1) TURN state'inde: leg başlangıcındaki PLANLANAN (ideal) noktadan
     hedefe olan açı (atan2) BİR KERE hesaplanır, /auv/heading_setpoint'e
     basılır. Araç dönerken ilerlemez (cmd_vel.linear.x = 0).
  2) Dönüş bitti sayılır: ölçülen heading (sensor_node'dan gelen ham
     /auv/sensors/heading, controller'ın kullandığı ters çevrilmiş
     kuralla) hedef açıya HEADING_TOLERANCE_DEG içine girince.
  3) STRAIGHT state'ine geçilir: heading_setpoint bir daha DEĞİŞMEZ,
     sabit tutulur. cmd_vel.linear.x sabit ileri verilir.
  4) DVL'den okunan pozisyonla, leg başlangıcından bu yana alınan yol
     (Euclidean mesafe) hesaplanır. Bu mesafe, başta hesaplanan
     hipotenüse (leg_length) ulaşınca "vardı" sayılır ve durulur.

İDEAL GEOMETRİ (açı + hedef mesafe):
  "Ne yöne dönmem lazım" ve "ne kadar gitmem lazım" soruları HER ZAMAN
  İKİ PLANLANAN (ideal) NOKTA arasından hesaplanır:
      leg_start = bir önceki waypoint'in PLANLANAN koordinatı
                  (ilk leg için (0,0) — aracın kalkış noktası)
      leg_target_heading = atan2(target_y - leg_start_y, target_x - leg_start_x)
      leg_length          = hypot(target_x - leg_start_x, target_y - leg_start_y)
  Bu ikisi BİR KERE hesaplanır, leg boyunca hiç güncellenmez. Aracın
  DVL'ye göre gerçekte nerede olduğu (sürüklenme/akıntı sapması) bu
  hesaba HİÇ KARIŞMAZ — drift görmezden gelinir (ideal geometri).

MESAFE SAYACI (atalet/momentum düzeltmesi):
  Yukarıdaki "ne kadar gitmem lazım" (leg_length) ideal plandan gelse
  de, "şu ana kadar GERÇEKTEN ne kadar gittim" sorusu ayrı bir referans
  noktasından (dist_ref_x/y) ölçülür — bu nokta, PLAN koordinatı DEĞİL,
  aracın bu leg'e GERÇEKTEN başladığı andaki DVL konumudur:
      dist_ref_x, dist_ref_y = pos_x, pos_y   (leg başlarken bir kere)
      distance_traveled = hypot(pos_x - dist_ref_x, pos_y - dist_ref_y)
  Neden gerekli: bir önceki leg "vardı" dendiğinde motor durur ama
  araç su içinde atalet yüzünden birkaç santim/metre daha kayabilir.
  Sayaç PLAN noktasından saymaya kalksa bu kaymayı "bedava ilerleme"
  sanıp erkenden "vardım" derdi. Gerçek başlangıç noktasından sayarak
  bu hata önlenir — araç gerçekten leg_length kadar yol almadan
  "vardım" demez.

ÖZETLE: hedef açı ve hedef mesafe = İDEAL (plan), ilerleme ÖLÇÜMÜ =
GERÇEK (DVL). İki farklı referans noktası, bilerek.

Kullanılan mevcut mimari:
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
    → TURN → STRAIGHT (leg 2) → DONE
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
    (4.0, -2.0),   # 1. waypoint  (başlangıçtan buraya)
    (2.0, 0.0),  # 2. waypoint  (1. waypoint'in PLANLANAN koordinatından buraya)
]

TARGET_DEPTH          = 0.60   # m — dalış hedefi
HEADING_TOLERANCE_DEG = 5.0    # bu toleransa girince dönüş tamamlandı sayılır
FORWARD_SPEED         = 0.6    # cmd_vel.linear.x oranı (düz giderken sabit)
ARRIVAL_MARGIN        = 0.30   # m — bu kadar yakına gelince "vardı" say
DEPTH_ARRIVAL_MARGIN  = 0.15
TURN_TIMEOUT_SEC      = 15.0   # dönüş bu süreyi geçerse yine de STRAIGHT'e geç
                                # (IMU/PID takılırsa sonsuz beklemesin diye emniyet)
SHUTDOWN_ON_FINISH    = False


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

        self.timer = self.create_timer(0.1, self._control_loop)  # 10 Hz

        self.get_logger().info('🚀 mission_straight_dvl başlatıldı (ideal geometri — bir kere dön, düz git)')
        self.get_logger().info(f'📍 Toplam {self.total_legs} waypoint: {self.waypoints}')

    # ── Callback'ler ──────────────────────────────────────────────────────
    def _pos_x_cb(self, msg: Float32):
        self.pos_x = msg.data

    def _pos_y_cb(self, msg: Float32):
        self.pos_y = msg.data

    def _depth_cb(self, msg: Float32):
        self.current_depth = msg.data

    def _raw_heading_cb(self, msg: Float32):
        # auv_controller.py'nin uyguladığı AYNI dönüşüm: current_heading = (360 - raw) % 360
        self.current_heading = (360.0 - msg.data) % 360.0

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
            # geometriden hesaplanıyor.
            self.leg_start_x, self.leg_start_y = self.planned_points[self.wp_index]
            dx = target_x - self.leg_start_x
            dy = target_y - self.leg_start_y
            # BİR KERE hesaplanıyor — bir daha güncellenmeyecek
            self.leg_target_heading = math.degrees(math.atan2(dy, dx)) % 360.0
            self.leg_length = math.hypot(dx, dy)

            # DÜZELTME (atalet/momentum sorunu): açı ve leg_length HÂLÂ ideal
            # plandan hesaplanıyor (yukarıda), ama mesafe SAYACININ referans
            # noktası artık planlanan koordinat DEĞİL — aracın bu leg'e
            # GERÇEKTEN başladığı andaki DVL konumu. Önceki leg bittiğinde
            # araç atalet yüzünden hedefi geçip kaymış olabilir; sayaç bunu
            # "bu leg'de kat edilen yol" sanıp anında "vardım" demesin diye
            # buradan sıfırlanıyor.
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
            self.wp_index += 1
            self.leg_initialized = False   # bir sonraki waypoint için sıfırla
            self._stop_cmd_vel()
            self.state = 'TURN'
            return

        if int(time.time() * 2) % 2 == 0:
            pct = 0.0 if self.leg_length <= 0 else min(100.0, 100.0 * distance_traveled / self.leg_length)
            self.get_logger().info(
                f'➡️  Leg {leg_no}/{self.total_legs} düz gidiyor: '
                f'kat edilen={distance_traveled:.2f}m kalan={remaining:.2f}m '
                f'(%{pct:.0f}) heading(sabit)={self.leg_target_heading:.1f}°')

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
