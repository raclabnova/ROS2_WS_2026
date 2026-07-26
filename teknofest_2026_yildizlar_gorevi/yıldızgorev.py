#!/usr/bin/env python3
"""
mission_straight_dvl.py — Açık Döngü (Open-Loop) DVL Waypoint Navigasyonu
                            + Her Waypoint Sonrası GERÇEK Yüzeye Çıkış
                              (dikey motorlar tamamen kapalı) / Konum Tut / Tekrar Dal
----------------------------------------------------------------------
YÖNTEM: Bir Kere Dön, Düz Git (Turn-Once-Then-Go)

Bu node yol boyunca AÇIYI YENİDEN HESAPLAMAZ. Sadece bir kere hesaplar,
IMU ile o açıya döner, sonra sabit o yönde gider. DVL SADECE "ne kadar
yol aldım" ölçmek için kullanılır — sapma düzeltmesi YOK (leg'in açı/
mesafe HEDEFİ için; bkz. aşağıda, ideal geometri).

Akış (2 waypoint, her birinden sonra GERÇEK yüzeye çık/tekrar dal):
  1) TURN: leg başlangıcındaki PLANLANAN (ideal) noktadan hedefe olan
     açı (atan2) BİR KERE hesaplanır, /auv/heading_setpoint'e basılır.
  2) STRAIGHT: heading sabit, cmd_vel ileri. Mesafe SAYACI bu leg'e
     GERÇEKTEN başladığı andaki DVL konumundan (dist_ref_x/y) ölçülür
     (atalet düzeltmesi). AÇI ve HEDEF MESAFE ise İKİ PLANLANAN NOKTA
     arasından (ideal geometri) hesaplanır — drift bu hesaba karışmaz.
  3) Waypoint'e varınca, sıradaki waypoint'e dönmeden ÖNCE:
       a) SURFACE_RISE — /auv/vertical_disable = True gönderilir.
          Bu, controller'daki DERİNLİK PID'İNİ TAMAMEN KAPATIR (M5-M8
          nötr/1500, hiçbir dikey itki yok) — araç kendi kaldırma
          kuvvetiyle SERBESTÇE yüzeye çıkar. Heading, o leg'in açısında
          SABİT tutulur (hiç değiştirilmez) ve cmd_vel sıfırdır — yatay
          motorlar (M1-M4) hâlâ çalışıyor, açı/yön yükselme sırasında
          bozulmuyor. Gerçek derinlik ~0'a (yüzey) yakın bir eşiğe
          düşünce ya da güvenlik zaman aşımı dolunca sıradaki adıma
          geçilir.
       b) SURFACE_HOLD — yüzeydeyken SURFACE_HOLD_SEC (2sn) beklenir.
          cmd_vel hâlâ sıfır, heading hâlâ AYNI açıda sabit — controller
          DVL ile o anki (x,y) konumunu ve açıyı koruyor.
       c) REDIVE — /auv/vertical_disable = False + /auv/depth_setpoint
          = TARGET_DEPTH (0.60) gönderilir. Heading YİNE AYNI açıda
          sabit (hiç değişmedi) — sanki controller ilk kez başlatılmış
          gibi AYNI KONUM ve AYNI AÇIDA tekrar dalış yapılır. Hedef
          derinliğe varınca: sırada waypoint kalmışsa TURN'e (ideal
          geometriyle sıradaki waypoint'in açısı/mesafesi hesaplanır),
          kalmamışsa DONE'a geçilir.

CONTROLLER NOTU (auv_controller.py — bu mission'la BİRLİKTE GÜNCELLENDİ,
ayrıca paylaşıldı, sensor_node.py İSE HİÇ DEĞİŞMEDİ):
  auv_controller.py'ye YENİ bir topic eklendi: /auv/vertical_disable
  (Bool). True olduğunda:
    - Derinlik PID'i TAMAMEN atlanır (M5-M8 nötr/1500, sıfır dikey itki).
    - Konum tutma (M1-M4), depth_hold_active YERİNE bu bayrağa da
      bakacak şekilde güncellendi — yani derinlik durumundan bağımsız
      olarak, sırf vertical_disable=True olduğu için çalışmaya devam
      ediyor. Bu sayede "dikey motorlar dursun ama yatay motorlar DVL
      ile hem konumu hem açıyı korusun" senaryosu mümkün oluyor.
    - Pitch/Roll PID de aynı şekilde bu bayrakla da tetikleniyor —
      araç serbest yüzeye çıkarken yamulmasın diye seviye/yatık
      düzeltmesi devam ediyor.
  Bu mission SADECE bu yeni topic'i (True/False) ve mevcut
  depth_setpoint/heading_setpoint/cmd_vel topic'lerini kullanıyor.

Kullanılan mevcut mimari:
  Subscribe: /auv/sensors/dvl_pos_x, /auv/sensors/dvl_pos_y,
             /auv/sensors/depth, /auv/sensors/heading (ham pusula),
             /auv/reference_heading
  Publish:   /auv/heading_setpoint, /auv/depth_setpoint, /auv/cmd_vel,
             /auv/manual_override, /auv/shutdown, /auv/vertical_disable (YENİ)

Görev akışı (state machine):
  WAIT_CALIBRATION → WAIT_DEPTH → TURN → STRAIGHT (leg 1)
    → SURFACE_RISE → SURFACE_HOLD → REDIVE
    → TURN → STRAIGHT (leg 2)
    → SURFACE_RISE → SURFACE_HOLD → REDIVE → DONE
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
import math
import time

# ── Görev parametreleri ──────────────────────────────────────────────────
WAYPOINTS = [
    (2.0, -2.0),   # 1. waypoint  (başlangıçtan buraya)
    (-1.0, -1.0),  # 2. waypoint  (1. waypoint'in PLANLANAN koordinatından buraya)
]

TARGET_DEPTH          = 0.60
HEADING_TOLERANCE_DEG = 5.0
FORWARD_SPEED         = 0.6
ARRIVAL_MARGIN        = 0.30
DEPTH_ARRIVAL_MARGIN  = 0.15
TURN_TIMEOUT_SEC      = 15.0
SHUTDOWN_ON_FINISH    = False

# ── GERÇEK YÜZEYE ÇIKIŞ / KONUM TUT / TEKRAR DAL parametreleri ──────────
SURFACE_ARRIVAL_DEPTH     = 0.05   # m — bu değerin altına inince "yüzeye çıktı" say
SURFACE_RISE_TIMEOUT_SEC  = 20.0   # s — bu süre geçerse yine de yüzeyde sayılır (emniyet)
SURFACE_HOLD_SEC          = 2.0    # s — yüzeyde DVL ile konum tutarak bekleme süresi


class MissionStraightDVL(Node):
    def __init__(self):
        super().__init__('mission_straight_dvl')

        self.heading_sp_pub = self.create_publisher(Float32, '/auv/heading_setpoint', 10)
        self.depth_sp_pub   = self.create_publisher(Float32, '/auv/depth_setpoint',   10)
        self.cmd_vel_pub    = self.create_publisher(Twist,   '/auv/cmd_vel',          10)
        self.override_pub   = self.create_publisher(Bool,    '/auv/manual_override',  10)
        self.shutdown_pub   = self.create_publisher(Bool,    '/auv/shutdown',         10)
        self.vertical_disable_pub = self.create_publisher(Bool, '/auv/vertical_disable', 10)

        self.create_subscription(Float32, '/auv/sensors/dvl_pos_x', self._pos_x_cb, 10)
        self.create_subscription(Float32, '/auv/sensors/dvl_pos_y', self._pos_y_cb, 10)
        self.create_subscription(Float32, '/auv/sensors/depth',     self._depth_cb, 10)
        self.create_subscription(Float32, '/auv/sensors/heading',   self._raw_heading_cb, 10)
        self.create_subscription(Float32, '/auv/reference_heading', self._ref_heading_cb, 10)

        self.pos_x = None
        self.pos_y = None
        self.current_depth = 0.0
        self.current_heading = None
        self.controller_calibrated = False

        self.waypoints = list(WAYPOINTS)
        self.total_legs = len(self.waypoints)
        self.wp_index = 0
        self.state = 'WAIT_CALIBRATION'

        self.planned_points = [(0.0, 0.0)] + list(self.waypoints)

        self.leg_start_x        = 0.0
        self.leg_start_y        = 0.0
        self.leg_target_heading = 0.0
        self.leg_length         = 0.0
        self.leg_initialized    = False
        self.turn_start_time    = None

        self.dist_ref_x = 0.0
        self.dist_ref_y = 0.0

        self.surface_rise_start_time = None
        self.surface_hold_start_time = None

        self.timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info('🚀 mission_straight_dvl başlatıldı (ideal geometri + GERÇEK yüzeye çık/konum tut/tekrar dal)')
        self.get_logger().info(f'📍 Toplam {self.total_legs} waypoint: {self.waypoints}')
        self.get_logger().info(
            f'🌊 Her waypoint sonrası: dikey motorlar kapatılıp yüzeye çıkış, '
            f'{SURFACE_HOLD_SEC:.1f}s konum tutma, {TARGET_DEPTH:.2f}m\'e tekrar dalış')

    def _pos_x_cb(self, msg: Float32):
        self.pos_x = msg.data

    def _pos_y_cb(self, msg: Float32):
        self.pos_y = msg.data

    def _depth_cb(self, msg: Float32):
        self.current_depth = msg.data

    def _raw_heading_cb(self, msg: Float32):
        self.current_heading = (360.0 - msg.data) % 360.0

    def _ref_heading_cb(self, msg: Float32):
        self.controller_calibrated = True

    def _control_loop(self):
        if self.state == 'WAIT_CALIBRATION':
            self._publish_override(True)
            if self.controller_calibrated:
                self.get_logger().info('✅ Controller kalibre oldu, dalışa geçiliyor')
                self.state = 'WAIT_DEPTH'
            return

        if self.state == 'WAIT_DEPTH':
            self._publish_override(False)
            self._publish_vertical_disable(False)
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

        if self.state == 'SURFACE_RISE':
            self._surface_rise_step()
            return

        if self.state == 'SURFACE_HOLD':
            self._surface_hold_step()
            return

        if self.state == 'REDIVE':
            self._redive_step()
            return

        if self.state == 'DONE':
            self._stop_cmd_vel()
            return

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
            self.leg_start_x, self.leg_start_y = self.planned_points[self.wp_index]
            dx = target_x - self.leg_start_x
            dy = target_y - self.leg_start_y
            self.leg_target_heading = math.degrees(math.atan2(dy, dx)) % 360.0
            self.leg_length = math.hypot(dx, dy)

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

    def _straight_step(self):
        heading_msg = Float32(); heading_msg.data = float(self.leg_target_heading)
        self.heading_sp_pub.publish(heading_msg)

        cmd = Twist()
        cmd.linear.x = float(FORWARD_SPEED)
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

        dx = self.pos_x - self.dist_ref_x
        dy = self.pos_y - self.dist_ref_y
        distance_traveled = math.hypot(dx, dy)
        remaining = self.leg_length - distance_traveled
        leg_no = self.wp_index + 1

        if remaining <= ARRIVAL_MARGIN:
            self.get_logger().info(
                f'✅ Leg {leg_no}/{self.total_legs} — waypoint {self.wp_index} ulaşıldı — '
                f'kat edilen={distance_traveled:.2f}m / hedef={self.leg_length:.2f}m')

            self.wp_index += 1
            self.leg_initialized = False
            self._stop_cmd_vel()

            self._start_surface_sequence()
            return

        if int(time.time() * 2) % 2 == 0:
            pct = 0.0 if self.leg_length <= 0 else min(100.0, 100.0 * distance_traveled / self.leg_length)
            self.get_logger().info(
                f'➡️  Leg {leg_no}/{self.total_legs} düz gidiyor: '
                f'kat edilen={distance_traveled:.2f}m kalan={remaining:.2f}m '
                f'(%{pct:.0f}) heading(sabit)={self.leg_target_heading:.1f}°')

    def _start_surface_sequence(self):
        self.state = 'SURFACE_RISE'
        self.surface_rise_start_time = time.time()
        self._publish_vertical_disable(True)
        self.get_logger().info('🚁 Dikey motorlar kapatıldı — araç serbestçe yüzeye çıkıyor...')

    def _surface_rise_step(self):
        # Heading AYNI leg'in açısında SABİT — yükselirken yön bozulmuyor.
        heading_msg = Float32(); heading_msg.data = float(self.leg_target_heading)
        self.heading_sp_pub.publish(heading_msg)
        self._publish_vertical_disable(True)
        self._stop_cmd_vel()

        elapsed = time.time() - self.surface_rise_start_time

        if int(time.time() * 2) % 2 == 0:
            self.get_logger().info(
                f'🚁 Yüzeye çıkılıyor: derinlik={self.current_depth:.2f}m '
                f'(hedef ≤{SURFACE_ARRIVAL_DEPTH:.2f}m) elapsed={elapsed:.1f}s')

        arrived = self.current_depth <= SURFACE_ARRIVAL_DEPTH
        timeout = elapsed >= SURFACE_RISE_TIMEOUT_SEC

        if arrived or timeout:
            if timeout and not arrived:
                self.get_logger().warn(
                    f'⚠️ Yüzeye çıkış zaman aşımına uğradı! (derinlik={self.current_depth:.2f}m, '
                    f'{SURFACE_RISE_TIMEOUT_SEC:.0f}s doldu) Yine de devam ediliyor.')
            else:
                self.get_logger().info(f'✅ Yüzeye çıkıldı ({self.current_depth:.2f}m) — konum tutuluyor')
            self.surface_hold_start_time = time.time()
            self.state = 'SURFACE_HOLD'

    def _surface_hold_step(self):
        heading_msg = Float32(); heading_msg.data = float(self.leg_target_heading)
        self.heading_sp_pub.publish(heading_msg)
        self._publish_vertical_disable(True)
        self._stop_cmd_vel()

        elapsed = time.time() - self.surface_hold_start_time
        if int(time.time() * 2) % 2 == 0:
            self.get_logger().info(f'🌊 Yüzeyde konum tutuluyor: {elapsed:.1f}/{SURFACE_HOLD_SEC:.1f}s')

        if elapsed >= SURFACE_HOLD_SEC:
            self.get_logger().info(f'✅ Konum tutma bitti — {TARGET_DEPTH:.2f}m\'e tekrar dalınıyor')
            self.state = 'REDIVE'
            self._publish_vertical_disable(False)
            depth_sp = Float32(); depth_sp.data = float(TARGET_DEPTH)
            self.depth_sp_pub.publish(depth_sp)

    def _redive_step(self):
        # Heading YİNE AYNI leg'in açısında SABİT — sanki controller ilk
        # kez başlatılmış gibi aynı açıda tekrar dalış.
        heading_msg = Float32(); heading_msg.data = float(self.leg_target_heading)
        self.heading_sp_pub.publish(heading_msg)
        self._publish_vertical_disable(False)
        depth_sp = Float32(); depth_sp.data = float(TARGET_DEPTH)
        self.depth_sp_pub.publish(depth_sp)
        self._stop_cmd_vel()

        if int(time.time() * 2) % 2 == 0:
            self.get_logger().info(
                f'🌊 Tekrar dalınıyor: derinlik={self.current_depth:.2f}m '
                f'hedef={TARGET_DEPTH:.2f}m')

        if self.current_depth >= (TARGET_DEPTH - DEPTH_ARRIVAL_MARGIN):
            self.get_logger().info(f'✅ Hedef derinliğe tekrar ulaşıldı ({self.current_depth:.2f}m)')
            if self.wp_index >= len(self.waypoints):
                self.get_logger().info("🏁 Tüm waypoint'ler ve yüzey rutinleri tamamlandı")
                self.state = 'DONE'
                if SHUTDOWN_ON_FINISH:
                    sd = Bool(); sd.data = True
                    self.shutdown_pub.publish(sd)
            else:
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

    def _publish_vertical_disable(self, value: bool):
        msg = Bool(); msg.data = value
        self.vertical_disable_pub.publish(msg)

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
