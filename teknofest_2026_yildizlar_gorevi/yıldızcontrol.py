#!/usr/bin/env python3
"""
AUV Controller — DVL TABANLI KONUM TUTMA (STATION KEEPING) + ROTA TAKİBİ
                  + YÜZEYE ÇIKIŞ İÇİN DİKEY MOTOR DEVRE DIŞI BIRAKMA (YENİ)

YENİ EKLENEN (Akıntı Telafisi):
  Controller artık DVL'in dead-reckoning konumunu (/auv/sensors/dvl_pos_x,
  /auv/sensors/dvl_pos_y) ve DVL yaw'ını (/auv/sensors/dvl_yaw) dinliyor.

  İKİ ÇALIŞMA MODU:

  A) DURUYORKEN (lin_x = lin_y = ang_z = 0)  →  KONUM TUTMA
     Araç derinliğe oturduğu anda o noktanın DVL koordinatı referans
     olarak kilitlenir. Akıntı aracı sürüklerse, sapma vektörü gövde
     çerçevesine döndürülür ve hem yengeç (sway) hem ileri/geri (surge)
     düzeltmesi basılarak araç o noktaya geri oturur.

  B) İLERİ GİDERKEN (lin_x != 0)  →  ROTA (CROSS-TRACK) TAKİBİ
     Hareket başladığı anda, başlangıç noktasından aracın baktığı yön
     boyunca sanal bir HAT tanımlanır. Sapmanın hat boyunca olan bileşeni
     (along-track) yok sayılır — çünkü ileri gitmesi zaten isteniyor.
     Sadece hatta DİK olan bileşen (cross-track) düzeltilir. Akıntı
     aracı sağa X metre sürüklediyse, sola X metrelik yengeç basılıp
     araç hattın üstüne geri oturur → gerçekten dümdüz gider.

  C) DÖNERKEN (ang_z != 0, circle_mode, veya bilerek yengeç komutu varsa)
     Konum düzeltmesi pasif, referans sürekli yenilenir. Dönüş bitince
     yeni hat aracın yeni yönü boyunca oradan başlar.

  Kapatma: /auv/pos_hold (Bool, False = konum tutma kapalı).
  manual_override zaten tüm controller çıkışını susturuyor.

  DEBUG: /auv/debug/pos_err_fwd  ve  /auv/debug/pos_err_right
         (metre cinsinden, gövde çerçevesinde sapma — tuning için izle)

YENİ (VERTICAL DISABLE — dikey motorları tamamen devre dışı bırakma):
  /auv/vertical_disable (Bool) topic'i eklendi.
  True olduğunda:
    - Derinlik PID'i TAMAMEN atlanır — M5-M8'e hiçbir dikey itki
      eklenmez (base_v her zaman nötr 1500). Araç artık aktif olarak
      bir derinliğe tutulmuyor; kendi kaldırma kuvvetiyle serbestçe
      yükselir/batar (örn. yüzeye çıkış senaryosu — torpido gövdeli
      araçlarda üst gövde su yüzeyine değse bile DVL genelde hâlâ
      su altında kalır, ölçüme devam eder).
    - Pitch/Roll PID (araç seviyesini korumak için) HÂLÂ ÇALIŞIR —
      bu sadece diferansiyel bir düzeltme, dikey itki (heave) değil,
      o yüzden serbest yükselirken aracın yamulmasını önlemeye devam
      eder.
    - KONUM TUTMA (M1-M4, yatay motorlar) NORMALDE depth_hold_active
      bayrağına bağlıydı (yani "hedef derinliğe ulaştım" durumuna).
      vertical_disable=True iken bu şart ATLANIR — konum tutma,
      derinlik durumundan bağımsız olarak, sadece bu bayrak True
      olduğu için çalışmaya devam eder. Yani "dikey motorlar dursun
      ama yatay motorlar DVL ile konumu korusun" senaryosu bu şekilde
      mümkün oluyor.
  False'a dönünce: derinlik PID'i normal şekilde devam eder (mission
  hangi depth_setpoint'i basıyorsa ona doğru dalar/çıkar), konum
  referansı da o anki DVL konumuna göre tazelenir (ani sıçrama olmasın
  diye).

ÖNCEKİ ÖZELLİKLER KORUNDU:
  - Açı telafisi/offset yok, ham kalibrasyon açısı referans.
  - Pusula okuması (360 - msg) % 360 ile CCW'ye çevriliyor.
  - Manuel override, circle_mode, depth hold, pitch/roll PID aynı.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray, Bool
from geometry_msgs.msg import Twist
import time
import sys
import math

PITCH_SIGN   = 1.0   # ters düzeltiyorsa -1.0 yap
ROLL_SIGN    = -1.0  # ters düzeltiyorsa -1.0 yap
CIRCLE_PWM_H = 1600  # daire modu yatay motor yüksek PWM
CIRCLE_PWM_L = 1400  # daire modu yatay motor düşük PWM

# Heading setpoint sıçrama eşiği
HEADING_JUMP_RESET_DEG = 5.0

# Derinlik PID'in motor bütçesini yemesini önleyen sınırlar
DEPTH_PID_MAX   = 400.0   # pitch/roll'a her zaman pay bırakır
DEPTH_DERIV_MAX = 1.0     # m/s — türev teriminde ani sıçrama/kick sınırı

# ==================================================================== #
#  🔧 AYAR BLOĞU — SADECE BURAYI DEĞİŞTİR
#  Dosyayı kaydet, node'u yeniden başlat. Terminalde parametre gerekmez.
# ==================================================================== #

# ---- İŞARETLER (bir kere doğrula, sonra dokunma) ----
STRAFE_SIGN  = -1.0   # yengeç ters yöne gidiyorsa çevir  (senin araçta -1.0)
SURGE_SIGN   =  1.0   # ileri/geri düzeltmesi ters ise çevir
DVL_VY_SIGN  =  1.0   # aracı SAĞA iterken dvl_vy negatif çıkıyorsa -1.0 yap
DVL_VX_SIGN  =  1.0   # aracı İLERİ iterken dvl_vx negatif çıkıyorsa -1.0 yap

# ---- TEPKİ HIZI (asıl oynayacağın yer) ----
STRAFE_MIN_PWM = 30.0
Y_KP           = 900.0
Y_KI           =  40.0
Y_KD           = 250.0

# ---- İLERİ/GERİ ----
SURGE_MIN_PWM = 30.0
X_KP          = 600.0
X_KI          =  30.0
X_KD          = 200.0

# ---- SINIRLAR ----
POS_DEADBAND_M = 0.03
STRAFE_MAX     = 400.0
SURGE_MAX      = 250.0

# ---- MONTAJ ----
DVL_MOUNT_YAW_DEG = 0.0
USE_DVL_YAW = True
HEADING_TO_DVL_SIGN = -1.0

POS_I_MAX         = 2.0
DVL_TIMEOUT_S     = 1.0
DVL_VEL_TIMEOUT_S = 0.5
DEADZONE_MIN_CMD  = 5.0


def _clamp(v, lim):
    return max(min(v, lim), -lim)


class AUVController(Node):
    def __init__(self):
        super().__init__('auv_controller')

        self.declare_parameter('base_speed',        300)
        self.declare_parameter('target_depth',      0.60)
        self.declare_parameter('ref_samples',       20)
        self.declare_parameter('depth_hold_margin', 0.20)
        self.declare_parameter('pos_hold',          True)

        self.base_speed         = self.get_parameter('base_speed').value
        self.target_depth       = self.get_parameter('target_depth').value
        self.ref_sample_count   = self.get_parameter('ref_samples').value
        self.depth_hold_margin  = self.get_parameter('depth_hold_margin').value
        self.pos_hold_enabled   = self.get_parameter('pos_hold').value

        self.strafe_sign    = STRAFE_SIGN
        self.surge_sign     = SURGE_SIGN
        self.dvl_vy_sign    = DVL_VY_SIGN
        self.dvl_vx_sign    = DVL_VX_SIGN
        self.pos_deadband   = POS_DEADBAND_M
        self.strafe_min_pwm = STRAFE_MIN_PWM
        self.surge_min_pwm  = SURGE_MIN_PWM
        self.strafe_max     = STRAFE_MAX
        self.surge_max      = SURGE_MAX

        self.motor_pub = self.create_publisher(
            Int32MultiArray, '/auv/motor_pwm', 10)
        self.ref_heading_pub = self.create_publisher(
            Float32, '/auv/reference_heading', 10)
        self.err_fwd_pub = self.create_publisher(
            Float32, '/auv/debug/pos_err_fwd', 10)
        self.err_right_pub = self.create_publisher(
            Float32, '/auv/debug/pos_err_right', 10)

        self.create_subscription(Float32, '/auv/sensors/heading',  self._heading_cb,     10)
        self.create_subscription(Float32, '/auv/sensors/pitch',    self._pitch_cb,       10)
        self.create_subscription(Float32, '/auv/sensors/roll',     self._roll_cb,        10)
        self.create_subscription(Float32, '/auv/sensors/depth',    self._depth_cb,       10)
        self.create_subscription(Twist,   '/auv/cmd_vel',          self._cmd_vel_cb,     10)
        self.create_subscription(Float32, '/auv/depth_setpoint',   self._depth_sp_cb,    10)
        self.create_subscription(Float32, '/auv/heading_setpoint', self._heading_sp_cb,  10)
        self.create_subscription(Bool,    '/auv/circle_mode',      self._circle_mode_cb, 10)
        self.create_subscription(Bool,    '/auv/shutdown',         self._shutdown_cb,    10)
        self.create_subscription(Bool,    '/auv/manual_override',  self._manual_override_cb, 10)
        self.create_subscription(Float32, '/auv/sensors/dvl_pos_x', self._dvl_x_cb,   10)
        self.create_subscription(Float32, '/auv/sensors/dvl_pos_y', self._dvl_y_cb,   10)
        self.create_subscription(Float32, '/auv/sensors/dvl_yaw',   self._dvl_yaw_cb, 10)
        self.create_subscription(Float32, '/auv/sensors/dvl_vx',    self._dvl_vx_cb,  10)
        self.create_subscription(Float32, '/auv/sensors/dvl_vy',    self._dvl_vy_cb,  10)
        self.create_subscription(Bool,    '/auv/pos_hold',          self._pos_hold_cb, 10)
        # YENİ — dikey motorları tamamen devre dışı bırakma (yüzeye çıkış)
        self.create_subscription(Bool,    '/auv/vertical_disable',  self._vertical_disable_cb, 10)

        self.current_heading   = 0.0
        self.reference_heading = None
        self._ref_samples      = []
        self.calibrated        = False

        self.current_depth     = 0.0
        self.depth_hold_active = False
        self.circle_mode       = False

        self.manual_override   = False
        self.vertical_disable  = False   # YENİ

        self.current_pitch     = 0.0
        self.current_roll      = 0.0
        self.reference_pitch   = None
        self.reference_roll    = None
        self._ref_p_samples    = []
        self._ref_r_samples    = []

        self.lin_x = 0.0
        self.ang_z = 0.0
        self.lin_y = 0.0
        self._prev_ang_z = 0.0

        self.prev_time = time.time()

        self.dvl_x   = 0.0
        self.dvl_y   = 0.0
        self.dvl_yaw = 0.0
        self.dvl_vx  = 0.0
        self.dvl_vy  = 0.0
        self.dvl_last_t     = 0.0
        self.dvl_vel_last_t = 0.0
        self.dvl_ready  = False
        self._dvl_yaw_seen = False
        self._heading_at_dvl_origin = None

        self.ref_x   = 0.0
        self.ref_y   = 0.0
        self.ref_yaw = 0.0
        self.pos_ref_set   = False
        self._prev_pos_mode = None

        self.h_Kp       = 9.0
        self.h_Ki       = 0.5
        self.h_Kd       = 1.0
        self.h_integral = 0.0
        self.h_prev_err = 0.0

        self.d_Kp       = 900.0
        self.d_Ki       = 5.0
        self.d_Kd       = 200.0
        self.d_integral = 0.0
        self.d_prev_err = 0.0

        self.p_Kp       = 6.0
        self.p_Ki       = 0.0
        self.p_Kd       = 1.5
        self.p_integral = 0.0
        self.p_prev_err = 0.0

        self.r_Kp       = 2.0
        self.r_Ki       = 0.5
        self.r_Kd       = 1.0
        self.r_integral = 0.0
        self.r_prev_err = 0.0

        self.y_Kp = Y_KP
        self.y_Ki = Y_KI
        self.y_Kd = Y_KD
        self.y_integral = 0.0

        self.x_Kp = X_KP
        self.x_Ki = X_KI
        self.x_Kd = X_KD
        self.x_integral = 0.0

        self.timer = self.create_timer(0.02, self._control_loop)

        self.get_logger().info('🚀 AUV Controller başlatıldı (DVL KONUM TUTMA + VERTICAL DISABLE)')
        self.get_logger().info('⚙️  Açı telafisi/offset YOK — ham kalibrasyon açısı doğrudan referans alınacak')
        self.get_logger().info(f'🌊 Akıntı telafisi: {"AÇIK" if self.pos_hold_enabled else "KAPALI"}')
        self.get_logger().info(f'⏳ Kalibrasyon bekleniyor ({self.ref_sample_count} örnek)...')

    def _heading_cb(self, msg: Float32):
        self.current_heading = (360.0 - msg.data) % 360.0
        if not self.calibrated:
            self._ref_samples.append(self.current_heading)
            if len(self._ref_samples) % 5 == 0:
                self.get_logger().info(
                    f'📐 Kalibrasyon: {len(self._ref_samples)}/{self.ref_sample_count}')
            if len(self._ref_samples) >= self.ref_sample_count:
                raw_reference = self._circular_mean(self._ref_samples)
                self.reference_heading = raw_reference
                self.calibrated = True
                self.get_logger().info(
                    f'✅ Kalibrasyon bitti. Kilitlenen Yön: {self.reference_heading:.1f}°')
                self._publish_reference()

    def _pitch_cb(self, msg: Float32):
        self.current_pitch = msg.data
        if self.reference_pitch is None:
            self._ref_p_samples.append(msg.data)
            if len(self._ref_p_samples) >= self.ref_sample_count:
                self.reference_pitch = sum(self._ref_p_samples) / len(self._ref_p_samples)

    def _roll_cb(self, msg: Float32):
        self.current_roll = msg.data
        if self.reference_roll is None:
            self._ref_r_samples.append(msg.data)
            if len(self._ref_r_samples) >= self.ref_sample_count:
                self.reference_roll = sum(self._ref_r_samples) / len(self._ref_r_samples)

    def _depth_cb(self, msg: Float32):
        self.current_depth = msg.data
        if (self.calibrated
                and not self.depth_hold_active
                and self.current_depth >= (self.target_depth - self.depth_hold_margin)):
            self.depth_hold_active = True
            self._capture_pos_ref()
            self._reset_pos_pid()
            self.get_logger().info(
                f'🔒 Depth hold aktif — {self.current_depth:.2f}m  '
                f'Konum kilitlendi: ({self.ref_x:.2f}, {self.ref_y:.2f})')

    def _cmd_vel_cb(self, msg: Twist):
        self.lin_x = msg.linear.x
        self.ang_z = msg.angular.z
        self.lin_y = msg.linear.y

    def _depth_sp_cb(self, msg: Float32):
        self.target_depth      = msg.data
        self.depth_hold_active = False

    def _heading_sp_cb(self, msg: Float32):
        new_ref = msg.data % 360.0
        if self.reference_heading is None:
            self.reference_heading = new_ref
            self.h_integral        = 0.0
            self.h_prev_err        = 0.0
            self._publish_reference()
            return
        jump = abs(self._heading_error(self.current_heading, new_ref))
        self.reference_heading = new_ref
        if jump > HEADING_JUMP_RESET_DEG:
            self.h_integral = 0.0
            self.h_prev_err = 0.0
        self._publish_reference()

    def _circle_mode_cb(self, msg: Bool):
        self.circle_mode = msg.data
        if self.circle_mode:
            self.h_integral = 0.0
            self.h_prev_err = 0.0

    def _shutdown_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().info('🛑 Kapatma komutu alındı! Controller kapatılıyor...')
            self.stop_motors()
            sys.exit(0)

    def _manual_override_cb(self, msg: Bool):
        self.manual_override = msg.data
        if self.manual_override:
            self.get_logger().info('🖐️ MANUEL OVERRIDE AKTİF')
        else:
            self.get_logger().info('🤖 MANUEL OVERRIDE KAPALI — controller tekrar devrede')
            self.h_integral = 0.0; self.h_prev_err = 0.0
            self.p_integral = 0.0; self.p_prev_err = 0.0
            self.r_integral = 0.0; self.r_prev_err = 0.0
            self._capture_pos_ref()
            self._reset_pos_pid()

    # YENİ — dikey motorları tamamen devre dışı bırakma callback'i
    def _vertical_disable_cb(self, msg: Bool):
        was = self.vertical_disable
        self.vertical_disable = msg.data
        if self.vertical_disable and not was:
            self.get_logger().info(
                '🚁 DİKEY MOTORLAR DEVRE DIŞI — araç serbest yükseliyor/batıyor '
                '(konum tutma derinlikten BAĞIMSIZ devam ediyor)')
            self.d_integral = 0.0
            self.d_prev_err = 0.0
            self._capture_pos_ref()
            self._reset_pos_pid()
        elif (not self.vertical_disable) and was:
            self.get_logger().info('🌊 DİKEY MOTORLAR TEKRAR AKTİF — normal derinlik kontrolü')
            self.d_integral = 0.0
            self.d_prev_err = 0.0
            self._capture_pos_ref()
            self._reset_pos_pid()

    def _dvl_x_cb(self, msg: Float32):
        self.dvl_x = msg.data
        self.dvl_last_t = time.time()
        if not self.dvl_ready:
            self.dvl_ready = True
            self._heading_at_dvl_origin = self.current_heading
            self.get_logger().info('📡 DVL konum verisi alındı — akıntı telafisi hazır')

    def _dvl_y_cb(self, msg: Float32):
        self.dvl_y = msg.data
        self.dvl_last_t = time.time()

    def _dvl_yaw_cb(self, msg: Float32):
        self.dvl_yaw = msg.data
        self._dvl_yaw_seen = True

    def _dvl_vx_cb(self, msg: Float32):
        self.dvl_vx = msg.data
        self.dvl_vel_last_t = time.time()

    def _dvl_vy_cb(self, msg: Float32):
        self.dvl_vy = msg.data
        self.dvl_vel_last_t = time.time()

    def _pos_hold_cb(self, msg: Bool):
        self.pos_hold_enabled = msg.data
        self._capture_pos_ref()
        self._reset_pos_pid()

    def _frame_yaw_deg(self):
        if USE_DVL_YAW and self._dvl_yaw_seen:
            psi = self.dvl_yaw
        elif self._heading_at_dvl_origin is not None:
            psi = HEADING_TO_DVL_SIGN * (self.current_heading - self._heading_at_dvl_origin)
        else:
            psi = 0.0
        return psi - DVL_MOUNT_YAW_DEG

    def _capture_pos_ref(self):
        self.ref_x   = self.dvl_x
        self.ref_y   = self.dvl_y
        self.ref_yaw = self._frame_yaw_deg()
        self.pos_ref_set = True

    def _reset_pos_pid(self):
        self.x_integral = 0.0
        self.y_integral = 0.0

    @staticmethod
    def _apply_deadzone(cmd, min_pwm):
        if abs(cmd) < DEADZONE_MIN_CMD:
            return 0.0
        return cmd + math.copysign(min_pwm, cmd)

    def _position_control(self, dt):
        """
        YENİ: artık depth_hold_active VEYA vertical_disable'a bakıyor —
        dikey motorlar devre dışı bırakıldığında (yüzeye çıkış) konum
        tutma, derinlik durumundan bağımsız olarak çalışmaya devam eder.
        """
        if not self.pos_hold_enabled or not self.dvl_ready:
            self._reset_pos_pid()
            return 0.0, 0.0

        if (time.time() - self.dvl_last_t) > DVL_TIMEOUT_S:
            self._reset_pos_pid()
            self.get_logger().warn(
                'DVL verisi yok (dip kilidi kayıp?) — akıntı telafisi pasif',
                throttle_duration_sec=3.0)
            return 0.0, 0.0

        if not (self.depth_hold_active or self.vertical_disable):
            self._capture_pos_ref()
            self._reset_pos_pid()
            return 0.0, 0.0

        moving   = abs(self.lin_x) > 1e-3
        strafing = abs(self.lin_y) > 1e-3
        turning  = (abs(self.ang_z) > 1e-3) or self.circle_mode

        mode = (moving, strafing, turning)
        if (mode != self._prev_pos_mode) or (not self.pos_ref_set):
            self._capture_pos_ref()
            self._reset_pos_pid()
            self._prev_pos_mode = mode

        if turning or strafing:
            self._capture_pos_ref()
            return 0.0, 0.0

        dx = self.dvl_x - self.ref_x
        dy = self.dvl_y - self.ref_y

        if moving:
            lpsi = math.radians(self.ref_yaw)
            lx, ly = math.cos(lpsi), math.sin(lpsi)
            along = dx * lx + dy * ly
            dx -= along * lx
            dy -= along * ly

        ex, ey = -dx, -dy
        cpsi = math.radians(self._frame_yaw_deg())
        c, s = math.cos(cpsi), math.sin(cpsi)
        err_fwd   =  ex * c + ey * s
        err_right = -ex * s + ey * c

        self._publish_pos_error(err_fwd, err_right)

        vel_fresh = (time.time() - self.dvl_vel_last_t) < DVL_VEL_TIMEOUT_S
        v_right = self.dvl_vy_sign * self.dvl_vy if vel_fresh else 0.0
        v_fwd   = self.dvl_vx_sign * self.dvl_vx if vel_fresh else 0.0

        e = err_right
        if abs(e) < self.pos_deadband:
            e = 0.0
            self.y_integral = 0.0
        self.y_integral = _clamp(self.y_integral + e * dt, POS_I_MAX)
        raw = (self.y_Kp * e + self.y_Ki * self.y_integral - self.y_Kd * v_right)
        raw = self._apply_deadzone(raw, self.strafe_min_pwm)
        strafe_corr = self.strafe_sign * _clamp(raw, self.strafe_max)

        e = err_fwd
        if abs(e) < self.pos_deadband:
            e = 0.0
            self.x_integral = 0.0
        self.x_integral = _clamp(self.x_integral + e * dt, POS_I_MAX)
        raw = (self.x_Kp * e + self.x_Ki * self.x_integral - self.x_Kd * v_fwd)
        raw = self._apply_deadzone(raw, self.surge_min_pwm)
        surge_corr = self.surge_sign * _clamp(raw, self.surge_max)

        return surge_corr, strafe_corr

    def _publish_pos_error(self, fwd, right):
        m = Float32(); m.data = float(fwd);   self.err_fwd_pub.publish(m)
        m = Float32(); m.data = float(right); self.err_right_pub.publish(m)

    def _control_loop(self):
        if not self.calibrated:
            return
        if self.manual_override:
            return

        self._publish_reference()

        now = time.time()
        dt  = now - self.prev_time
        self.prev_time = now
        if dt < 0.001: dt = 0.02

        # 1. HEADING PID
        if self.circle_mode:
            pid_out = 0
        elif self.ang_z != 0.0:
            pid_out = int(self.ang_z * self.base_speed)
            self.h_integral = 0.0
            self.h_prev_err = 0.0
        elif self._prev_ang_z != 0.0 and self.ang_z == 0.0:
            self.reference_heading = self.current_heading
            self.h_integral        = 0.0
            self.h_prev_err        = 0.0
            pid_out = 0
            self._publish_reference()
            self.get_logger().info(f'🔒 Yaw bitti — yeni referans: {self.reference_heading:.1f}°')
        elif self.depth_hold_active or self.vertical_disable:
            err = self._heading_error(self.current_heading, self.reference_heading)
            self.h_integral += err * dt
            self.h_integral  = max(min(self.h_integral, 60.0), -60.0)
            derivative      = (err - self.h_prev_err) / dt
            self.h_prev_err = err
            pid_out = (self.h_Kp * err + self.h_Ki * self.h_integral + self.h_Kd * derivative)
            pid_out = max(min(pid_out, 200.0), -200.0)
        else:
            pid_out = 0

        self._prev_ang_z = self.ang_z

        # 2. DEPTH PID (vertical_disable iken TAMAMEN atlanır)
        if self.vertical_disable:
            depth_pid       = 0.0
            self.d_integral = 0.0
            self.d_prev_err = 0.0
            base_v = 1500
        else:
            depth_err = self.target_depth - self.current_depth
            if abs(depth_err) < 0.05:
                depth_err       = 0.0
                self.d_integral = 0.0
            self.d_integral += depth_err * dt
            self.d_integral  = max(min(self.d_integral, 10.0), -10.0)
            deriv_d = (depth_err - self.d_prev_err) / dt
            deriv_d = max(min(deriv_d, DEPTH_DERIV_MAX), -DEPTH_DERIV_MAX)
            self.d_prev_err = depth_err
            depth_pid = (self.d_Kp * depth_err + self.d_Ki * self.d_integral + self.d_Kd * deriv_d)
            depth_pid = max(min(depth_pid, DEPTH_PID_MAX), -DEPTH_PID_MAX)
            base_v    = 1500 + int(depth_pid)

        # 3. PITCH & ROLL PID
        pid_pitch = 0.0
        if (self.depth_hold_active or self.vertical_disable) and self.reference_pitch is not None:
            err_p = self.reference_pitch - self.current_pitch
            self.p_integral += err_p * dt
            self.p_integral  = max(min(self.p_integral, 60.0), -60.0)
            deriv_p         = (err_p - self.p_prev_err) / dt
            self.p_prev_err = err_p
            pid_pitch = PITCH_SIGN * (self.p_Kp * err_p + self.p_Ki * self.p_integral + self.p_Kd * deriv_p)
            pid_pitch = max(min(pid_pitch, 300.0), -300.0)
        else:
            self.p_integral = 0.0; self.p_prev_err = 0.0

        pid_roll = 0.0
        if (self.depth_hold_active or self.vertical_disable) and self.reference_roll is not None:
            err_r = self.reference_roll - self.current_roll
            self.r_integral += err_r * dt
            self.r_integral  = max(min(self.r_integral, 60.0), -60.0)
            deriv_r         = (err_r - self.r_prev_err) / dt
            self.r_prev_err = err_r
            pid_roll = ROLL_SIGN * (self.r_Kp * err_r + self.r_Ki * self.r_integral + self.r_Kd * deriv_r)
            pid_roll = max(min(pid_roll, 300.0), -300.0)
        else:
            self.r_integral = 0.0; self.r_prev_err = 0.0

        # 3.5 DVL KONUM / AKINTI TELAFİSİ
        surge_corr, strafe_corr = self._position_control(dt)

        # 4. MOTOR KOMUTLARI
        pwm_m5 = max(min(base_v - int(pid_pitch) + int(pid_roll), 1900), 1100)
        pwm_m6 = max(min(base_v + int(pid_pitch) + int(pid_roll), 1900), 1100)
        pwm_m7 = max(min(base_v - int(pid_pitch) - int(pid_roll), 1900), 1100)
        pwm_m8 = max(min(base_v + int(pid_pitch) - int(pid_roll), 1900), 1100)

        msg = Int32MultiArray()

        if self.circle_mode:
            msg.data = [CIRCLE_PWM_H, CIRCLE_PWM_L, CIRCLE_PWM_L, CIRCLE_PWM_H,
                        pwm_m5, pwm_m6, pwm_m7, pwm_m8]
        else:
            base_x = int(self.lin_x * self.base_speed + surge_corr)
            strafe = int(self.lin_y * self.base_speed + strafe_corr)
            yaw    = int(pid_out)

            m1 = max(min(1500 + base_x + strafe + yaw, 1900), 1100)
            m2 = max(min(1500 + base_x - strafe - yaw, 1900), 1100)
            m3 = max(min(1500 + base_x - strafe + yaw, 1900), 1100)
            m4 = max(min(1500 + base_x + strafe - yaw, 1900), 1100)

            msg.data = [m1, m2, m3, m4, pwm_m5, pwm_m6, pwm_m7, pwm_m8]

        self.motor_pub.publish(msg)

        if int(now * 2) % 2 == 0:
            hold_str = '🔒HOLD' if self.depth_hold_active else '⏳bekle'
            if self.vertical_disable:
                hold_str = '🚁DİKEY-KAPALI'
            pos_str = ''
            if self.pos_hold_enabled and self.dvl_ready:
                drift = math.hypot(self.dvl_x - self.ref_x, self.dvl_y - self.ref_y)
                pos_str = (f'| 🌊 sürüklenme {drift:.2f}m sway={strafe_corr:+.0f} surge={surge_corr:+.0f}')
            self.get_logger().info(
                f'{hold_str} | YAW {self.current_heading:.1f}→{self.reference_heading:.1f}° '
                f'PID={pid_out:.1f} | D {self.current_depth:.2f}→{self.target_depth:.2f}m {pos_str}'
            )

    def _publish_reference(self):
        if self.reference_heading is not None:
            msg      = Float32()
            msg.data = float(self.reference_heading)
            self.ref_heading_pub.publish(msg)

    @staticmethod
    def _heading_error(current: float, reference: float) -> float:
        err = reference - current
        if err >  180.0: err -= 360.0
        if err < -180.0: err += 360.0
        return err

    @staticmethod
    def _circular_mean(angles: list) -> float:
        sin_sum = sum(math.sin(math.radians(a)) for a in angles)
        cos_sum = sum(math.cos(math.radians(a)) for a in angles)
        return math.degrees(math.atan2(sin_sum, cos_sum)) % 360.0

    def stop_motors(self):
        stop_msg      = Int32MultiArray()
        stop_msg.data = [1500] * 8
        for _ in range(10):
            self.motor_pub.publish(stop_msg)
            time.sleep(0.05)


def main():
    rclpy.init()
    node = AUVController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Controller kapatılıyor...')
        node.stop_motors()
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
