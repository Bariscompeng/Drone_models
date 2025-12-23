#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')

        # ---- Sub/Pub ----
        self.create_subscription(Float32, 'line_error', self.error_cb, 10)
        self.create_subscription(Float32, 'line_confidence', self.conf_cb, 10)

        # ✅ Odom ile yükseklik
        self.ODOM_TOPIC = '/odom'
        self.create_subscription(Odometry, self.ODOM_TOPIC, self.odom_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/model/M100/cmd_vel', 10)

        # ---- Durumlar ----
        self.state = "TAKEOFF"

        # ---- TAKEOFF (ODOM) ----
        self.target_alt = 2.0
        self.alt_tol = 0.05
        self.alt_kp = 1.2
        self.alt_vz_limit = 0.8
        self.alt_hold_time = 0.4
        self.alt_reached_since = None
        self.z = None

        # ---- SEARCH (sadece başlangıçta 1 kez 360) ----
        self.search_speed = 0.5
        self.dur_360 = (2 * math.pi) / self.search_speed
        self.search_360_done = False
        
        # ---- RECOVER (30 derece sağ-sol) ----
        self.dur_30 = math.radians(30) / self.search_speed
        self.dur_60 = math.radians(60) / self.search_speed
        self.line_timeout = 2.0  # 1.8'den 2.0'a (biraz daha toleranslı)

        # ---- Hız parametreleri ----
        self.max_speed = 0.25
        self.min_speed = 0.12  # 0.10'dan 0.12'ye (biraz daha hızlı minimum)

        # ---- Confidence eşikleri (AŞİRİ DÜŞÜK - ULTRA HASSAS) ----
        self.confidence = 0.0
        self.conf_low = 100.0  # 150'den 100'e 
        self.conf_high = 450.0  # 500'den 450'ye
        self.min_conf_to_follow = 80.0  # 120'den 80'e (AŞİRİ düşük!)
        
        # Debug için log ekle
        self.last_conf_log = 0.0
        self.get_logger().info(f"Min confidence eşiği: {self.min_conf_to_follow}")

        # ---- Error ölçümleri ----
        self.error = 0.0
        self.prev_error = 0.0
        self.last_msg_time = self.get_clock().now()

        # ---- DAHA YUMUŞAK FİLTRELER ----
        self.err_filt = 0.0
        self.err_alpha = 0.20  # 0.18'den 0.20'ye (biraz daha hızlı tepki)

        self.d_filt = 0.0
        self.d_alpha = 0.06  # 0.05'ten 0.06'ya

        self.deadband = 0.06  # 0.07'den 0.06'ya (biraz daha hassas)

        # ---- PID kazançları (DAHA YUMUŞAK) ----
        self.Kp_base = 0.32  # 0.38'den 0.32'ye (daha yumuşak)
        self.Kd_base = 0.06  # 0.08'den 0.06'ya (daha yumuşak)
        self.Ki = 0.04       # 0.05'ten 0.04'e (daha yumuşak)

        self.integral = 0.0
        self.integral_limit = 0.25  # 0.3'ten 0.25'e

        # ---- Turn limitleri (DAHA YUMUŞAK) ----
        self.turn_limit = 0.50  # 0.55'ten 0.50'ye
        self.prev_turn = 0.0
        self.max_turn_rate = 0.6  # 0.7'den 0.6'ya (daha yavaş değişim)

        self.turn_filt = 0.0
        self.turn_alpha = 0.15  # 0.10'dan 0.15'e (daha hızlı tepki ama hala yumuşak)

        # ✅ BASİTLEŞTİRİLMİŞ GEÇİŞ
        self.just_found_line = False
        self.found_line_time = 0.0
        self.initial_blend_duration = 0.6  # İlk 0.6 saniye yumuşak hareket

        # ---- Zamanlar ----
        now = self.get_clock().now()
        self.state_start_time = now
        self.prev_time = now

        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info("LineFollower: Basit yumuşak geçiş aktif")

    # ---------- Odom callback ----------
    def odom_cb(self, msg: Odometry):
        self.z = float(msg.pose.pose.position.z)

    # ---------- Callbacks ----------
    def error_cb(self, msg: Float32):
        self.error = float(msg.data)
        self.last_msg_time = self.get_clock().now()

    def conf_cb(self, msg: Float32):
        self.confidence = float(msg.data)
        
        # ✅ DEBUG: Confidence logla
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_conf_log > 0.5:  # Her 0.5 saniyede bir logla
            self.get_logger().info(f"Conf: {self.confidence:.1f} | State: {self.state}")
            self.last_conf_log = now_sec
        
        # ✅ ÇİZGİ GÖRÜLÜR GÖRMEZ HEMEN GEÇİŞ
        if self.state in ["SEARCH_360", "RECOVER_RIGHT", "RECOVER_LEFT", "RECOVER_CENTER"]:
            if self.confidence >= self.min_conf_to_follow:
                # ✅ DEBUG: Geçiş logu
                self.get_logger().warn(f"!!! ÇİZGİ TESPİT !!! Conf={self.confidence:.1f} Eşik={self.min_conf_to_follow}")
                if self.state != "FOLLOW":  # Tekrar geçiş yapma
                    self.get_logger().info(f"✅ ÇİZGİ BULUNDU ({self.state}) -> FOLLOW")
                    self.change_state("FOLLOW")
                    self.just_found_line = True
                    self.found_line_time = self.get_clock().now().nanoseconds / 1e9
                    self.reset_follow_memory()

    # ---------- Helpers ----------
    def change_state(self, new_state: str):
        self.state = new_state
        self.state_start_time = self.get_clock().now()

    def reset_follow_memory(self):
        # Hafızayı sıfırla ama mevcut error'u başlangıç yap
        self.prev_error = self.error
        self.integral = 0.0
        self.d_filt = 0.0
        # Turn'ü SIFIRLA (sarsıntıyı önler)
        self.prev_turn = 0.0
        self.turn_filt = 0.0
        # Error filtresini mevcut değerle başlat
        self.err_filt = self.error

    # ---------- Main Loop ----------
    def loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds / 1e9

        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 0.05
        self.prev_time = now

        cmd = Twist()

        # ================== TAKEOFF (ODOM) ==================
        if self.state == "TAKEOFF":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            if self.z is None:
                cmd.linear.z = 0.0
                self.cmd_pub.publish(cmd)
                return

            alt_err = self.target_alt - self.z
            vz = clamp(self.alt_kp * alt_err, -self.alt_vz_limit, self.alt_vz_limit)
            cmd.linear.z = float(vz)

            if abs(alt_err) <= self.alt_tol:
                if self.alt_reached_since is None:
                    self.alt_reached_since = now.nanoseconds / 1e9
                if (now.nanoseconds / 1e9) - self.alt_reached_since >= self.alt_hold_time:
                    self.get_logger().info("TAKEOFF bitti -> SEARCH_360")
                    cmd.linear.z = 0.0
                    self.alt_reached_since = None
                    self.change_state("SEARCH_360")
            else:
                self.alt_reached_since = None

        # ================== SEARCH_360 (SADECE BAŞTA) ==================
        elif self.state == "SEARCH_360":
            cmd.linear.x = 0.0
            cmd.linear.z = 0.0

            if elapsed < self.dur_360:
                cmd.angular.z = self.search_speed
            else:
                self.get_logger().warn("360 bitti -> RECOVER_RIGHT")
                self.search_360_done = True
                self.change_state("RECOVER_RIGHT")

        # ================== FOLLOW (BASİTLEŞTİRİLMİŞ YUMUŞAK) ==================
        elif self.state == "FOLLOW":
            cmd.linear.z = 0.0

            time_since_msg = (now - self.last_msg_time).nanoseconds / 1e9
            if time_since_msg > self.line_timeout or self.confidence < 100:  # 100'ün altına düşerse kayıp say
                self.get_logger().warn("Çizgi kayıp -> RECOVER_RIGHT")
                self.reset_follow_memory()
                self.just_found_line = False
                self.change_state("RECOVER_RIGHT")
            else:
                # EMA filtre
                self.err_filt = (1 - self.err_alpha) * self.err_filt + self.err_alpha * self.error
                e = 0.0 if abs(self.err_filt) < self.deadband else self.err_filt

                # Confidence normalize
                conf_n = (self.confidence - self.conf_low) / max(1e-6, (self.conf_high - self.conf_low))
                conf_n = clamp(conf_n, 0.0, 1.0)

                # Curve strength
                curve_strength = clamp(abs(e) / 0.30, 0.0, 1.0)  # 0.25'ten 0.30'a

                # Adaptive PID (DAHA YUMUŞAK)
                Kp = self.Kp_base * (0.80 + 0.35 * curve_strength)
                Kd = self.Kd_base * (0.70 + 0.45 * curve_strength)

                # Derivative
                d_raw = (e - self.prev_error) / dt
                self.d_filt = (1 - self.d_alpha) * self.d_filt + self.d_alpha * d_raw

                # Integral
                self.integral += e * dt
                self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)

                # PID çıkışı
                turn = (Kp * e) + (self.Ki * self.integral) + (Kd * self.d_filt)

                # Confidence bazlı
                turn *= (0.70 + 0.30 * conf_n)  # Daha az agresif

                # ✅ İLK BULUŞTA YUMUŞAK BAŞLANGIÇ (BASİTLEŞTİRİLMİŞ)
                tsec = now.nanoseconds / 1e9
                if self.just_found_line:
                    time_since_found = tsec - self.found_line_time
                    if time_since_found < self.initial_blend_duration:
                        # İlk 0.6 saniye yumuşak hareket
                        blend_progress = time_since_found / self.initial_blend_duration
                        blend_progress = clamp(blend_progress, 0.0, 1.0)
                        
                        # Daha hızlı başlangıç: 0.25'ten 1.0'a (önceden 0.15'ten)
                        blend_factor = 0.25 + 0.75 * (blend_progress ** 2)
                        turn *= blend_factor
                        
                        # ✅ DEBUG: Blend factor logla
                        if int(time_since_found * 10) % 2 == 0:  # Her 0.2s'de
                            self.get_logger().info(f"Blend: {blend_factor:.2f} | Turn: {turn:.2f}")
                    else:
                        self.just_found_line = False

                # Rate limiter (DAHA GÜÇLÜ)
                max_step = self.max_turn_rate * dt
                turn = clamp(turn, self.prev_turn - max_step, self.prev_turn + max_step)
                self.prev_turn = turn

                # Turn filtresi
                self.turn_filt = (1.0 - self.turn_alpha) * self.turn_filt + self.turn_alpha * turn
                turn = self.turn_filt

                # Turn limiti ve yumuşak anti-windup
                if turn > self.turn_limit:
                    turn = self.turn_limit
                    self.integral *= 0.5
                elif turn < -self.turn_limit:
                    turn = -self.turn_limit
                    self.integral *= 0.5

                # Hız hesaplama
                speed_reduction = (abs(e) ** 1.2) * 0.45
                speed = self.max_speed - speed_reduction
                speed = clamp(speed, self.min_speed, self.max_speed)

                # Confidence bazlı hız
                speed *= (0.60 + 0.40 * conf_n)
                speed = clamp(speed, self.min_speed, self.max_speed)

                cmd.linear.x = float(speed)
                cmd.angular.z = float(turn)

                self.prev_error = e

        # ================== RECOVER (30° SAĞ-SOL) ==================
        elif self.state == "RECOVER_RIGHT":
            cmd.linear.x = 0.0
            cmd.linear.z = 0.0
            
            if elapsed < self.dur_30:
                cmd.angular.z = -self.search_speed
            else:
                self.get_logger().info("RECOVER_RIGHT bitti -> RECOVER_LEFT")
                self.change_state("RECOVER_LEFT")

        elif self.state == "RECOVER_LEFT":
            cmd.linear.x = 0.0
            cmd.linear.z = 0.0
            
            if elapsed < self.dur_60:
                cmd.angular.z = self.search_speed
            else:
                self.get_logger().info("RECOVER_LEFT bitti -> RECOVER_CENTER")
                self.change_state("RECOVER_CENTER")

        elif self.state == "RECOVER_CENTER":
            cmd.linear.x = 0.0
            cmd.linear.z = 0.0
            
            if elapsed < self.dur_30:
                cmd.angular.z = -self.search_speed
            else:
                self.get_logger().error("Recovery başarısız -> STOP")
                self.change_state("STOP")

        # ================== STOP ==================
        elif self.state == "STOP":
            cmd = Twist()

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
