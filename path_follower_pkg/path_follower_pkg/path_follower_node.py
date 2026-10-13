import os
import csv
import math

import rclpy
from rclpy.node import Node
from gnss_ros_standardization.msg import GnssSolution
from ackermann_msgs.msg import AckermannDriveStamped

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower_node')
        
        # 1. CSV のパス自動取得
        current_dir = os.path.dirname(__file__)
        self.csv_file_path = os.path.join(
            os.path.dirname(current_dir), 'share', 'path_follower_pkg', 'config', 'marker_position.csv'
        )
        if not os.path.exists(self.csv_file_path):
            self.csv_file_path = os.path.expanduser('~/ros2_ws/src/path_follower_pkg/config/marker_position.csv')

        # 平面直角座標系（第9系）の基準点
        self.phi0 = math.radians(36.0)
        self.lambda0 = math.radians(140.0 + 12.5 / 60.0)
        
        # 目標軌道のデータリスト
        self.target_path = []
        self.target_speeds = []
        self.load_waypoints()

        # 周回管理・自車位置の変数
        self.target_wp_idx = 0
        self.lookahead_distance = 1.0
        self.wheelbase = 0.26

        # 最新の自車位置を保持する変数（初期値は0）
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.gnss_ready = False
    
        self.start_straight_counter = 0

        self.steering_trim = math.radians(6.0)

        # 2. パブリッシャーとサブスクライバー
        self.ackermann_pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        
        # 自己位置の更新だけをコールバックで行う
        self.subscription = self.create_subscription(
            GnssSolution,
            '/gnss/solution',
            self.gnss_solution_callback,
            10
        )
        
        # ★追加：1秒間に20回（0.05秒おき）に命令を連射するタイマー
        # これによってラジコン側のWatchdog（安全停止）を突破します
        timer_period = 0.05  # 秒
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Path Follower Node (Timer Ver.) has been started.')

    def wgs84_to_plane_rect(self, lat, lon):
        """緯度経度を平面直角座標系（第9系）のメートル単位に変換する"""
        phi = math.radians(lat)
        lam = math.radians(lon)
        a = 6378137.0
        f = 1.0 / 298.257222101
        m0 = 0.9999
        n = f / (2.0 - f)
        A_ = a / (1.0 + n) * (1.0 + (n**2)/4.0 + (n**4)/64.0)
        alpha = [0, (1.0/2.0)*n - (2.0/3.0)*(n**2) + (5.0/16.0)*(n**3), (13.0/48.0)*(n**2) - (3.0/5.0)*(n**3), (61.0/240.0)*(n**3)]
        t = math.sinh(math.atanh(math.sin(phi)) - (2.0*math.sqrt(n)/(1.0+n)) * math.atanh((2.0*math.sqrt(n)/(1.0+n))*math.sin(phi)))
        xi_prime = math.atan(t / math.cos(lam - self.lambda0))
        eta_prime = math.atanh(math.sin(lam - self.lambda0) / math.sqrt(1.0 + t**2))
        xi = xi_prime
        eta = eta_prime
        for j in range(1, 4):
            xi += alpha[j] * math.sin(2.0 * j * xi_prime) * math.cosh(2.0 * j * eta_prime)
            eta += alpha[j] * math.cos(2.0 * j * xi_prime) * math.sinh(2.0 * j * eta_prime)
        x_m = A_ * xi * m0
        y_m = A_ * eta * m0
        t0 = math.sinh(math.atanh(math.sin(self.phi0)) - (2.0*math.sqrt(n)/(1.0+n)) * math.atanh((2.0*math.sqrt(n)/(1.0+n))*math.sin(self.phi0)))
        xi0_prime = math.atan(t0 / 1.0)
        xi0 = xi0_prime
        for j in range(1, 4):
            xi0 += alpha[j] * math.sin(2.0 * j * xi0_prime)
        S0 = A_ * xi0 * m0
        return x_m - S0, y_m

    def load_waypoints(self):
        """CSVから目標軌道と目標速度を読み込む"""
        try:
            with open(self.csv_file_path, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)
                header = next(reader)
                for row in reader:
                    if not row: continue
                    lat = float(row[1])
                    lon = float(row[2])
                    speed = float(row[4])
                    x_m, y_m = self.wgs84_to_plane_rect(lat, lon)
                    self.target_path.append((x_m, y_m))
                    self.target_speeds.append(speed)
            self.get_logger().info(f'成功: {len(self.target_path)} 個のWPを読み込みました。')
        except Exception as e:
            self.get_logger().error(f'CSVの読み込みに失敗: {str(e)}')

    def gnss_solution_callback(self, msg):
        """オドメトリを受信したときは自車位置の変数を更新するだけにする"""
        try:
            lat = getattr(msg, 'lat', getattr(msg, 'latitude', None))
            lon = getattr(msg, 'lon', getattr(msg, 'longitude', None))
            if lat is not None and lon is not None:
                
                self.current_x, self.current_y = self.wgs84_to_plane_rect(lat, lon)

                if not self.gnss_ready:
                    self.gnss_ready = True
                    self.get_logger().info('【GNSS受信開始】有効な位置データを検知しました。これより直線カウントを開始します。')

            else:
               self.get_logger().warn('GnssSolution メッセージ内に緯度経度のデータが見つかりません。')
        except Exception as e:
            self.get_logger().error(f'自己位置の変換中にエラーが発生: {str(e)}')


       
        

    def timer_callback(self):
        """★新設：0.05秒ごとに超高速ループで実行される関数"""
        if not self.target_path:
            return

        if not self.gnss_ready:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = self.get_clock().now().to_msg()
            drive_msg.header.frame_id = 'base_link'
            drive_msg.drive.speed = 0.2  # GNSSが来るまでは「完全停止」させておく場合（動かしたいなら 0.2 など微速にする）
            drive_msg.drive.steering_angle = 0.0  # ステアリングは完全直進
            self.ackermann_pub.publish(drive_msg)
            self.get_logger().info('【GNSS未受信】データを待っています。直進状態で待機中...', throttle_duration_sec=1.0)
            return

        # 最新の自車位置を使って距離計算
        target_wp = self.target_path[self.target_wp_idx]
        dx = target_wp[0] - self.current_x
        dy = target_wp[1] - self.current_y
        distance_to_target = math.sqrt(dx**2 + dy**2)

        # ターゲットWPに近づいたら次へ進める
        if distance_to_target < self.lookahead_distance:
            self.target_wp_idx += 1
            if self.target_wp_idx >= len(self.target_path):
                self.target_wp_idx = 0
                self.get_logger().info('1周')
            target_wp = self.target_path[self.target_wp_idx]
        ld = distance_to_target if distance_to_target > 0.5 else self.lookahead_distance
        target_heading = math.atan2(dy, dx)

        final_steering = 0.0 
        
        if self.start_straight_counter < 60:
            # 走行開始直後（1回目の受信時）の位置を記録しておく
            if self.start_straight_counter == 0:
                self.init_start_x = self.current_x
                self.init_start_y = self.current_y

            final_steering = 0.0
            self.start_straight_counter += 1
            self.get_logger().info('【初期化中】GNSS受信完了。3秒間の直線走行中...', throttle_duration_sec=1.0)
            
            # 3秒間の直線走行がちょうど終わる瞬間（60回目のループ）
            if self.start_straight_counter == 60:
                dx_run = self.current_x - self.init_start_x
                dy_run = self.current_y - self.init_start_y
                
                # 3秒間で一定以上（50cm以上）実際に動いていれば向きを計算
                if math.sqrt(dx_run**2 + dy_run**2) > 0.5:
                    self.current_yaw = math.atan2(dy_run, dx_run)
                    self.get_logger().info(f'【初期化完了】移動軌跡から方位を確定しました: {math.degrees(self.current_yaw):.1f}°')
                else:
                    self.get_logger().error('【初期化失敗】3秒間で十分な距離を移動できませんでした。方位が不正確な可能性があります。')
                    # 必要であればカウンタをリセットしてやり直す場合はここを0にする
                    # self.start_straight_counter = 0 
        else:
            # 3秒経過後は、確定した self.current_yaw を使って通常のPure Pursuit計算を行う
            alpha = self.current_yaw - target_heading
            alpha = math.atan2(math.sin(alpha), math.cos(alpha))

            steering_angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld)

            max_steer = math.radians(25.0)
            steering_angle = max(-max_steer, min(max_steer, steering_angle))
            final_steering = steering_angle



        current_target_speed = self.target_speeds[self.target_wp_idx]
        
        corrected_steering = final_steering + self.steering_trim    
        

        # アッカーマンメッセージを常にパブリッシュし続ける
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = current_target_speed * 0.3
        drive_msg.drive.steering_angle = final_steering
        
        self.ackermann_pub.publish(drive_msg)


        self.get_logger().info(f'WP:#{self.target_wp_idx+1} | 距離:{distance_to_target:.2f}m | 指令角:{math.degrees(final_steering):.1f}°')
        

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
