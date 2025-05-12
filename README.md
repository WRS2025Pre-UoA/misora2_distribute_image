# misora2_distribute_image
## 内容
 - オペレータPCから受け取った信号をもとに、決められた時間画像を送信するノード
## 挙動の仕様
### ノード起動時、以下のパラメータを取得する
 - mode：実行するミッション番号 P1~P4,P6
 - check_duration_sec：画像を何秒間送信を行うか (秒)
 - timer_interval_ms：画像分配関数の実行間隔 (ミリ秒)

### 信号を受け取ったら
 - pressure, qr, crakcs, metal_lossのいずれかの起動トリガーを受信した場合
    - 対応するノードに対して、misoraから受け取った画像をcheck_duration_sec秒間送信する
    - 送信終了後、終了の合図として黒画像を送信する。
## 実行コード
~~~bash!
git clone git@github.com:WRS2025Pre-UoA/misora2_distribute_image.git
colcon build
source install/setup.bash
ros2 run misora2_distribute_image distribute_image_node --ros-args -p mode:=P<ミッション番号> -p check_duration_sec:=1.0 -p timer_interval_ms:=500
~~~