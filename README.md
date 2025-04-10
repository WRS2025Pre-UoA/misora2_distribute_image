# misora2_distribute_image
## 内容
 - misoraから受け取った画像をオペレーターPCからの信号をもとに、pressure,qr,cracks,metal_lossそれぞれに1.0secだけ送信する

## 実行コード
~~~bash!
colcon build
source install/setup.bash
ros2 run misora2_distribute_image distribute_image_node --ros-args -p mode:=P<ミッション番号>
~~~