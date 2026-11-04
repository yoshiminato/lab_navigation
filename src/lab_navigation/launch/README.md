# lab_navigation の launch ファイル

ナビゲーションの起動ファイルは `navigation_<環境>_<測位方式>.launch.py` と命名します。
環境は `real`、`gazebo`、`isaacsim`、測位方式は `lidar` または `visual` です。
新しい方式を追加するときも、この順序を使ってください。

| ファイル | 用途 |
| --- | --- |
| `navigation_real_lidar.launch.py` | 実機で LiDAR 測位と Nav2 を起動 |
| `navigation_real_visual.launch.py` | 実機で HLoc 視覚測位と Nav2 を起動 |
| `navigation_gazebo_lidar.launch.py` | Gazebo と LiDAR 測位、Nav2 を起動 |
| `navigation_isaacsim_lidar.launch.py` | Isaac Sim の ROS トピックを使い LiDAR 測位と Nav2 を起動 |
| `navigation_isaacsim_visual.launch.py` | Isaac Sim の ROS トピックを使い HLoc 視覚測位と Nav2 を起動 |

例えば `ros2 launch lab_navigation navigation_real_lidar.launch.py` で起動します。
Isaac Sim 用の二つは、Isaac Sim 側が ROS トピックを配信している状態で使います。

実験用は目的が分かる名前にします。`localization_evaluation_real.launch.py` は
実機で LiDAR 測位を Nav2 に使い、HLoc の推定結果を比較用に記録します。
`sfm_capture.launch.py` は実機とシミュレーションで共通の画像収集、
`sfm_capture_isaacsim.launch.py` は Isaac Sim の真値 TF を加える収集です。

`diffbot_control.launch.py`、`joy_controller.launch.py`、`velodyne_points.launch.py`
は再利用する部品の起動ファイルです。
