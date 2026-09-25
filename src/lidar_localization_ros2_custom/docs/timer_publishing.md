`enable_timer_publishing: true`にすると、NDTの処理とは別のコールバックグループで、最後に得た座標変換を繰り返し配信します。`lidar_localization_node`は2スレッドのexecutorを使用します。統合起動で使う`lab_navigation/params/localization.yaml`では、配信を有効にし、`pose_publish_frequency: 30.0`に設定しています。

`enable_map_odom_tf: true`の場合の動作は次のとおりです。

1. 固定初期姿勢、手動の初期姿勢、またはNDT結果を受け取ったら、その姿勢と**同じ時刻**の`odom → base_link`から`map → odom`を求め、保持します。
2. タイマーは保持した`map → odom`を現在時刻のTFとして配信します。NDTが遅れても、前回の変換を引き続き使います。
3. `/pcl_pose`は、保持した`map → odom`と最新の`odom → base_link`を合成して配信します。姿勢のstampは、そのodomのstampです。odomの更新が止まった場合まで、姿勢の時刻を新しく見せる処理ではありません。
4. 次のNDT結果を採用できたら、保持する変換を更新します。`/path`にはNDT結果を記録し、タイマーの再配信では点を追加しません。

元の姿勢のstampは保持しています。1秒以上古い値を再利用している場合は、`Reusing localization correction: source pose is ... sec old`という警告を最大5秒に1回出します。TFの配信継続は、NDTによる新しい観測や自己位置の精度向上を意味しません。

手動で自己位置を修正した場合は以前の保持値を無効にします。その修正時刻のodomが取得できなければ、古い修正値を再配信せず、次に変換を求められる結果を待ちます。起動時も、変換を一度も求められていない間は配信しません。

`enable_map_odom_tf: false`の場合は、最後の`map → base_link`と姿勢を現在時刻で繰り返し配信します。この場合、再計算の間の移動は配信姿勢に反映されません。`enable_timer_publishing: false`では従来どおりNDTの結果が出た時点で配信します。

ノードのdeactivate・cleanup・shutdown・error時はタイマーを停止します。配信スレッドを分けても、OSがスレッドを実行できないほどCPUが不足する場合の周期は保証しません。

自動テストはROS domain 219と専用トピックを使い、ハードウェアへ接続せずに実行します。計算側のコールバックグループを500ms止めてもTFが継続すること、移動と回転の反映、手動修正、deactivate/activateを検証します。

```bash
colcon build --packages-select lidar_localization_ros2_custom --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select lidar_localization_ros2_custom --event-handlers console_direct+
colcon test-result --test-result-base build/lidar_localization_ros2_custom --verbose
```
