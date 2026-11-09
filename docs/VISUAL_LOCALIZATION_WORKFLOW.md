# Visual Localization 作業フロー

画像収集からSfM、HLoc、ROS 2、Nav2までの入口です。詳細なコマンドと仕様は各工程の手順書を参照し、この文書では工程の順番、分岐、共通パスだけを定義します。
地図の保存先と既存データの一覧は [DATASET_LAYOUT.md](DATASET_LAYOUT.md) にまとめています。

## 1. 作業対象を一度だけ設定

以下を作業に使う Bash 端末の最初に実行します。別端末を使う場合は、このブロックをその端末でも実行してください。シェル変数は端末間では共有されません。

```bash
WORKSPACE_DIR="$HOME/lab_navigation_ws"
SESSION_NAME="isaac_001"
CAPTURE_DIR="$WORKSPACE_DIR/captures/$SESSION_NAME"
POSE_DIR="$CAPTURE_DIR/poses_world_camera_gt"
SIFT_DIR="$CAPTURE_DIR/maps/sift"
SUPERPOINT_DIR="$CAPTURE_DIR/maps/superpoint"
RETRIEVAL_DIR="$CAPTURE_DIR/retrieval"
RUNS_DIR="$CAPTURE_DIR/runs"
CAMERA_FRAME="CameraRGB"
WORLD_FRAME="world"
GT_TF_TOPIC="/sfm_gt_tf"

cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

新しい収集を始める場合だけ `SESSION_NAME` を未使用の名前へ変更します。既存セッションを処理する場合は、その収集先の名前を設定します。収集時に `$PWD` がワークスペース以外だった場合は、実際の収集先を `CAPTURE_DIR` に直接設定してください。

```bash
CAPTURE_DIR="$HOME/captures/isaac_001"
POSE_DIR="$CAPTURE_DIR/poses_world_camera_gt"
SIFT_DIR="$CAPTURE_DIR/maps/sift"
SUPERPOINT_DIR="$CAPTURE_DIR/maps/superpoint"
RETRIEVAL_DIR="$CAPTURE_DIR/retrieval"
RUNS_DIR="$CAPTURE_DIR/runs"
```

## 2. 画像を収集して検査

実機、Gazebo、通常のROS画像収集は [SFM_CAPTURE.md](SFM_CAPTURE.md) を使います。Isaac Simでカメラ真値TFも記録する場合は [ISAAC_SIM_POSE_PRIORS.md](ISAAC_SIM_POSE_PRIORS.md) を使います。

収集後、SfMへ進む前に実行します。

```bash
ros2 run sfm_capture check_dataset "$CAPTURE_DIR"
```

出力先が意図した場所か、画像数、CameraInfo、bag内の必要トピックを確認します。Isaac Simの位置制約付きSfMでは `/sfm_gt_tf` がbagに記録されている必要があります。

## 3. 画像ごとのカメラ位置を作る

Isaac Simのカメラ真値を使う場合は、実際のTF child名を `CAMERA_FRAME` に設定し、[ISAAC_SIM_POSE_PRIORS.md](ISAAC_SIM_POSE_PRIORS.md) の「各画像のカメラ中心位置を抽出」を実行します。出力の `valid` が `images` と一致し、`missing` が0であることを確認します。

一般のTFや実機の取り付け補正を使う場合は [POSE_PRIORS.md](POSE_PRIORS.md) を参照します。

## 4. SfM方式を選ぶ

| 経路 | 選ぶ場面 | 手順 |
| --- | --- | --- |
| SIFT + COLMAP | 標準的な基準モデル、COLMAPのSIFT特徴を使う | [COLMAP_SFM.md](COLMAP_SFM.md)。Isaacカメラ真値で位置制約を使う場合は [ISAAC_SIM_POSE_PRIORS.md](ISAAC_SIM_POSE_PRIORS.md) と [POSE_PRIORS.md](POSE_PRIORS.md) も参照 |
| SuperPoint + LightGlue + COLMAP | SP + LGの特徴で新しいSfMモデルを作る、またはpose prior付きSfMを行う | [HLoc + SP + LG SfM手順](../src/hloc_localizer/README_hloc_superpoint_colmap_sfm.md) |
| 既存SIFTモデルからSP + LG地図を追加 | SIFTモデルのカメラ姿勢を維持してSP + LG用3D点を再構築する | [既存SIFT地図へのSP + LG追加手順](../src/hloc_localizer/README_HLoc_ROS2_SP_LG.md) |

SIFTとSuperPointでは特徴点の座標・数・順序が異なります。SIFTモデルの3D点IDをSP + LG特徴にそのまま流用できません。目的に応じた上記の経路を選びます。

## 5. HLoc参照データを作る

SfMモデルだけではオンラインVisual Localizationを起動できません。モデルと同じ特徴方式に対応する参照特徴量・検索用データを作ります。

- SIFT地図: `$SIFT_DIR/sparse/0/`、`$SIFT_DIR/reference_features.h5`。
- SP + LG地図: `$SUPERPOINT_DIR/sparse/0/`、`$SUPERPOINT_DIR/reference_features.h5`。[新規SfM](../src/hloc_localizer/README_hloc_superpoint_colmap_sfm.md)または[既存SIFT姿勢からの再三角測量](../src/hloc_localizer/README_HLoc_ROS2_SP_LG.md)で作ります。
- 共通の検索特徴量: `$RETRIEVAL_DIR/netvlad.h5`。`python3 script/prepare_localization.py "$CAPTURE_DIR" --method sift` または `--method superpoint` で不足分を生成・検証します。

`data_path` は常に `$CAPTURE_DIR` に設定し、`local_method:=sift` または `local_method:=superpoint` で地図を選びます。従来の `superpoint_lightglue` も同じ地図を使う別名として受け付けます。特徴抽出やマッチングの中間ファイルは `$RUNS_DIR/<method>/<run名>/` に置きます。

## 6. ROS 2でLocalizationを確認

Isaac Sim統合の起動、入力topic、HLoc pose、TF確認は [ISAAC_SIM_VISUAL_LOCALIZATION.md](ISAAC_SIM_VISUAL_LOCALIZATION.md) が手順の正本です。HLocノード単体と特徴方式の切り替えは [README_HLoc_ROS2.md](../src/hloc_localizer/README_HLoc_ROS2.md) を参照します。

最初は自律移動を無効にして起動します。画像入力、`/hloc/camera_pose`、`/hloc/localization_status`、`map → odom → base_link` の順に確認してからNav2を有効にします。

## 7. Nav2へ接続して評価

`world → map` の補正、カメラ取り付けTF、LiDAR障害物検出が正しいことを確認してから短い目標で試します。HLocと真値を比較するときは、同じ座標系、対象フレーム、画像時刻を使います。

一般設計と座標系の説明は [NAV2_VISUAL_LOCALIZATION.md](NAV2_VISUAL_LOCALIZATION.md)、実機LiDAR基準の記録と解析は [visual_localization_evaluation.md](../src/lab_navigation/docs/visual_localization_evaluation.md) を参照します。

## 8. 成果物の対応

| 工程 | 成果物 |
| --- | --- |
| 収集 | `$CAPTURE_DIR/images/`、`timestamps.csv`、`camera_info.json`、`bag/` |
| Isaacカメラ位置抽出 | `$POSE_DIR/poses.csv`、`poses_report.json` |
| COLMAP SfM | `$CAPTURE_DIR/maps/<method>/database.db`、`sparse/0/` |
| HLoc参照データ | `maps/<method>/reference_features.h5`、`retrieval/netvlad.h5` |
| ROS 2運用 | `/hloc/camera_pose`、`/hloc/localization_status`、`map → odom` |

各処理を再実行するときは、新しい出力先を使います。COLMAPの特徴抽出DB、位置prior登録後DB、matcherの結果は処理段階ごとに意味が異なるため、同じファイル名を別工程の入力として使い回さないでください。
