# 収集セッションと地図のディレクトリ配置

Visual Localization の入力は収集セッションのルートです。ディレクトリ名は特徴量方式で揃え、HLocの内部ライブラリ名を成果物名に含めません。

```text
captures/<session>/
├── images/                              元画像
├── timestamps.csv
├── camera_info.json
├── bag/                                 記録したTF・CameraInfo等
├── poses_world_camera_gt/poses.csv     使用する位置事前情報の例
├── maps/
│   ├── sift/
│   │   ├── database.db
│   │   ├── sparse/0/
│   │   └── reference_features.h5
│   └── superpoint/
│       ├── database.db                  DBを作る経路の場合
│       ├── sparse/0/
│       └── reference_features.h5
├── retrieval/netvlad.h5                参照画像検索用。方式間で共有
└── runs/
    ├── sift/<run名>/                    中間・比較・評価結果
    └── superpoint/<run名>/              pairs、matches、作業用DB等
```

`maps/<method>/sparse/0` と同じ方式の `reference_features.h5` は一組です。特徴点の順序が3D点の対応と一致しなければ測位に使えません。`retrieval/netvlad.h5` は共通で、各地図に登録された画像の記述子を含みます。`database.db` は地図生成と追跡調査用に保持します。オンライン測位では疎モデル、参照H5、NetVLADを読み込みます。

ROS 2 の `data_path` は常に `<session>` を指します。`local_method:=sift` は `maps/sift`、`local_method:=superpoint` は `maps/superpoint` を選びます。従来の `superpoint_lightglue` も同じ方式の別名として受け付けます。実装上の対応表は [dataset_layout.py](../src/hloc_localizer/hloc_localizer/dataset_layout.py) にあります。特徴方式を増やす場合は新しい `maps/<method>` とbackendの対応を追加し、既存地図の参照特徴量を流用しません。

## このPCにある地図

| セッション | 完成地図 | 備考 |
| --- | --- | --- |
| `isaac_001` | `sift`, `superpoint` | 両方式とも908登録画像。共通NetVLADあり |
| `isaac_gt_test_002` | `sift` | 602登録画像。共通NetVLADあり |
| `hv_building_lab/20261010` | `sift`, `superpoint` | 両方式とも1370登録画像。共通NetVLADあり |
| `hv_building_lab/20261007` | `sift`の疎モデルのみ | 参照H5とNetVLADは未作成 |
| `calib_data` | なし | カメラ校正用画像 |

既存セッションの移動は [migrate_map_layout.py](../script/migrate_map_layout.py) に記録しています。引数なしの移動は行わず、`python3 script/migrate_map_layout.py captures` は予定表示、`--apply` は移動の実行です。このPCの対象データは移行済みです。スクリプトは既存の保存先を上書きせず、再実行しても追加移動がなければ終了します。

新規SIFT地図は [COLMAP_SFM.md](COLMAP_SFM.md) または [ISAAC_SIM_POSE_PRIORS.md](ISAAC_SIM_POSE_PRIORS.md) に従って `maps/sift` へ生成します。SIFT参照H5とNetVLADは [prepare_localization.py](../script/prepare_localization.py) で作成・検証します。新規SuperPoint地図は [build_superpoint_map.py](../script/build_superpoint_map.py) または [SuperPoint手順](../src/hloc_localizer/README_hloc_superpoint_colmap_sfm.md) を使います。
