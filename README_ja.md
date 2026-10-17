# reduced-dof-humanoid（日本語）

ROBO-ONE Light 向けに軸数を減らしたヒューマノイドの検討リポジトリです。Rhoban の
[Microban](https://github.com/Rhoban/microban) から派生していますが、Rhoban および原作者
Marc Duclusaud 氏とは無関係の非公式プロジェクトです。英語版は [README.md](README.md)。

**状態: 実験段階。** 同じ生成スクリプトから 3 つの MuJoCo モデルを出しています。

| モデル | 構成 | 質量 |
| --- | --- | --- |
| 16 軸（中間段階） | 足首ピッチをパラレルリンクで受動化、頭を固定 | 0.795 kg |
| **12 軸（標準）** | さらに肩ロールと股ヨーを省略。脚 4×2、腕 2×2。外したサーボの位置には XL330 と同じ外形の 3D プリント製ダミーを入れる | 0.735 kg |
| **14 軸（アップグレード）** | 12 軸の股ヨー位置のダミーを本物のサーボに差し替えたもの。その場旋回ができる | 0.765 kg |

Microban は 19 軸、0.819 kg です。外したサーボはすべて同形のダミーに置き換えるので、
元のブラケットは無改造で使え、サーボは後から戻せます。

<p align="center">
  <img src="docs/images/preview12.png" alt="12 軸版の側面図と斜め図（直立としゃがみ）" width="640">
</p>

## なぜ軸を減らすのか

Pollen Robotics の Microduck は腕も足首ロールも無い 15 軸で、学習したポリシーで歩きます。
小型ヒューマノイドの標準的な 19〜20 軸は、歩くだけなら過剰だと示唆しています。
一方 ROBO-ONE Light の規則は、各 1 軸以上の可動軸を持つ 2 本の腕、胴体と独立した頭、
足裏を 10 mm 以上上げた前後左右の歩行を要求します。そこで「規則の中で、キーフレーム制御のまま
どこまで削れるか」を検討しました。詳細は [docs/design_ja.md](docs/design_ja.md)。

| 軸 | 判断 | 理由 |
| --- | --- | --- |
| 足首ピッチ | 受動化（実施済み） | 大腿・下腿の平行四辺形で足裏を骨盤と平行に保つ（ashigaru と同じ機構） |
| 首 | 省略（実施済み） | 規則は頭の存在と寸法のみ要求。固定頭でも外形は保てる |
| 肩ロール | 省略（実施済み） | 規則は腕 1 軸以上。肩ピッチ＋肘でパンチと起き上がりはできる |
| 股ヨー | 12 軸では省略、14 軸で復活 | 無いと両足が常に平行で、旋回は床との滑りに頼る（Robovie-nano と同じ） |
| 股ロール・股ピッチ・膝・足首ロール | 残す | 横移動、10 mm の足上げ、位置制御サーボでの足裏を水平に保った荷重移動に必要 |

## リポジトリの内容

```
model/microban_pl12/     12 軸モデル（標準）。本家の関節名版、LME 命名版、ビューア用シーン
model/microban_pl14/     14 軸モデル（アップグレード）
model/microban_pl/       16 軸モデルと、Microban のメッシュ（無改変、CC BY-NC-SA 4.0）
model/parts/             追加部品の印刷用ドラフト STL（tools/make_parts.py の出力）
tools/
  make_microban_pl.py    Microban の robot.xml から上記モデルを生成する
  make_parts.py          リンク、膝ヨーク、軸キャップ、スペーサ、ダミーサーボの CadQuery スクリプト
  verify_parallel_leg.py 閉ループ残差・足裏平行性・起立の検証
  render_preview.py      プレビュー画像の生成
  step_holes.py          Microban の STEP から穴パターンを読む
docs/
  design_ja.md           軸数削減の検討メモ（規則との対応、各段階の検証結果、費用）
  parts_ja.md            追加部品の設計メモ（ピボット配置、配置制約、部品一覧、未確認事項）
```

## パラレルリンク脚

脚ごとに、大腿と脛に沿った 2 つの平行四辺形を後方リンクで作り、膝ブロックと足裏を骨盤と
平行に保ちます。足首ピッチの関節は受動になり、サーボとホーンを外せます。

- 膝サーボの指令角は大腿基準ではなく**骨盤基準**になります。既存の Microban のモーションは作り直しが必要です。
- 大腿側リンクは前方ピボットの後方 20 mm・下 10 mm、下腿側は後方 25 mm・上 25 mm（足首ロールサーボと足を避けるため）。
  特異点から 8° の余裕をとって、可動域は股ピッチ −109°〜+55°、膝 −37°〜+127°（骨盤基準）です。
- モデル上のリンクと膝ヨークは `tools/make_parts.py` の STL を緑色で描いています。

## 実機で変わる部品

| 部品 | 変更 | モデル上の効果 |
| --- | --- | --- |
| 足首ブロック | 足首ピッチの XL330 とアイドラホーンを外し、受動ヒンジ付きダミーブロックを入れる | 片足 47 g → 33 g。膝は 132° まで折れる（元は 117°） |
| 首 | 胴体上部の XL330 を外し、ダミーを入れて頭を直付け | −15 g |
| 肩ロール | 上腕上端の XL330 を外し、ダミーを入れる | −30 g |
| 股ヨー（12 軸） | 胴体下部の XL330 2 個を外し、ダミーを入れる。14 軸ではサーボに戻す | −30 g |
| 後方リンク | 脚ごとに大腿バー 2 本（62.2 mm）と下腿バー 1 本（61.7 mm）、3×8 mm、M2 ピン | 平行四辺形を閉じる |
| 膝ヨーク | 膝後方の新部品。大腿側と下腿側のピボットを持ち、外側の腕で膝軸キャップに乗る | 6 g |
| 膝軸キャップ | 膝サーボのホーンに共締めし、Ø8 のスタブ軸を外側に出す | ヨークの軸受け |

配置制約（腕との 1 mm の間隔、左右脚の間の 6 mm、股・足首後方のロールサーボ）と未確認事項は
[docs/parts_ja.md](docs/parts_ja.md) にまとめています。印刷前に Onshape で元部品と組み合わせて確認してください。

<p align="center">
  <img src="docs/images/parts_check.png" alt="ドラフト部品を配置し元部品を半透明にした確認図" width="640">
</p>

## 検証

`tools/verify_parallel_leg.py` による機構の検証と、LegacyMotionEditor のモーションツールによる
動作検証（サーボ零点誤差 0.3° と初期傾き 1° の外乱付き 8 試行）:

| 項目 | 12 軸 | 14 軸 |
| --- | --- | --- |
| 閉ループ残差（初期姿勢） | 0.000 mm | 0.000 mm |
| 前進・後退・横移動 | 8/8 | 8/8 |
| 旋回 | 8/8（すり足） | 8/8（股ヨー、72° / 141°） |
| ジャブ | 8/8 | 8/8 |
| うつ伏せからの起き上がり | 8/8 | 8/8 |
| 仰向けからの起き上がり（寝返り＋同じ起き上がり） | 8/8 | 8/8 |

歩行は静的歩行で約 36 mm/s です。Microban 本家の RL 歩行（前進上限 0.7 m/s）のような動的歩行とは
方式が違います。肘は本家と同じ ±125° で、起き上がりは肘を後ろ側に曲げて押す経路を使っています
（前屈曲のみに制限すると起き上がりの頑健性が大きく落ちました。`--elbow-flexion-only` で制限できます）。

## 使い方

必要なもの: Python 3.10 以上、`mujoco`（3.1 以上）、`numpy`。描画には `Pillow`、部品生成には `cadquery`。

```bash
# ビューアで見る
python -m mujoco.viewer --mjcf model/microban_pl12/scene.xml      # 12 軸
python -m mujoco.viewer --mjcf model/microban_pl14/scene.xml      # 14 軸
# Microban のクローンから再生成
python tools/make_microban_pl.py path/to/microban/src/model/mjcf/robot.xml \
    model/microban_pl12/microban_pl12.xml --dof 12 --meshdir ../microban_pl/assets --parts ../../parts \
    --scene model/microban_pl12/scene.xml
# 検証
python tools/verify_parallel_leg.py model/microban_pl12/microban_pl12.xml
# 部品 STL
python tools/make_parts.py model/parts
```

主なオプション: `--dof 16|14|12`、`--naming lme`（LegacyMotionEditor の関節名）、`--back`/`--tilt`/
`--shank-back`/`--shank-tilt`（ピボット配置。可動域はそこから計算）、`--clevis`、`--keep-ankle-servo`、
`--neck-motor`、`--keep-motor`（RL 用のトルクアクチュエータ）、`--elbow-flexion-only`、`--forearm-ext`。

LegacyMotionEditor では `*_lme.xml` を使います。閉ループを検出して受動関節をスライダーから隠し、
受動関節の名前は Meridim の対応表に無いものにしてあるので、サーボのスロットと衝突しません。

## 費用概算

Microban の BOM（XL330 1 個 $23.90、サーボ以外 約 $113）に、追加部品の M2 ねじ・ワッシャ・
フィラメント約 $5 を足したものです。設定用の U2D2 一式（$56）は共通で別。

| 構成 | サーボ | 合計 | Microban（$567）比 |
| --- | --- | --- | --- |
| 12 軸標準 | 12 | 約 $405 | −$162（−29 %） |
| 14 軸 | 14 | 約 $453 | −$114（−20 %） |

## 今後

1. ドラフト部品を Onshape で元部品と組み合わせてフィット確認し、印刷する。
2. 足裏寸法を ROBO-ONE Light の表（脚長の 55 % / 35 %）と照合する。
3. 動的歩行を試すなら、本家の学習パイプライン mjlab_microban にこのモデルを差し替える。

## ライセンス

- コード、ドキュメント、生成した MJCF: **Apache-2.0**（本家 robot.xml も Apache-2.0 で、その表示を保持）。
- `model/microban_pl/assets/` のメッシュ: **CC BY-NC-SA 4.0**（Marc Duclusaud 氏、Microban から無改変で複製）。
  このためモデル全体としては**非営利**に限られます。キットとして販売するには原作者の許諾が必要です。
  詳細は [NOTICE.md](NOTICE.md)。

Microban 由来の内容を使う場合は Microban も引用してください（[README.md](README.md) の BibTeX 参照）。
