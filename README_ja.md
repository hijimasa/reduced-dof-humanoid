# reduced-dof-humanoid（日本語）

ROBO-ONE Light 向けに軸数を減らしたヒューマノイドの検討リポジトリです。Rhoban の
[Microban](https://github.com/Rhoban/microban) から派生していますが、Rhoban および原作者
Marc Duclusaud 氏とは無関係の非公式プロジェクトです。英語版は [README.md](README.md)。

**状態: 実験段階、未製作。** 同じ生成スクリプトから 3 つの MuJoCo モデルを出しています。サーボを外した箇所は、
そのサーボが入っていた部品にその機能（足首のヒンジ、肩、首）を生やして印刷し直します。股ヨーの位置だけは
サーボ外形のダミーブロックを入れ、14 軸へはそのまま差し替えで上げられます。印刷部品は CAD ドラフトで、
モデルと Microban の CAD に対しては確認済みですが、印刷・組立はまだです。

| モデル | 構成 | 質量 |
| --- | --- | --- |
| 16 軸（中間段階） | 足首ピッチをパラレルリンクで受動化、頭を固定 | 0.809 kg |
| **12 軸（標準）** | さらに肩ロールと股ヨーを省略。脚 4×2、腕 2×2。股ヨーの位置には XL330 と同じ外形の 3D プリント製ダミーを入れ、足首・肩・首は隣の部品と一体化した印刷部品にする | 0.726 kg |
| **14 軸（アップグレード）** | 12 軸の股ヨー位置のダミーを本物のサーボに差し替えたもの。その場旋回ができる | 0.756 kg |

Microban は 19 軸、0.819 kg です。

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
model/parts_derived/     Microban の CAD から派生した部品（CC BY-NC-SA 4.0）: 80 mm 足裏、一体化した足首・股ブロック、首付きの頭、延長した前腕
tools/
  make_microban_pl.py    Microban の robot.xml から上記モデルを生成する
  make_parts.py          リンク、膝ヨーク、軸キャップ、肩＋上腕、股ヨーのダミー、Microban の STEP から派生する部品（足首・股ブロック、頭、前腕）の CadQuery スクリプト。CAD 交差チェックも報告
  verify_parallel_leg.py 閉ループ残差・足裏平行性・起立の検証
  check_clearance.py     股・膝の可動域を走査して追加部品の干渉を報告
  render_preview.py      プレビュー画像の生成
  render_parts_check.py  追加部品の配置図（元部品を半透明に）の生成
  step_holes.py          Microban の STEP から穴パターンを読む
motion/                  検証に使ったモーションツールとロボットプロファイル（LegacyMotionEditor の tools/ からパスを
                         直して複製。motion/README.md 参照）と、生成した LME プロジェクト（motion/projects/）
docs/
  design_ja.md           軸数削減の検討メモ（規則との対応、各段階の検証結果、費用）
  parts_ja.md            追加部品の設計メモ（ピボット配置、配置制約、部品一覧、未確認事項）
```

## パラレルリンク脚

脚ごとに、大腿と脛に沿った 2 つの平行四辺形をリンクバーで作り、膝ブロックと足裏を骨盤と
平行に保ちます。足首ピッチの関節は受動になり、サーボとホーンを外せます。

- 膝サーボの指令角は大腿基準ではなく**骨盤基準**になります。既存の Microban のモーションは作り直しが必要です。
- 大腿側のバーは大腿の**後ろ**にあり、ピボットは股・膝の軸の後方 22 mm・下 8 mm（特異点は屈曲 110.6°、伸展 69.4°）。
  股側のピボットは股ブラケットの後ろ裾のすぐ横、大腿側板が決して掃かない象限に来るので、ブラケットから生やした
  ラグで持ちます（`parts_derived/hip_block_integrated`。壁にフランジを当て、20×22 から 10×10 へ 23 mm でテーパーし、付け根はフィレット。直角の根元にしない）。
  下腿側のバーは脛の後ろで、後方 30 mm・上 30 mm（足首ロールサーボと足を避けるため。特異点 135° / 45°）。
  特異点から 8° の余裕をとり、可動域は股ピッチ −100°〜+62°、膝 −37°〜+127°（骨盤基準）です。
- このロボットのために新設・変更した部品はすべて緑色で描いています（サーボ形状のダミーは琥珀色、短くした足も緑）。リンクと膝ヨークには衝突形状（凸包）も持たせています。
  `tools/check_clearance.py` で可動域を走査しても他部品との干渉はありません。

## 実機で変わる部品

| 部品 | 変更 | モデル上の効果 |
| --- | --- | --- |
| 足首ブロック | 足首ピッチの XL330 とアイドラホーンを外す。2 つのサーボをまたぐ Microban の足首ブロック（キャップ）を、サーボの位置にヒンジ体を持つ 1 部品として印刷し直す。ピッチ軸上の Ø5.9 スタブ軸が脛側板の既存穴に入り、下腿バーのピボット支持（根元ブロック・梁・腕）も外側に一体化（`parts_derived/ankle_block_integrated_{right,left}`） | 片足 47 g → 33 g。膝は 132° まで折れる（元は 117°） |
| 股ブラケット | 後ろ裾に大腿バーの上側ピボットのラグを生やして印刷し直す（`parts_derived/hip_block_integrated_{right,left}`） | 片側 +3.6 g |
| 首 | 胴体上部の XL330 を外す（Microban は首サーボを逆さに付けていて、ホーンが胴体側、頭がサーボ本体側）。頭に首を生やした 1 部品として印刷し直す（`parts_derived/head_integrated`）。頭の裏の旧ホーン座（PCD 12 の穴と座ぐり）はパッドで埋め、旧サーボ軸上の素直な Ø18 ステムがポケットに入る台座まで下りる。固定はホーン用の M2 4 本で胴体の内側から。頭の位置は変わらない | −17 g |
| 前腕 | Microban の前腕の手先に Ø10 の棒を 40 mm 生やして延長（`parts_derived/radius_long`）。腕は肩軸から 183 mm になり、KXR や Robovie-nano に近い比率で、起き上がりの手の届きも伸びる。ROBO-ONE Light は肩軸から 26 cm まで・歩行中は手先を床から 50 mm 以上と定めており、歩行の屈んだ姿勢で腕を伸ばしても 86 mm、歩容の肘前 60° では 114 mm ある。`--forearm-ext 0` で元に戻せる | 片腕 +4 g |
| 肩ロール | 上腕上端の XL330 を外す。肩ブラケットと上腕を 1 つの太い印刷部品にする（`parts/shoulder_arm_simple_{right,left}`）。残すのは Microban とのインターフェースだけ: 肩ピッチホーンの PCD 12 の板と、Microban の上腕プレートと同じ肘サーボの保持（XL330 の端面にねじ穴は無く、ホーン側の面とアイドラ側の面の上隅の穴にホーン軸方向の M2 4 本。天板から下がる 2 枚の短いタブ、x −45.4 / −20.0）。腕は元のロール軸まわりに 30° 回した形で、24×18 の胴（壁 3 mm、角は丸め）が板と 16×16 の柱をつなぐ。ホーンの 4 本のねじは胴を貫く Ø5 の段付き穴からドライバで締める。垂らした腕は膝のリンク積層の真上にあるので、外転させて初めて歩行で肘を前に曲げられる（歩容は前 60°）。`--arm-abduct` で角度を変えられる | −30 g |
| 股ヨー（12 軸） | 胴体下部の XL330 2 個を外し、ダミーを入れる。14 軸ではサーボに戻す | −30 g |
| 足 | 接地板の爪先側を 10 mm 詰めて接地長 80 mm に（ROBO-ONE Light の「脚長の 55 % 以下」）、`parts_derived/foot_sole80` | 衝突形状の板も同じ長さに |
| リンクバー | 脚ごとに大腿バー 1 本（62.2 mm）と下腿バー 1 本（61.7 mm）、どちらも脚の後ろ、3×8 mm、M2 ピン。脚の外側で面を分けて配置 | 平行四辺形を閉じる |
| 膝ヨーク | 膝の外側の三角板。両バーのピボットを持ち、膝軸キャップに乗る | 6 g |
| 膝軸キャップ | 膝サーボのホーンに共締めし、Ø8 のスタブ軸を外側に出す | ヨークの軸受け |

配置は何度か見直しました。いずれも `tools/check_clearance.py` の可動域走査、歩容の再生、ブラケット延長部の
CAD 交差チェック（`tools/make_parts.py` が報告）で確認しています。垂らした前腕を避けるため一時は大腿バーを
前に置きましたが、腕を外転させてその理由が消え、前のバーは支持の迂回が長くなるので、後ろに戻して股ブラケットの
すぐ横で支持する形にしました。両バーが後ろにあるので大腿バーはヨークの外側（35〜38 mm）に置きます（内側だと
ヨークの下腿ピンが掃き、股伸展 57° 以上で膝軸スタブにも当たる）。経緯・横方向の積層・配置制約・未確認事項は
[docs/parts_ja.md](docs/parts_ja.md) にまとめています。印刷前に Onshape で元部品と組み合わせて確認してください。

質量は 0.833 kg（本家モデル）→ 0.809 kg（16 軸）→ 0.756 kg（14 軸）→ 0.726 kg（12 軸）。印刷部品は 1.0 g/cm³（PLA、壁 3 層・充填 40 % 程度）で計上し、一体化足首ブロック 17.3 cm³（キャップ単体 9.2）、股ブラケット 13.3 cm³（9.7）、肩＋上腕 13.5 cm³、頭に生やした首 6.4 cm³、前腕の延長 4 g、股ヨーのダミー 7.7 cm³（2 mm シェル）。`--dummies` で以前の、足首・肩・首にもサーボ形状のダミーを使う版が生成できます。

<p align="center">
  <img src="docs/images/integrated_parts.png" alt="股ブラケットから生やしたピボット支持、一体化した足首ブロック、肩＋上腕、首付きの頭" width="900">
</p>

<p align="center">
  <img src="docs/images/parts_hip_shoulder.png" alt="一体化した股ブラケットと肩＋上腕の単体図" width="900">
</p>

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
| うつ伏せからの起き上がり | 8/8（6.7 s、9 姿勢） | 8/8（同じ姿勢列を再タイミング、5.2 s） |
| 仰向けからの起き上がり | 8/8（5.1 s、8 姿勢。40 試行で 40/40） | 8/8（同じ姿勢列を再タイミング、4.8 s。39/40） |

起き上がりは両方とも最終形状で探索し直しました（`motion/getup_supine.py --pitch 90` / `--pitch -90`）。前腕を 40 mm 延長した
ことで、うつ伏せ探索は以前の形状で 16 シード中 15 を捕らえていた膝立ちの局所解に落ちなくなり、8 シード中 2 つが再タイミング前に
5/8・6/8 で立ち上がりました。プロファイルに入れたのはシード 72 の 9 姿勢の経路（`motion/tune_getup.py` で再タイミング）で、
寝返って反対向きに立ち上がります。仰向けからは腕で上体を起こし、股を新しい限界の −100°（大腿リンクを後ろに移して広がった屈曲域）まで
曲げて座り、しゃがんで立ちます。シード 80 の 8 姿勢で、探索時の時間配分では 4/8、各モデルで再タイミングして 8/8 です。
以前の方式（1 姿勢でうつ伏せに寝返ってから同じ経路）は 7/8 で 8 s かかっており、その寝返り姿勢と、うつ伏せから仰向けへの寝返りは
`motion/candidates/` に残しています。
歩行は肘を前に 60° 曲げ、腕は肩の一体部品で 30° 外転した姿勢で行います。モーション関係のツール
（脚 IK によるキーフレーム生成、外乱付きの検証、歩容と起き上がりのチューナ、起き上がりのビーム探索）と
ロボットプロファイルは `motion/` に置いてあり（LegacyMotionEditor から複製）、このリポジトリだけで表を再現できます:

```bash
LME_ROBOT=microban_12 python3 motion/build_project.py        # -> motion/projects/LME_microban_12_save.xml
LME_ROBOT=microban_12 python3 motion/sim_check.py --trials 8  # 上の表
```

歩行は静的歩行で約 32 mm/s です。Microban 本家の RL 歩行（前進上限 0.7 m/s）のような動的歩行とは
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
python tools/check_clearance.py model/microban_pl12/microban_pl12.xml     # 追加部品の干渉走査
# 部品 STL と配置図
python tools/make_parts.py model/parts path/to/microban/cad/step   # 第 2 引数で Microban の STEP から派生部品も生成
MUJOCO_GL=egl python tools/render_parts_check.py model/microban_pl12/scene.xml docs/images/parts_check.png
```

主なオプション: `--dof 16|14|12`、`--naming lme`（LegacyMotionEditor の関節名）、`--back`/`--tilt`/
`--shank-back`/`--shank-tilt`（ピボット配置。負で前方・上。可動域はそこから計算）、`--link-x`/`--shank-link-x`
（バーの横方向の面）、`--arm-abduct`（肩の部品に作り込む外転角）、`--forearm-ext`（前腕の延長）、`--dummies`
（一体化部品の代わりにサーボ形状のダミーを使う旧構成）、`--derived`（派生部品のディレクトリ）、`--clevis`、
`--keep-ankle-servo`、`--neck-motor`、`--keep-motor`（RL 用のトルクアクチュエータ）、`--elbow-flexion-only`。

LegacyMotionEditor では `*_lme.xml` を使います。閉ループを検出して受動関節をスライダーから隠し、
受動関節の名前は Meridim の対応表に無いものにしてあるので、サーボのスロットと衝突しません。

## 費用概算

Microban の BOM（XL330 1 個 $23.90、サーボ以外 約 $113）に、追加部品の M2 ねじ・ワッシャ・
フィラメント約 $5 を足したものです。設定用の U2D2 一式（$56）は共通で別。

| 構成 | サーボ | 合計 | Microban（$567）比 |
| --- | --- | --- | --- |
| 12 軸標準 | 12 | 約 $405 | −$162（−29 %） |
| 14 軸 | 14 | 約 $453 | −$114（−20 %） |

## ROBO-ONE Light の規定との照合

モデル上の実測（脚長は規定どおり「最上部のピッチ軸から足裏まで、脚を伸ばした状態」）:

| 規定（1 kg 以下） | この脚長（146 mm）での上限 | Microban のまま | このリポジトリ |
| --- | --- | --- | --- |
| 足裏の前後長 ≤ 脚長の 55 % | 80.4 mm | 90 mm | 接地板を 80 mm に（`--sole-length` 既定値）。実機の足も短縮が必要 |
| 足裏の左右幅 ≤ 脚長の 35 % | 51.2 mm | 45 mm | 45 mm |
| 立位で左右の足裏が重ならない | — | 27 mm 離れている | 同左 |
| しゃがみ歩行禁止（膝関節 90° 以下はしゃがみ） | > 90° | — | 立ち姿勢を高くして歩容を再調整（docs/design_ja.md） |

## 今後

1. ドラフト部品を Onshape で元部品と組み合わせてフィット確認し、印刷する（未確認事項は docs/parts_ja.md）。
2. プロジェクトを LegacyMotionEditor / MuJoCo Studio と実機で再生する（モーションは MuJoCo に対してのみ調整済み）。
3. 動的歩行を試すなら、本家の学習パイプライン mjlab_microban にこのモデルを差し替える。

## ライセンス

- コード、ドキュメント、生成した MJCF: **Apache-2.0**（本家 robot.xml も Apache-2.0 で、その表示を保持）。
- `model/microban_pl/assets/` のメッシュ: **CC BY-NC-SA 4.0**（Marc Duclusaud 氏、Microban から無改変で複製）。
  `model/parts_derived/` の部品は Microban の CAD 部品を改変したもので、同じライセンスです。
  このためモデル全体としては**非営利**に限られます。キットとして販売するには原作者の許諾が必要です。
  詳細は [NOTICE.md](NOTICE.md)。
- `model/parts/` の部品はこのリポジトリの独自設計で Apache-2.0。

Microban 由来の内容を使う場合は Microban も引用してください（[README.md](README.md) の BibTeX 参照）。
