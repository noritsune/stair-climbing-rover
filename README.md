# Stair-Climbing Rover — ESP32 Firmware

実機ローバー（6輪・全輪ステア / ロッカーボギー）の ESP32 制御スケッチ。

- 各タイヤのステアは **±180°** の広角（2:1 ギアのワイドレンジサーボ）。タイヤは向きたい方向へ直接向く。
- ステアのスルーレートは 2 倍（2:1 ギア分）。
- **逆転フリップ最適化**：目標まで 90° を超えて回す必要があるときは、モーターを逆転して目標±180° を狙い、回転量を 90° 未満に抑える。
- サーボ追従中はモーター出力を `cos(角度誤差)` でゲーティングし、横滑りを防ぐ。
- **起動時は全輪 0°**（サーボはセンター）。

[![moving](moving.mp4)](

## ハードウェア構成

| 部品 | 役割 |
|---|---|
| ESP32 (WROOM-32 想定) | メインコントローラー |
| PCA9685 (I2C 0x40) | ステアリングサーボ 6ch ドライバー |
| DRV8833 × 3 | 走行モーター 6個（各ドライバー 2個）|
| サーボ × 6 | 各輪のステアリング（2:1 ワイドレンジギア）|
| DCモーター × 6 | 各輪の駆動 |
| DualShock 4 | コントローラー（Bluetooth）|

### 輪の呼称と並び順

コード内の固定順序は `FL, FR, ML, MR, BL, BR`。

| 略号 | 位置 |
|---|---|
| FL / FR | front（前）左 / 右 |
| ML / MR | middle（中）左 / 右 |
| BL / BR | back（後）左 / 右 |

### ピン配線（`RoverConfig.h`）

**ステアリングサーボ（PCA9685）**

| 輪 | PCA9685 ch |
|---|---|
| FL | 5 |
| FR | 4 |
| ML | 3 |
| MR | 2 |
| BL | 1 |
| BR | 0 |

ESP32 ↔ PCA9685 は I2C：`SDA=GPIO13`, `SCL=GPIO26`。PCA9685 の V+ にはサーボ用電源を別途供給する。

**走行モーター（DRV8833、IN1=正転 / IN2=逆転）**

| 輪 | IN1 (GPIO) | IN2 (GPIO) |
|---|---|---|
| FL | 23 | 22 |
| FR | 16 | 4 |
| ML | 21 | 19 |
| MR | 32 | 33 |
| BL | 18 | 17 |
| BR | 27 | 14 |

> **注意:** GPIO16 は ESP32-**WROVER** では PSRAM に使われる（FR IN1 に割り当て）。WROVER を使う場合は `RoverConfig.h` でピンを変更すること。入力専用ピン(34-39)・フラッシュ(6-11)・ブートストラップ(0/2/12/15) は避けてある。I2C は `SDA=GPIO13` / `SCL=GPIO26` を使用。

## 依存ライブラリ

- [PS4-esp32](https://github.com/aed3/PS4-esp32)（aed3）
- Adafruit PWM Servo Driver Library

> **重要:** esp32 ボードパッケージは **3.1.3** を使用すること。
> それ以降では PS4-esp32 が使う `esp_base_mac_addr_set` でコンパイルエラーになる。
> 参考: https://forum.arduino.cc/t/compilar-library-ps4-controler/1373660

## コントローラーの事前準備（初回のみ）

1. DS4 を PC に**有線接続**する
2. [Sixaxis Pair Tool](https://sourceforge.net/projects/sixaxispairtool/) を開く
3. アドレス欄に `aa:bb:cc:dd:ee:ff` を入力する（`RoverConfig.h` の `PS4_MAC_ADDRESS` と一致させる）
4. **Update** を押す → DS4 を PC から抜く

## 操作方法

| 操作 | 動作 |
|---|---|
| 左スティック 上下 | 前後並進 |
| 左スティック 左右 | 左右並進（カニ歩き）|
| 右スティック 左右 | 旋回 |
| `R1`（押している間）| ブースト（速度 ×2.2）|
| `Options` | 全ステアを 0° にリセンター |

DS4 未接続時は走行モーターを停止（ステア角は保持）するフェイルセーフ。

## 書き込み方法

1. Arduino IDE で `stair-climbing-rover.ino` を開く（同フォルダの `RoverConfig.h` / `RoverKinematics.h` も自動で読み込まれる）
2. ボードを **ESP32 Dev Module** に設定
3. ESP32 を USB 接続し、正しい COM ポートを選択
4. 書き込む（ポートが出ない場合はデータ転送対応の USB ケーブルに替える）

## キャリブレーション（`RoverConfig.h`）

| 定数 | 用途 |
|---|---|
| `SERVO_MIN_US` / `SERVO_MAX_US` | サーボのパルス幅（0–180°）。実機サーボに合わせる |
| `SERVO_TRIM_DEG[]` | 各輪の機械的ゼロ点の微調整 |
| `SERVO_REVERSED[]` | サーボホーンが逆向きの輪を反転 |
| `MAX_WHEEL_SPEED_MPS` | 駆動の m/s → PWM デューティ正規化基準（小さくすると速く感じる）|
| `MOTOR_MIN_DUTY` | 停動（スタール）するなら 40 程度に上げる |

サーボ角への変換は 2:1 ギアを前提に `servo = outputDeg / 2 + 90`（出力 −180°→サーボ 0°、0°→90°、+180°→180°）。

## ファイル構成

| ファイル | 役割 |
|---|---|
| `stair-climbing-rover.ino` | エントリポイント。入力読み取り → 運動学 → サーボ/モーター駆動 |
| `RoverConfig.h` | ピン配置・寸法・速度・キャリブレーション定数 |
| `RoverKinematics.h` | ハードウェア非依存のピュア運動学（シミュレーターから移植・単体テスト可能）|
| `test/verify_kinematics.py` | 運動学の検算スクリプト（ホストで実行）|

## 検証

`RoverKinematics.h` と同一の数式を Python で再現し、Pattern B の期待挙動を検算済み。

```bash
python test/verify_kinematics.py
```

前進 / カニ歩き / 後退（逆転フリップ）/ その場旋回 / デッドゾーン / ブースト / コサインゲーティング / サーボ角マッピングの 16 項目が PASS。

> 実機サーボ・モーターの動作確認は、ハードウェア接続後に各輪を個別に通電して、ステア方向（`SERVO_REVERSED`）と駆動極性（`MOTOR_IN1`/`IN2` の正転方向）を確認すること。
