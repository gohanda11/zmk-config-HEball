# HEball スリープ復帰 技術メモ

## 現状 (feature/sleep-wakeup ブランチ)

### 復帰できるもの
- ✅ **PMW3610 トラックボール** — 右側のみ搭載。wakeup-source + PM callbacks無効化済みでGPIO SENSEが維持される
- ✅ **エンコーダープッシュ** — kscan_gpio_sw に wakeup-source 設定済み。kscan_composite にも wakeup-source を追加したことで子デバイスのGPIOが破壊されなくなった

### 復帰できないもの
- ❌ **HEスイッチ（SC4823 AWAKE）** — 下記「SC4823の問題」参照
- ❌ **エンコーダー回転** — ZMK v0.3.0ではEC11ドライバーが `GPIO_INT_EDGE_BOTH` を使用。nRF52840の System OFF ではGPIOTE（エッジ割り込み）が無効化されるため、ハードウェア的に不可能

---

## SC4823 AWAKE ピンでの復帰が不可能な理由

### SC4823 Mode 3 の動作
SC4823はMode 3（SLEEP=HIGH）で低消費電力スリープに入り、12.5ms間隔でVOUTを監視する。
VOUTがVQ（=90%×VCC=2970mV @3.3V）から **0.4V以上低下** した場合、AWAKEピンがLOWになる。

### 問題の本質：0.4Vの固定しきい値
SC4823の設計思想は「磁石が遠くにある（VOUT≈VQ）状態が安静」。
しかしHEキーボードでは「磁石がセンサー近くに常在し、キー押下で更に近づく」設計のため：

| 状態 | SC4823想定 | HEball実態 |
|------|-----------|-----------|
| 安静時VOUT | ≈VQ (2970mV) | 2363mV〜2900mV |
| VQとの差分 | ≈0mV | 70〜607mV |

**安静時点で既にVQ-VOUT > 400mVとなるキーが存在**し、Mode 3に入った瞬間にAWAKEがLOWになる。
AWAKEは全SC4823で共有（wired-OR）のため、**1つでもNGキーがあると全体が復帰不可能**。

### テスト結果の例
- key[17]: VOUT=2363mV, delta=607mV → **NG**（磁石の物理的な漏洩磁界で隣接キーからの干渉）
- key[13]: VOUT=2574mV, delta=396mV → ギリギリOK
- key[23]: VOUT=2576mV, delta=394mV → ギリギリOK

### 磁石を除去した場合
問題のキーの磁石を物理的に除去すると、Phase 2テスト（Mode 3アイドル時のAWAKE状態）はHIGHを維持し、キー押下でLOWになることを確認済み。つまり**ハードウェア的にはAWAKEピンのメカニズム自体は正常に動作する**。

---

## 今後SC4823で復帰を実現するために必要なこと

### 方法1: 物理的な磁石距離調整（最も現実的）
- 全キーの安静時VOUTがVQ-400mV = 2570mV以上になるよう磁石の距離を調整
- Phase 0テスト（キャリブレーション時のADC値からVOUTを計算）で確認可能
- 課題：key[17]は隣接キーからの磁気漏洩で磁石なしでもdelta≈410mV

### 方法2: LPCOMP（nRF52840内蔵コンパレータ）での代替 → **不可能**
- SC4823 Mode 3ではVOUTが**ハイインピーダンス**（12.5ms周期で約27µsだけ駆動）
- LPCOMPは安定したアナログ入力が必要なため使用不可
- Mode 1を維持すると22×0.7mA=15.4mAの消費電流で電池が持たない
- 全SC4823が共通のSLEEPラインのため選択的にMode 1にできない

### 方法3: SC4823以外のホールセンサーICに変更
- VQ（基準電圧）やしきい値が調整可能なICを選定する
- または「磁石近接=安静」の設計思想に合ったIC

### 方法4: 外部ウェイクアップ回路
- 別途コンパレータICでVOUT閾値を設定し、nRF52840のGPIOに接続
- SC4823のAWAKEは使わず、外部回路でMCUを起こす

---

## ドライバーの実装状態

### PM Suspend（スリープ時）
1. `k_work_cancel_delayable()` でスキャン停止
2. `gpio_pin_set_dt(&sleep_gpio, 1)` で SC4823 を Mode 3 に移行（低消費電力: <1.5µA/IC）

### PM Resume（復帰時）
1. `gpio_pin_set_dt(&sleep_gpio, 0)` で Mode 1 に復帰
2. 5ms 待機
3. `he_calibrate()` で再キャリブレーション
4. スキャン再開

### 以前のAWAKE SENSE設定（現在は削除済み）
PM Suspend で `nrf_gpio_cfg_sense_set(pin, NRF_GPIO_PIN_SENSE_LOW)` を設定していたが、
自動復帰（スリープ直後に勝手に起きる）問題が発生したため削除。

**重要な発見**: `gpio_pin_interrupt_configure_dt(GPIO_INT_LEVEL_ACTIVE)` は使用不可。
nrfx_gpiote の PORT イベント ISR 内で `do...while(latch_pending_read_and_check())` の
無限ループに入り、同じGPIOポート上の他のコールバック（PMW3610等）も巻き込む。
HALレベルの `nrf_gpio_cfg_sense_set()` を使えば ISR は発火しないが、
AWAKE がアイドル時にLOWになる問題が根本的に解決できない限り意味がない。

---

## ピンアサイン（P1ポート、復帰に関連）

| ピン | 機能 | 復帰 |
|------|------|------|
| P1.01 | SC4823 AWAKE (active-low, pull-up, wired-OR) | ❌ 現在無効 |
| P1.03 | エンコーダープッシュ (active-low, pull-up) | ✅ |
| P1.05/P1.07 | EC11 A/B ピン | ❌ edge割り込みのため不可 |
| P1.11 | PMW3610 IRQ (active-low, pull-up) | ✅ |
| P1.12 | SC4823 SLEEP制御 (active-high) | — |

---

## nRF52840 System OFF の制約

- System OFF ではGPIOTE（エッジ割り込み）は無効化される
- **GPIO SENSE（レベル検知）のみ**が復帰トリガーとして機能
- `PIN_CNF.SENSE` フィールドで HIGH/LOW のレベル検知を設定
- DETECT信号 = 全ピンのSENSEマッチのOR → 1つでもマッチすればウェイクアップ
- System OFF 前に GPIO LATCH レジスタをクリアしないと即座に再起動する可能性あり
