# Komyak Eye (コミャク・アイ)

![Komyak Eye](https://via.placeholder.com/640x480.png?text=Komyak+Eye)

An animated digital eye display project for the Waveshare RP2040-LCD-1.28 display. This project creates a realistic-looking animated eye that can move in various patterns.

[日本語版はこちら](#コミャク・アイ)

## Overview

Komyak Eye is an Arduino project that displays an animated eye on a circular LCD. The eye follows different movement patterns and is optimized for smooth animation and efficient rendering on microcontroller hardware.

## Features

- Realistic eye animation with iris movement
- Multiple eye movement patterns:
  - Spiral motion
  - Random movement
  - Horizontal scanning
  - Vertical scanning
  - Fixed staring with micro-movements
- Optimized rendering with delta-drawing (only updating changed pixels)
- Memory optimization options
- Smooth animation with various performance optimizations

## Hardware Requirements

- Waveshare RP2040-LCD-1.28 (or compatible circular LCD)
- Raspberry Pi Pico (RP2040) microcontroller

### Pin Configuration

| LCD Pin | RP2040 Pin |
|---------|------------|
| CS      | 9          |
| DC      | 8          |
| RST     | 12         |
| BL      | 25         |
| SCK     | 10         |
| MOSI    | 11         |

## Software Dependencies

- Arduino IDE
- [LovyanGFX Library](https://github.com/lovyan03/LovyanGFX)

## Installation

1. Install the Arduino IDE
2. Install the LovyanGFX library:
   - In Arduino IDE, go to Sketch > Include Library > Manage Libraries
   - Search for "LovyanGFX" and install it
3. Connect your RP2040 board to your computer
4. Open the KomyakuEye.ino file in Arduino IDE
5. Select the appropriate board from Tools > Board menu
6. Select the correct port from Tools > Port menu
7. Click the Upload button to flash the code to your device

## Configuration Options

The code includes several optimization options that can be enabled or disabled by commenting/uncommenting the corresponding `#define` directives at the top of the file:

```cpp
#define DEBUG                         // Enable debug logging
#define OPTIMIZE_DRAWING              // Enable drawing optimizations
#define OPTIMIZE_MEMORY               // Optimize memory usage
#define USE_DMA                       // Use DMA transfers
#define FRAME_BUFFER                  // Use frame buffer
#define TRIG_TABLE                    // Use trigonometric lookup tables
#define FAST_MATH                     // Use fast math approximations
#define OPTIMIZE_SPI                  // Optimize SPI communication
```

## Eye Parameters

You can customize the eye appearance by modifying these parameters:

```cpp
const int R  = 95;          // eyeball radius
const int r  = 45;          // iris radius
const float B_MIN = 0.60f;  // min b/a (side view)
```

## Movement Patterns

The eye can move in five different patterns:
1. Spiral motion
2. Random movement
3. Left-right scanning
4. Up-down scanning
5. Fixed staring with micro-movements

The probability of each pattern can be adjusted by modifying the `PATTERN_WEIGHTS` array:

```cpp
const int PATTERN_WEIGHTS[] = {20, 10, 10, 10, 50};  // Total should be 100
```

## How It Works

The project uses an optimized rendering approach:
1. The background (red body + white eyeball + black outline) is drawn once during setup
2. During the main loop, only the differences between frames are updated:
   - Areas that were blue in the previous frame but not in the current frame are erased (drawn in white)
   - Areas that are blue in the current frame but weren't in the previous frame are drawn in blue
3. Various optimizations are applied to improve performance:
   - Pre-calculated trigonometric functions
   - Bounding box optimization
   - Batch processing with startWrite/endWrite
   - Fast pixel writing
   - Loop calculation optimization
   - Drawing area optimization

## License

[MIT License](LICENSE)

## Credits

Developed by [Your Name/Organization]

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

---

# コミャク・アイ

![コミャク・アイ](https://via.placeholder.com/640x480.png?text=Komyak+Eye)

Waveshare RP2040-LCD-1.28ディスプレイ用のアニメーションデジタル目玉表示プロジェクトです。このプロジェクトは、様々なパターンで動く、リアルな目玉をディスプレイに表示します。

## 概要

コミャク・アイは、円形LCDにアニメーション目玉を表示するArduinoプロジェクトです。目玉は異なる動きのパターンに従って動き、マイクロコントローラーハードウェア上でスムーズなアニメーションと効率的なレンダリングのために最適化されています。

## 特徴

- 虹彩の動きによるリアルな目のアニメーション
- 複数の目の動きパターン：
  - スパイラル運動
  - ランダムな動き
  - 水平スキャン
  - 垂直スキャン
  - 微小な動きを伴う固定した凝視
- デルタ描画による最適化されたレンダリング（変更されたピクセルのみを更新）
- メモリ最適化オプション
- 様々なパフォーマンス最適化によるスムーズなアニメーション

## ハードウェア要件

- Waveshare RP2040-LCD-1.28（または互換性のある円形LCD）
- Raspberry Pi Pico（RP2040）マイクロコントローラー

### ピン設定

| LCDピン | RP2040ピン |
|---------|------------|
| CS      | 9          |
| DC      | 8          |
| RST     | 12         |
| BL      | 25         |
| SCK     | 10         |
| MOSI    | 11         |

## ソフトウェア依存関係

- Arduino IDE
- [LovyanGFXライブラリ](https://github.com/lovyan03/LovyanGFX)

## インストール方法

1. Arduino IDEをインストールする
2. LovyanGFXライブラリをインストールする：
   - Arduino IDEで、スケッチ > ライブラリをインクルード > ライブラリを管理を選択
   - 「LovyanGFX」を検索してインストール
3. RP2040ボードをコンピュータに接続する
4. Arduino IDEでKomyakuEye.inoファイルを開く
5. ツール > ボードメニューから適切なボードを選択
6. ツール > ポートメニューから正しいポートを選択
7. アップロードボタンをクリックしてコードをデバイスに書き込む

## 設定オプション

コードには、ファイルの先頭にある対応する`#define`ディレクティブをコメント/コメント解除することで有効または無効にできるいくつかの最適化オプションが含まれています：

```cpp
#define DEBUG                         // デバッグログを有効にする
#define OPTIMIZE_DRAWING              // 描画最適化を有効にする
#define OPTIMIZE_MEMORY               // メモリ使用量を最適化する
#define USE_DMA                       // DMA転送を使用する
#define FRAME_BUFFER                  // フレームバッファを使用する
#define TRIG_TABLE                    // 三角関数ルックアップテーブルを使用する
#define FAST_MATH                     // 高速な数学近似を使用する
#define OPTIMIZE_SPI                  // SPI通信を最適化する
```

## 目のパラメータ

以下のパラメータを変更することで、目の外観をカスタマイズできます：

```cpp
const int R  = 95;          // 目玉の半径
const int r  = 45;          // 虹彩の半径
const float B_MIN = 0.60f;  // 最小b/a（側面図）
```

## 動きのパターン

目は5つの異なるパターンで動くことができます：
1. スパイラル運動
2. ランダムな動き
3. 左右スキャン
4. 上下スキャン
5. 微小な動きを伴う固定した凝視

各パターンの確率は、`PATTERN_WEIGHTS`配列を変更することで調整できます：

```cpp
const int PATTERN_WEIGHTS[] = {20, 10, 10, 10, 50};  // 合計が100になるようにする
```

## 動作原理

このプロジェクトは最適化されたレンダリングアプローチを使用しています：
1. 背景（赤い本体 + 白い目玉 + 黒い輪郭）はセットアップ時に一度だけ描画されます
2. メインループでは、フレーム間の差分のみが更新されます：
   - 前のフレームで青だったが現在のフレームではない領域は消去されます（白で描画）
   - 現在のフレームで青だが前のフレームではなかった領域は青で描画されます
3. パフォーマンスを向上させるためにさまざまな最適化が適用されています：
   - 事前計算された三角関数
   - 境界ボックスの最適化
   - startWrite/endWriteによるバッチ処理
   - 高速ピクセル書き込み
   - ループ計算の最適化
   - 描画領域の最適化

## ライセンス

[MITライセンス](LICENSE)

## クレジット

開発者：[あなたの名前/組織]

## 貢献

貢献は歓迎します！お気軽にプルリクエストを提出してください。
