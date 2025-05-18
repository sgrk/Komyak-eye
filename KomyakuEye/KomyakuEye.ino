/********************************************************************* 
 *  RP2040-LCD-1.28  Comyak Eye  ☆  Delta-Draw Debug Build
 *  ─ 変更点 ─
 *    1. 背景（赤胴体＋白目＋外周黒線）を setup() で一度だけ描画
 *    2. loop() では
 *          a) 前フレームと現フレームの差分のみを更新
 *          b) 前フレームにあって現フレームにない部分を WHITE で消す
 *          c) 前フレームになくて現フレームにある部分を BLUE で描く
 *    3. 最適化
 *          a) トリゴノメトリ関数の事前計算
 *          b) 境界ボックスの最適化
 *          c) バッチ処理の実装 (startWrite/endWrite)
 *          d) 高速ピクセル書き込み (writePixel)
 *          e) ループ計算の最適化
 *          f) 描画範囲の最適化
 *  
 *  LovyanGFX版に変更
 *********************************************************************/
#include <LovyanGFX.hpp>
//#define DEBUG                         // ← ログ不要ならコメントアウト
#define OPTIMIZE_DRAWING              // ← 描画最適化を有効にする
#define OPTIMIZE_MEMORY               // ← メモリ使用量を最適化する
#define USE_DMA                       // ← DMA転送を使用する (必要に応じてコメント解除)
#define FRAME_BUFFER                  // ← フレームバッファを使用する
#define TRIG_TABLE                    // ← 三角関数テーブルを使用する
#define FAST_MATH                     // ← 高速な数学関数近似を使用する
#define OPTIMIZE_SPI                  // ← SPI通信を最適化する

/* --- LCD Pinmap (Waveshare RP2040-LCD-1.28) ----------------------- */
#define LCD_CS   9
#define LCD_DC   8
#define LCD_RST 12
#define LCD_BL  25
#define LCD_SCK 10
#define LCD_MOSI 11
const uint32_t SPI_SPEED = 60'000'000;

/* --- LovyanGFX Setup --------------------------------------------- */
class LGFX : public lgfx::LGFX_Device {
public:
  lgfx::Panel_GC9A01      _panel_instance;
  lgfx::Bus_SPI           _bus_instance;
  lgfx::Light_PWM         _light_instance;

  LGFX(void) {
    { // バス設定
      auto cfg = _bus_instance.config();
      cfg.spi_host = 1;              // SPI1を使用
      cfg.spi_mode = 0;              // SPI通信モード
      cfg.freq_write = SPI_SPEED;    // 送信時のSPIクロック
      cfg.freq_read  = 16000000;     // 受信時のSPIクロック
      cfg.pin_sclk = LCD_SCK;        // SCLKピン
      cfg.pin_mosi = LCD_MOSI;       // MOSIピン
      cfg.pin_miso = -1;             // MISOピン（使用しない場合は-1）
      cfg.pin_dc = LCD_DC;           // DCピン
      
      _bus_instance.config(cfg);     // 設定を反映
      _panel_instance.setBus(&_bus_instance);  // バスをパネルにセット
    }

    { // 表示パネル設定
      auto cfg = _panel_instance.config();
      cfg.pin_cs = LCD_CS;           // CSピン
      cfg.pin_rst = LCD_RST;         // RSTピン
      cfg.pin_busy = -1;             // BUSYピン（使用しない場合は-1）
      cfg.panel_width = 240;         // 実際のパネル幅
      cfg.panel_height = 240;        // 実際のパネル高さ
      cfg.offset_x = 0;              // パネルのX方向オフセット
      cfg.offset_y = 0;              // パネルのY方向オフセット
      cfg.offset_rotation = 0;       // 回転方向の調整
      cfg.dummy_read_pixel = 8;      // ピクセル読出し前のダミーリードのビット数
      cfg.dummy_read_bits = 1;       // ビット読出し前のダミーリードのビット数
      cfg.readable = false;          // 読出し可能な場合 true
      cfg.invert = true;             // パネルの明暗が反転してる場合 true
      cfg.rgb_order = false;         // パネルの赤と青が入れ替わってる場合 true
      cfg.dlen_16bit = false;        // 16bitパラレルやSPIでデータ長を16bit単位で送信する場合 true
      cfg.bus_shared = false;         // SDカードとバスを共有する場合 true
      
      _panel_instance.config(cfg);
    }

    { // バックライト設定
      auto cfg = _light_instance.config();
      cfg.pin_bl = LCD_BL;           // バックライトピン
      cfg.invert = false;            // バックライトの明暗が反転してる場合 true
      cfg.freq = 44100;              // PWM周波数
      cfg.pwm_channel = 1;           // 使用するPWMのチャンネル番号
      
      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);  // バックライトをパネルにセット
    }

    setPanel(&_panel_instance);      // パネルをセット
  }
};

LGFX gfx;

/* --- Eye Parameters ---------------------------------------------- */
const int R  = 90;          // eyeball radius
const int r  =  45;          // iris radius
const float B_MIN = 0.75f;   // min b/a (side view)
// Screen dimensions
const int SCREEN_WIDTH = 240;
const int SCREEN_HEIGHT = 240;
// Buffer size for bit-packed buffers (each byte stores 8 pixels)
const int BUFFER_SIZE = (SCREEN_WIDTH * SCREEN_HEIGHT + 7) / 8;
// Center coordinates
const int CENTER_X = 120;
const int CENTER_Y = min(120,R+10);  // Shifted upward by 120-R pixels

/* --- Eye Movement Patterns -------------------------------------- */
// パターン関連の定数
const int PATTERN_COUNT = 2;  // パターンの総数 (0: 移動, 1: じっと見つめる)
const unsigned long PATTERN_CHANGE_INTERVAL = 10000;  // パターン切り替え間隔（ミリ秒）

// パターンの重み（確率）を定義
// パターン0（移動）: 30%, パターン1（じっと見つめる）: 70%
const int PATTERN_WEIGHTS[] = {30, 70};  // 合計100になるようにする

// パターン関連の変数
int currentPattern = 0;  // 現在のパターン（0: 移動, 1: じっと見つめる）
int previousPattern = 0; // 前回のパターン（モード変更時の位置共有用）
unsigned long lastPatternChangeTime = 0;  // 最後にパターンを変更した時間

// 移動パターン用の変数
float targetDistance = 0.0f;  // 目標距離 (3/5*r から R-r*B_MIN の範囲)
float targetAngle = 0.0f;     // 目標角度 (ラジアン)
float currentDistance = 0.0f; // 現在の距離
float currentAngle = 0.0f;    // 現在の角度 (ラジアン)
float distanceDiff = 0.0f;    // 1サイクルあたりの距離の移動量
float angleDiff = 0.0f;       // 1サイクルあたりの角度の移動量
int remainingCycles = 0;      // 目標位置に到達するまでの残りサイクル数
float moveSpeed = 0.0f;       // 移動速度（1サイクルあたりの移動量の計算に使用）
bool needNewTarget = true;    // 新しい目標が必要かどうか

// 共有位置情報（モード変更時に使用）
float sharedPositionX = 0.0f; // 正規化されたX座標 (-1.0〜1.0)
float sharedPositionY = 0.0f; // 正規化されたY座標 (-1.0〜1.0)
bool positionInitialized = false; // 位置が初期化されたかどうか

#ifdef TRIG_TABLE
// Trigonometric lookup tables
const int TRIG_TABLE_SIZE = 360;  // 1度ごとの分解能
float sinTable[TRIG_TABLE_SIZE];
float cosTable[TRIG_TABLE_SIZE];

// 高速な逆正接テーブル (atan2の近似用)
const int ATAN2_TABLE_SIZE = 64;  // 64x64のテーブル (メモリ節約のため)
uint8_t atan2Table[ATAN2_TABLE_SIZE][ATAN2_TABLE_SIZE];

// 角度からテーブルのインデックスを計算（0-359の範囲に正規化）
inline int angleToIndex(float angle) {
  // ラジアンから度に変換し、0-359の範囲に正規化
  int index = (int)(angle * 180.0f / PI) % 360;
  if (index < 0) index += 360;  // 負の角度を正の範囲に変換
  return index;
}

// テーブルから正弦値を取得
inline float fastSin(float angle) {
  return sinTable[angleToIndex(angle)];
}

// テーブルから余弦値を取得
inline float fastCos(float angle) {
  return cosTable[angleToIndex(angle)];
}

// テーブルから高速にatan2を計算
inline float fastAtan2(float y, float x) {
  // 入力値を-1.0〜1.0の範囲に正規化
  float ax = constrain(fabs(x), 0.0f, 1.0f);
  float ay = constrain(fabs(y), 0.0f, 1.0f);
  
  // テーブルのインデックスを計算
  int x_idx = (int)(ax * (ATAN2_TABLE_SIZE-1));
  int y_idx = (int)(ay * (ATAN2_TABLE_SIZE-1));
  
  // テーブルから値を取得 (0-255の範囲)
  uint8_t angle_byte = atan2Table[y_idx][x_idx];
  
  // 0-255から0-PI/2に変換
  float angle = (float)angle_byte * (PI/2) / 255.0f;
  
  // 象限に応じて角度を調整
  if (x < 0 && y >= 0) angle = PI - angle;      // 第2象限
  else if (x < 0 && y < 0) angle = PI + angle;  // 第3象限
  else if (x >= 0 && y < 0) angle = 2*PI - angle; // 第4象限
  
  return angle;
}

// テーブルを初期化
void initTrigTables() {
  // sin/cosテーブルの初期化
  for (int i = 0; i < TRIG_TABLE_SIZE; i++) {
    float angle = i * PI / 180.0f;  // 度からラジアンに変換
    sinTable[i] = sinf(angle);
    cosTable[i] = cosf(angle);
  }
  
  // atan2テーブルの初期化 (第1象限のみ、0-PI/2)
  for (int y = 0; y < ATAN2_TABLE_SIZE; y++) {
    float y_norm = (float)y / (ATAN2_TABLE_SIZE-1);  // 0.0-1.0
    for (int x = 0; x < ATAN2_TABLE_SIZE; x++) {
      float x_norm = (float)x / (ATAN2_TABLE_SIZE-1);  // 0.0-1.0
      if (x_norm < 0.0001f) x_norm = 0.0001f;  // ゼロ除算を防止
      
      // 第1象限のatan2を計算 (0-PI/2)
      float angle = atan2f(y_norm, x_norm);
      
      // 0-PI/2の範囲を0-255にマッピング
      uint8_t angle_byte = (uint8_t)((angle * 255) / (PI/2));
      atan2Table[y][x] = angle_byte;
    }
  }
  
#ifdef DEBUG
  Serial.println(F("Trigonometric tables initialized."));
#endif
}
#endif

/* --- Helper: filled, rotated ellipse ----------------------------- */
void fillRotEllipse(int xc, int yc, int a, int b, float ang, uint32_t col)
{
  const int N = 20;
#ifdef TRIG_TABLE
  float ca = fastCos(ang), sa = fastSin(ang);
#else
  float ca = cosf(ang), sa = sinf(ang);
#endif
  int16_t vx[N], vy[N];
  for (int i=0; i<N; i++) {
    float th = TWO_PI*i/N;
#ifdef TRIG_TABLE
    float x = a*fastCos(th), y = b*fastSin(th);
#else
    float x = a*cosf(th), y = b*sinf(th);
#endif
    vx[i] = xc + (int16_t)(x*ca - y*sa);
    vy[i] = yc + (int16_t)(x*sa + y*ca);
  }
  for (int i=1; i<N-1; i++)
    gfx.fillTriangle(vx[0], vy[0], vx[i], vy[i], vx[i+1], vy[i+1], col);
}

/* --- Helper: check if a point is inside an ellipse --------------- */
bool isInsideEllipse(int x, int y, int xc, int yc, int a, int b, float ang, float ca, float sa)
{
  // Transform point to ellipse coordinate system (using pre-calculated sin/cos)
  float xt = (x - xc) * ca + (y - yc) * sa;
  float yt = -(x - xc) * sa + (y - yc) * ca;
  // Check if inside
  float a_squared = a*a;
  float b_squared = b*b;
  return ((xt*xt)/a_squared + (yt*yt)/b_squared <= 1.0f);
}

/* --- Helper: get pixel buffer for ellipse ------------------------ */
#ifdef OPTIMIZE_MEMORY
void getEllipsePixels(int xc, int yc, int a, int b, float ang, uint8_t* buffer, int width, int height)
{
  // Pre-calculate sin and cos values (optimization)
#ifdef TRIG_TABLE
  float ca = fastCos(ang), sa = fastSin(ang);
#else
  float ca = cosf(ang), sa = sinf(ang);
#endif
  
  // Clear buffer
  clearBuffer(buffer, BUFFER_SIZE);
  
  // Calculate tighter bounding box for the ellipse
  int ea = a;
  int eb = b;
  
  // Calculate the rotated bounding box more precisely
  int dx = (int)(ea * fabs(ca) + eb * fabs(sa)) + 1;
  int dy = (int)(ea * fabs(sa) + eb * fabs(ca)) + 1;
  
  // Define the scan area with the tighter bounding box
  int x_min = max(0, xc - dx);
  int x_max = min(width - 1, xc + dx);
  int y_min = max(0, yc - dy);
  int y_max = min(height - 1, yc + dy);
  
#ifdef OPTIMIZE_DRAWING
  // Pre-calculate squared values for faster ellipse check
  float a_squared = ea * ea;
  float b_squared = eb * eb;
  
  // Use scanline algorithm for better cache locality
  for (int y = y_min; y <= y_max; y++) {
    float y_transformed = (y - yc);
    
    for (int x = x_min; x <= x_max; x++) {
      float x_transformed = (x - xc);
      
      // Transform point to ellipse coordinate system (using pre-calculated sin/cos)
      float xt = x_transformed * ca + y_transformed * sa;
      float yt = -x_transformed * sa + y_transformed * ca;
      
      // Check if inside ellipse using pre-calculated squared values
      if ((xt*xt)/a_squared + (yt*yt)/b_squared <= 1.0f) {
        setPixel(buffer, x, y, width);
      }
    }
  }
#else
  // Original implementation
  for (int y = y_min; y <= y_max; y++) {
    for (int x = x_min; x <= x_max; x++) {
      if (isInsideEllipse(x, y, xc, yc, ea, eb, ang, ca, sa)) {
        setPixel(buffer, x, y, width);
      }
    }
  }
#endif
}
#else
void getEllipsePixels(int xc, int yc, int a, int b, float ang,  bool* buffer, int width, int height)
{
  // Pre-calculate sin and cos values (optimization)
#ifdef TRIG_TABLE
  float ca = fastCos(ang), sa = fastSin(ang);
#else
  float ca = cosf(ang), sa = sinf(ang);
#endif
  
  // Clear buffer
  memset(buffer, 0, width * height * sizeof(bool));
  
  // Calculate tighter bounding box for the ellipse
  int ea = a;
  int eb = b;
  
  // Calculate the rotated bounding box more precisely
  int dx = (int)(ea * fabs(ca) + eb * fabs(sa)) + 1;
  int dy = (int)(ea * fabs(sa) + eb * fabs(ca)) + 1;
  
  // Define the scan area with the tighter bounding box
  int x_min = max(0, xc - dx);
  int x_max = min(width - 1, xc + dx);
  int y_min = max(0, yc - dy);
  int y_max = min(height - 1, yc + dy);
  
#ifdef OPTIMIZE_DRAWING
  // Pre-calculate squared values for faster ellipse check
  float a_squared = ea * ea;
  float b_squared = eb * eb;
  
  // Use scanline algorithm for better cache locality
  for (int y = y_min; y <= y_max; y++) {
    // Pre-calculate the row offset to avoid multiplication in inner loop
    int row_offset = y * width;
    float y_transformed = (y - yc);
    
    for (int x = x_min; x <= x_max; x++) {
      float x_transformed = (x - xc);
      
      // Transform point to ellipse coordinate system (using pre-calculated sin/cos)
      float xt = x_transformed * ca + y_transformed * sa;
      float yt = -x_transformed * sa + y_transformed * ca;
      
      // Check if inside ellipse using pre-calculated squared values
      if ((xt*xt)/a_squared + (yt*yt)/b_squared <= 1.0f) {
        buffer[row_offset + x] = true;
      }
    }
  }
#else
  // Original implementation
  for (int y = y_min; y <= y_max; y++) {
    for (int x = x_min; x <= x_max; x++) {
      if (isInsideEllipse(x, y, xc, yc, ea, eb, ang, ca, sa)) {
        buffer[y * width + x] = true;
      }
    }
  }
#endif
}
#endif

/* --- Globals to remember previous frame -------------------------- */
int  prev_ix = 0, prev_iy = 0;
int  prev_a  = 0, prev_b  = 0;
float prev_phi = 0.0f;

#ifdef OPTIMIZE_MEMORY
// Use bit-packed buffers to save memory (8x less memory usage)
// Each byte stores 8 pixels
uint8_t* currentIrisPixels = NULL;
uint8_t* previousIrisPixels = NULL;

// Helper functions for bit-packed buffers
inline void setPixel(uint8_t* buffer, int x, int y, int width) {
  int index = (y * width + x);
  buffer[index / 8] |= (1 << (index % 8));
}

inline bool getPixel(uint8_t* buffer, int x, int y, int width) {
  int index = (y * width + x);
  return (buffer[index / 8] & (1 << (index % 8))) != 0;
}

inline void clearBuffer(uint8_t* buffer, int size) {
  memset(buffer, 0, size);
}
#else
// Original boolean buffers
bool* currentIrisPixels = NULL;
bool* previousIrisPixels = NULL;
#endif

#ifdef FRAME_BUFFER
// Frame buffer for calculating the entire frame before writing to display
uint16_t* frameBuffer = NULL;

// 変更されたピクセルの範囲を追跡するための変数
int dirty_x_min = SCREEN_WIDTH;
int dirty_x_max = 0;
int dirty_y_min = SCREEN_HEIGHT;
int dirty_y_max = 0;
bool frameBufferDirty = false;
#endif

#ifdef FAST_MATH
// 高速な平方根近似関数
inline float fastSqrt(float x) {
  // 平方根の高速近似 (精度は低いが、視覚的には問題ない)
  union {
    float f;
    uint32_t i;
  } conv;
  
  conv.f = x;
  conv.i = 0x5f3759df - (conv.i >> 1);
  conv.f = conv.f * (1.5f - 0.5f * x * conv.f * conv.f);
  return x * conv.f;
}
#endif

#ifdef USE_DMA
// DMA buffer for faster transfers (if supported by the display)
uint16_t* dmaBuffer = NULL;
#endif

/* --- Pattern Selection Function ---------------------------------- */
// 新しいパターンを重み付き確率で選択する関数
void selectRandomPattern() {
  // 現在のパターンを保存
  previousPattern = currentPattern;
  
  // 0〜99のランダムな値を生成
  int r = random(100);
  
  // 累積確率を計算して、どの範囲に入るかを判定
  int cumulativeWeight = 0;
  for (int i = 0; i < PATTERN_COUNT; i++) {
    cumulativeWeight += PATTERN_WEIGHTS[i];
    if (r < cumulativeWeight) {
      currentPattern = i;
      break;
    }
  }
  
#ifdef DEBUG
  Serial.print(F("Pattern changed to: "));
  Serial.println(currentPattern);
  Serial.print(F("(Random value: "));
  Serial.print(r);
  Serial.println(F(")"));
#endif
}

/* --- Pattern Movement Functions ---------------------------------- */
// パターン0: 移動パターン（目標位置に向かって移動）
void calculateMovingPattern(uint32_t now, uint32_t t0, float &dx, float &dy) {
  static uint32_t lastTargetChangeTime = 0;
  
  // パターン変更直後に現在位置を初期化
  if (previousPattern != currentPattern && positionInitialized) {
    // 共有された位置情報から現在の距離と角度を計算
    float dx_actual = (R - r*B_MIN) * sharedPositionX;
    float dy_actual = (R - r*B_MIN) * sharedPositionY;
    
    // 極座標に変換
    currentDistance = sqrt(dx_actual*dx_actual + dy_actual*dy_actual);
    if (currentDistance > 0) {
      currentAngle = atan2(dy_actual, dx_actual);
    } else {
      // 中心にいる場合は適当な角度から開始
      currentAngle = 0;
      currentDistance = 3.0f * r / 5.0f; // 最小距離から開始
    }
    
    // 新しい目標を設定するフラグを立てる
    needNewTarget = true;
    
    // パターン変更処理完了
    previousPattern = currentPattern;
    
#ifdef DEBUG
    Serial.println(F("Moving pattern initialized with shared position:"));
    Serial.print(F("X: ")); Serial.print(sharedPositionX);
    Serial.print(F(", Y: ")); Serial.println(sharedPositionY);
#endif
  }
  
  // 新しい目標が必要か、または一定時間経過したら新しい目標を設定
  if (needNewTarget || now - lastTargetChangeTime > 10000) {
    // 目標距離を3/5*rからR-r*B_MINの範囲でランダムに設定
    float min_distance = 3.0f * r / 5.0f;
    float max_distance = R - r * B_MIN;
    targetDistance = min_distance + random(1000) / 1000.0f * (max_distance - min_distance);
    
    // 目標角度を0〜2πの範囲でランダムに設定
    targetAngle = random(1000) / 1000.0f * TWO_PI;
    
    // 移動速度をランダムに設定（0.01〜0.05の範囲）
    moveSpeed = 0.01f + random(80) / 1000.0f;
    
    // 目標位置までの距離と角度の差を計算
    float totalDistanceDiff = targetDistance - currentDistance;
    float totalAngleDiff = targetAngle - currentAngle;
    
    // 角度差を-πからπの範囲に正規化（最短経路を計算）
    float shortestAngleDiff = totalAngleDiff;
    while (shortestAngleDiff > PI) shortestAngleDiff -= TWO_PI;
    while (shortestAngleDiff < -PI) shortestAngleDiff += TWO_PI;
    
    // 長い方の経路を計算
    float longerAngleDiff = (shortestAngleDiff > 0) ? shortestAngleDiff - TWO_PI : shortestAngleDiff + TWO_PI;
    
    // 3/4の確率で最短経路、1/4の確率で長い方の経路を選択
    if (random(4) < 3) {
      // 75%の確率で最短経路を選択
      totalAngleDiff = shortestAngleDiff;
#ifdef DEBUG
      Serial.println(F("Selected shortest path for rotation"));
#endif
    } else {
      // 25%の確率で長い方の経路を選択
      totalAngleDiff = longerAngleDiff;
#ifdef DEBUG
      Serial.println(F("Selected longer path for rotation"));
#endif
    }
    
    // 目標位置に到達するまでのサイクル数を計算
    // 距離と角度の両方が目標に到達するように調整
    float distanceSteps = fabs(totalDistanceDiff) / moveSpeed;
    float angleSteps = fabs(totalAngleDiff) / moveSpeed;
    remainingCycles = max((int)distanceSteps, (int)angleSteps);
    
    // 最小サイクル数を設定（あまりに短いと不自然な動きになるため）
    remainingCycles = max(remainingCycles, 20);
    
    // 1サイクルあたりの移動量を計算（一定速度になるように）
    if (remainingCycles > 0) {
      distanceDiff = totalDistanceDiff / remainingCycles;
      angleDiff = totalAngleDiff / remainingCycles;
    } else {
      distanceDiff = 0.0f;
      angleDiff = 0.0f;
    }
    
    // 現在位置がまだ初期化されていない場合は初期化
    if (!positionInitialized) {
      // 現在位置を取得（dx, dyから極座標に変換）
      float dx_norm = dx;
      float dy_norm = dy;
      
      // 正規化された座標から実際の距離と角度を計算
      float dx_actual = (R - r*B_MIN) * dx_norm;
      float dy_actual = (R - r*B_MIN) * dy_norm;
      
      // 極座標に変換
      currentDistance = sqrt(dx_actual*dx_actual + dy_actual*dy_actual);
      if (currentDistance > 0) {
        currentAngle = atan2(dy_actual, dx_actual);
      } else {
        // 中心にいる場合は適当な角度から開始
        currentAngle = 0;
        currentDistance = min_distance; // 最小距離から開始
      }
      
      positionInitialized = true;
    }
    
    needNewTarget = false;
    lastTargetChangeTime = now;
    
#ifdef DEBUG
    Serial.println(F("New target set:"));
    Serial.print(F("Distance: ")); Serial.println(targetDistance);
    Serial.print(F("Angle: ")); Serial.println(targetAngle);
    Serial.print(F("Speed: ")); Serial.println(moveSpeed);
#endif
  }
  
  // 目標位置に到達するまでの残りサイクル数がある場合は移動を続ける
  if (remainingCycles > 0) {
    // 一定の速度で角度と距離を更新
    currentAngle += angleDiff;
    currentDistance += distanceDiff;
    
    // 角度を0〜2πの範囲に正規化
    while (currentAngle < 0) currentAngle += TWO_PI;
    while (currentAngle >= TWO_PI) currentAngle -= TWO_PI;
    
    // 残りサイクル数を減らす
    remainingCycles--;
    
    // 残りサイクル数が0になったら目標に到達したと判断
    if (remainingCycles <= 0) {
      // 正確に目標位置に設定（浮動小数点の誤差を防ぐため）
      currentAngle = targetAngle;
      currentDistance = targetDistance;
      needNewTarget = true;
    }
  } else {
    // 既に目標に到達している場合は新しい目標を設定
    needNewTarget = true;
  }
  
  // 極座標から直交座標に変換
#ifdef TRIG_TABLE
  float cos_angle = fastCos(currentAngle);
  float sin_angle = fastSin(currentAngle);
#else
  float cos_angle = cosf(currentAngle);
  float sin_angle = sinf(currentAngle);
#endif
  
  // 正規化された座標に変換（-1.0〜1.0の範囲）
  dx = (currentDistance * cos_angle) / (R - r*B_MIN);
  dy = (currentDistance * sin_angle) / (R - r*B_MIN);
  
  // 値の範囲を制限
  dx = constrain(dx, -1.0f, 1.0f);
  dy = constrain(dy, -1.0f, 1.0f);
}

// パターン1: じっと見つめる
void calculateStarePattern(uint32_t now, uint32_t t0, float &dx, float &dy) {
  // 固定位置を見つめる（少しだけランダムな微動あり）
  static float baseX = 0, baseY = 0;
  static uint32_t lastChangeTime = 0;
  static bool baseInitialized = false;
  
  // パターン変更直後に現在位置を初期化
  if (previousPattern != currentPattern && positionInitialized) {
    // 共有された位置を基準位置として使用
    baseX = sharedPositionX;
    baseY = sharedPositionY;
    baseInitialized = true;
    
    // パターン変更処理完了
    previousPattern = currentPattern;
    
#ifdef DEBUG
    Serial.println(F("Stare pattern initialized with shared position:"));
    Serial.print(F("X: ")); Serial.print(baseX);
    Serial.print(F(", Y: ")); Serial.println(baseY);
#endif
  }
  
  // 基準位置がまだ初期化されていない場合は初期化
  if (!baseInitialized) {
    // ランダムな基準位置を設定（あまり端に寄りすぎないように）
    baseX = random(-70, 71) / 100.0f;  // -0.7〜0.7の範囲
    baseY = random(-70, 71) / 100.0f;
    baseInitialized = true;
    
    positionInitialized = true;
  }
  
  // 微小なランダムな動き（まばたきや微動を表現）
  float microMovementX = random(-5, 6) / 1000.0f;  // ±0.005の微動
  float microMovementY = random(-5, 6) / 1000.0f;
  
  dx = baseX + microMovementX;
  dy = baseY + microMovementY;
}

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  while(!Serial);
  Serial.println(F("Delta-draw build with LovyanGFX; init LCD…"));
#endif

  // ランダム関数の初期化（未接続のアナログピンの値を使用）
  randomSeed(analogRead(0));
  
  // 最初のパターンをランダムに選択
  selectRandomPattern();
  lastPatternChangeTime = millis();
  
#ifdef DEBUG
  Serial.print(F("Initial pattern: "));
  Serial.println(currentPattern);
#endif

  gfx.init();
  gfx.setBrightness(255);  // バックライト最大
  
#ifdef TRIG_TABLE
  // 三角関数テーブルを初期化
  initTrigTables();
#endif

  /* === STATIC BACKGROUND (draw once) ============================ */
  const int outerR = 230;             // red body radius (115*2)
  gfx.fillScreen(TFT_RED);            // red body
  gfx.fillCircle(CENTER_X, CENTER_Y, R, TFT_WHITE);  // eyeball

#ifdef FRAME_BUFFER
  // Allocate frame buffer for calculating the entire frame
  frameBuffer = (uint16_t*)malloc(SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint16_t));
  if (frameBuffer) {
    // Initialize frame buffer with background colors
    for (int y = 0; y < SCREEN_HEIGHT; y++) {
      for (int x = 0; x < SCREEN_WIDTH; x++) {
        // Calculate distance from center
        int dx = x - CENTER_X;
        int dy = y - CENTER_Y;
        int dist_squared = dx*dx + dy*dy;
        
        // Set color based on position (red body, white eyeball, black outline)
        if (dist_squared <= R*R) {
          frameBuffer[y * SCREEN_WIDTH + x] = TFT_WHITE;  // Inside eyeball
        }  else {
          frameBuffer[y * SCREEN_WIDTH + x] = TFT_RED;    // Red body
        }
      }
    }
#ifdef DEBUG
    Serial.println(F("Frame buffer initialized."));
#endif
  }
#endif

#ifdef OPTIMIZE_MEMORY
  // Allocate memory for bit-packed pixel buffers (8x less memory)
  currentIrisPixels = (uint8_t*)malloc(BUFFER_SIZE);
  previousIrisPixels = (uint8_t*)malloc(BUFFER_SIZE);
  
  // Initialize buffers
  if (currentIrisPixels && previousIrisPixels) {
    clearBuffer(previousIrisPixels, BUFFER_SIZE);
  }
  
#ifdef DEBUG
  Serial.println(F("Memory-optimized buffers allocated."));
#endif
#else
  // Allocate memory for original boolean pixel buffers
  currentIrisPixels = (bool*)malloc(SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(bool));
  previousIrisPixels = (bool*)malloc(SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(bool));
  
  // Initialize buffers
  if (currentIrisPixels && previousIrisPixels) {
    memset(previousIrisPixels, 0, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(bool));
  }
#endif

#ifdef USE_DMA
  // Allocate DMA buffer if using DMA
  dmaBuffer = (uint16_t*)malloc(SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint16_t));
  if (dmaBuffer) {
    // Initialize DMA buffer with background colors
    for (int y = 0; y < SCREEN_HEIGHT; y++) {
      for (int x = 0; x < SCREEN_WIDTH; x++) {
        // Calculate distance from center
        int dx = x - CENTER_X;
        int dy = y - CENTER_Y;
        int dist_squared = dx*dx + dy*dy;
        
        // Set color based on position (red body, white eyeball, black outline)
        if (dist_squared <= R*R) {
          dmaBuffer[y * SCREEN_WIDTH + x] = TFT_WHITE;  // Inside eyeball
        } else {
          dmaBuffer[y * SCREEN_WIDTH + x] = TFT_RED;    // Red body
        }
      }
    }
#ifdef DEBUG
    Serial.println(F("DMA buffer initialized."));
#endif
  }
#endif
  
#ifdef DEBUG
  Serial.println(F("Background drawn."));
#endif
}

void loop()
{
  static uint32_t t0 = millis(), tPrevLog = t0;
  static uint32_t frame = 0;
  uint32_t now = millis();

#ifdef OPTIMIZE_DRAWING
  // Skip frames if we're running too fast (to avoid excessive drawing)
  static uint32_t lastFrameTime = 0;
  const uint32_t MIN_FRAME_TIME = 16; // ~60fps max
  
  if (now - lastFrameTime < MIN_FRAME_TIME) {
    return; // Skip this frame to maintain reasonable frame rate
  }
  lastFrameTime = now;
#endif

  /* --- パターン切り替えの処理 ----------------------------------- */
  // パターンに応じて切り替え条件を変更
  if (currentPattern == 1) {
    // パターン1（じっと見つめる）: 10秒経過でパターン選択に移る
    if (now - lastPatternChangeTime > PATTERN_CHANGE_INTERVAL) {
      selectRandomPattern();
      lastPatternChangeTime = now;
    }
  } else if (currentPattern == 0) {
    // パターン0（移動）: 目標位置に到着したときのみパターン変更可能
    if (needNewTarget && now - lastPatternChangeTime > PATTERN_CHANGE_INTERVAL) {
      selectRandomPattern();
      lastPatternChangeTime = now;
    }
  }

  /* --- 現在のパターンに基づいて動きを計算 ----------------------- */
  float dx = 0, dy = 0;
  
  // 現在のパターンに応じて動きを計算
  switch (currentPattern) {
    case 0: // 移動パターン
      calculateMovingPattern(now, t0, dx, dy);
      break;
    case 1: // じっと見つめるパターン
      calculateStarePattern(now, t0, dx, dy);
      break;
    default: // 念のためデフォルトはじっと見つめる
      calculateStarePattern(now, t0, dx, dy);
      break;
  }
  
  // 現在位置を共有変数に保存（モード変更時に使用）
  sharedPositionX = dx;
  sharedPositionY = dy;
  // Optimize square root calculation
  float dx_squared = dx*dx;
  float dy_squared = dy*dy;
#ifdef FAST_MATH
  float dz = fastSqrt(max(0.0f, 1.0f - dx_squared - dy_squared));
#else
  float dz = sqrtf(max(0.0f, 1.0f - dx_squared - dy_squared));
#endif

  // Calculate initial iris position
  int ix = CENTER_X + (int)((R - r*B_MIN) * dx);
  int iy = CENTER_Y + (int)((R - r*B_MIN) * dy);
  
  // Ensure iris is at least 3/5*r away from eyeball center
  float min_distance = 3.0f * r / 5.0f;  // Minimum required distance (3/5*r)
  float dx_center = ix - CENTER_X;
  float dy_center = iy - CENTER_Y;
  float current_distance = sqrt(dx_center*dx_center + dy_center*dy_center);
  
  // If iris is too close to center, move it outward
  if (current_distance < min_distance && current_distance > 0) {
    float scale_factor = min_distance / current_distance;
    ix = CENTER_X + (int)(dx_center * scale_factor);
    iy = CENTER_Y + (int)(dy_center * scale_factor);
  }
  
  int a  = r;
  int b  = (int)(r * (B_MIN + (1.0f - B_MIN) * dz));
#ifdef TRIG_TABLE
  // テーブル参照によるatan2の高速化
  float phi = fastAtan2(-dx, dy);
#else
  float phi = atan2f(-dx, dy);
#endif
  
#ifdef OPTIMIZE_DRAWING
  // Skip redraw if the eye position hasn't changed significantly
  static int last_ix = 0, last_iy = 0;
  static int last_a = 0, last_b = 0;
  static float last_phi = 0.0f;
  
  const int MIN_MOVE_THRESHOLD = 1; // Minimum pixel movement to trigger redraw
  const float MIN_ANGLE_THRESHOLD = 0.01f; // Minimum angle change to trigger redraw
  
  bool significant_change = 
    abs(ix - last_ix) > MIN_MOVE_THRESHOLD ||
    abs(iy - last_iy) > MIN_MOVE_THRESHOLD ||
    abs(a - last_a) > MIN_MOVE_THRESHOLD ||
    abs(b - last_b) > MIN_MOVE_THRESHOLD ||
    fabs(phi - last_phi) > MIN_ANGLE_THRESHOLD;
    
  if (!significant_change && frame > 0) {
    frame++;
    return; // Skip redraw if no significant change
  }
  
  last_ix = ix; last_iy = iy;
  last_a = a; last_b = b;
  last_phi = phi;
#endif

  // Check if memory allocation was successful
  if (!currentIrisPixels || !previousIrisPixels) {
    // Fallback to original method if memory allocation failed
    if (frame > 0) {
      fillRotEllipse(prev_ix, prev_iy, prev_a, prev_b, prev_phi, TFT_WHITE);
    }
    fillRotEllipse(ix, iy, a, b, phi, TFT_BLUE);
  } else {
#ifdef FRAME_BUFFER
    if (frameBuffer) {
      // Calculate the bounding box for the current and previous iris
      int prev_dx = (int)(prev_a * 2.0f); // Increase margin to ensure all previous pixels are covered
      int prev_dy = (int)(prev_b * 2.0f);
      int curr_dx = (int)(a * 2.0f);
      int curr_dy = (int)(b * 2.0f);
      
      // Combined bounding box that covers both previous and current positions
      int x_min = max(0, min(prev_ix, ix) - max(prev_dx, curr_dx) - 5); // Add extra margin
      int x_max = min(SCREEN_WIDTH-1, max(prev_ix, ix) + max(prev_dx, curr_dx) + 5);
      int y_min = max(0, min(prev_iy, iy) - max(prev_dy, curr_dy) - 5);
      int y_max = min(SCREEN_HEIGHT-1, max(prev_iy, iy) + max(prev_dy, curr_dy) + 5);
      
      // Pre-calculate eyeball boundary check values
      int r_squared = R*R;
      
      // Clear current buffers before drawing new positions
#ifdef OPTIMIZE_MEMORY
      clearBuffer(currentIrisPixels, BUFFER_SIZE);
#else
      memset(currentIrisPixels, 0, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(bool));
#endif

      // Get current frame pixels
      getEllipsePixels(ix, iy, a, b, phi, currentIrisPixels, SCREEN_WIDTH, SCREEN_HEIGHT);
      
      // Update frame buffer with new eye position (only within bounding box)
      for (int y = y_min; y <= y_max; y++) {
#ifdef OPTIMIZE_MEMORY
        for (int x = x_min; x <= x_max; x++) {
          // Check if this pixel is inside the eyeball
          int dx_eye = x - CENTER_X;
          int dy_eye = y - CENTER_Y;
          if (dx_eye*dx_eye + dy_eye*dy_eye > r_squared) continue;
          
          bool prev_blue = getPixel(previousIrisPixels, x, y, SCREEN_WIDTH);
          bool curr_blue = getPixel(currentIrisPixels, x, y, SCREEN_WIDTH);
          
          // Only update frame buffer if there's a change
          if (prev_blue != curr_blue) {
            frameBuffer[y * SCREEN_WIDTH + x] = curr_blue ? TFT_BLUE : TFT_WHITE;
            
            // Track dirty region
            frameBufferDirty = true;
            dirty_x_min = min(dirty_x_min, x);
            dirty_x_max = max(dirty_x_max, x);
            dirty_y_min = min(dirty_y_min, y);
            dirty_y_max = max(dirty_y_max, y);
          }
        }
#else
        int row_offset = y * SCREEN_WIDTH;
        for (int x = x_min; x <= x_max; x++) {
          int index = row_offset + x;
          
          // Check if this pixel is inside the eyeball
          int dx_eye = x - CENTER_X;
          int dy_eye = y - CENTER_Y;
          if (dx_eye*dx_eye + dy_eye*dy_eye > r_squared) continue;
          
          bool prev_blue = previousIrisPixels[index];
          bool curr_blue = currentIrisPixels[index];
          
          // Only update frame buffer if there's a change
          if (prev_blue != curr_blue) {
            frameBuffer[index] = curr_blue ? TFT_BLUE : TFT_WHITE;
            
            // Track dirty region
            frameBufferDirty = true;
            dirty_x_min = min(dirty_x_min, x);
            dirty_x_max = max(dirty_x_max, x);
            dirty_y_min = min(dirty_y_min, y);
            dirty_y_max = max(dirty_y_max, y);
          }
        }
#endif
      }
      
#ifdef OPTIMIZE_SPI
      // フレームバッファが変更された場合のみ更新
      if (frameBufferDirty) {
        // 変更された領域のみを更新
        int update_x_min = max(x_min, dirty_x_min);
        int update_x_max = min(x_max, dirty_x_max);
        int update_y_min = max(y_min, dirty_y_min);
        int update_y_max = min(y_max, dirty_y_max);
        
        // 有効な更新領域がある場合のみ描画
        if (update_x_min <= update_x_max && update_y_min <= update_y_max) {
          gfx.startWrite();
          
          // 複数行をまとめて更新するために、より大きなウィンドウを設定
          gfx.setAddrWindow(update_x_min, update_y_min, 
                           update_x_max - update_x_min + 1, 
                           update_y_max - update_y_min + 1);
          
          // 行ごとにピクセルを書き込む
          for (int y = update_y_min; y <= update_y_max; y++) {
            gfx.writePixels(&frameBuffer[y * SCREEN_WIDTH + update_x_min], 
                           update_x_max - update_x_min + 1, true);
          }
          
          gfx.endWrite();
        }
        
        // ダーティフラグをリセット
        frameBufferDirty = false;
        dirty_x_min = SCREEN_WIDTH;
        dirty_x_max = 0;
        dirty_y_min = SCREEN_HEIGHT;
        dirty_y_max = 0;
      }
#else
      // 元の実装: 行ごとに更新
      gfx.startWrite();
      
      // Only update the region within the bounding box for efficiency
      for (int y = y_min; y <= y_max; y++) {
        // Set the address window for this row (more efficient than individual pixel writes)
        gfx.setAddrWindow(x_min, y, x_max - x_min + 1, 1);
        
        // Write the entire row at once
        gfx.writePixels(&frameBuffer[y * SCREEN_WIDTH + x_min], x_max - x_min + 1, true);
      }
      
      gfx.endWrite();
#endif
    } else {
      // Fallback if frame buffer allocation failed
      // Clear current buffers before drawing new positions
#ifdef OPTIMIZE_MEMORY
      clearBuffer(currentIrisPixels, BUFFER_SIZE);
#else
      memset(currentIrisPixels, 0, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(bool));
#endif

      // Get current frame pixels
      getEllipsePixels(ix, iy, a, b, phi,  currentIrisPixels, SCREEN_WIDTH, SCREEN_HEIGHT);
      
      // Calculate the bounding box for the current and previous iris
      int prev_dx = (int)(prev_a * 2.0f);
      int prev_dy = (int)(prev_b * 2.0f);
      int curr_dx = (int)(a * 2.0f);
      int curr_dy = (int)(b * 2.0f);
      
      // Combined bounding box
      int x_min = max(0, min(prev_ix, ix) - max(prev_dx, curr_dx) - 5);
      int x_max = min(SCREEN_WIDTH-1, max(prev_ix, ix) + max(prev_dx, curr_dx) + 5);
      int y_min = max(0, min(prev_iy, iy) - max(prev_dy, curr_dy) - 5);
      int y_max = min(SCREEN_HEIGHT-1, max(prev_iy, iy) + max(prev_dy, curr_dy) + 5);
      
      // Pre-calculate eyeball boundary check values
      int r_squared = R*R;
      
      // Use startWrite/endWrite for batch processing
      gfx.startWrite();
      
      // Update only the differences within the combined bounding box
      for (int y = y_min; y <= y_max; y++) {
#ifdef OPTIMIZE_MEMORY
        for (int x = x_min; x <= x_max; x++) {
          // Check if this pixel is inside the eyeball
          int dx_eye = x - CENTER_X;
          int dy_eye = y - CENTER_Y;
          if (dx_eye*dx_eye + dy_eye*dy_eye > r_squared) continue;
          
          bool prev_blue = getPixel(previousIrisPixels, x, y, SCREEN_WIDTH);
          bool curr_blue = getPixel(currentIrisPixels, x, y, SCREEN_WIDTH);
          
          // Only update if there's a change
          if (prev_blue != curr_blue) {
            gfx.writePixel(x, y, curr_blue ? TFT_BLUE : TFT_WHITE);
          }
        }
#else
        int row_offset = y * SCREEN_WIDTH;
        for (int x = x_min; x <= x_max; x++) {
          int index = row_offset + x;
          
          // Check if this pixel is inside the eyeball
          int dx_eye = x - CENTER_X;
          int dy_eye = y - CENTER_Y;
          if (dx_eye*dx_eye + dy_eye*dy_eye > r_squared) continue;
          
          bool prev_blue = previousIrisPixels[index];
          bool curr_blue = currentIrisPixels[index];
          
          // Only update if there's a change
          if (prev_blue != curr_blue) {
            gfx.writePixel(x, y, curr_blue ? TFT_BLUE : TFT_WHITE);
          }
        }
#endif
      }
      
      gfx.endWrite();
    }
#else
    // Original implementation without frame buffer
    // Clear current buffers before drawing new positions
#ifdef OPTIMIZE_MEMORY
    clearBuffer(currentIrisPixels, BUFFER_SIZE);
#else
    memset(currentIrisPixels, 0, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(bool));
#endif

    // Get current frame pixels
    getEllipsePixels(ix, iy, a, b, phi, currentIrisPixels, SCREEN_WIDTH, SCREEN_HEIGHT);
    
    // Calculate the bounding box for the current and previous iris
    int prev_dx = (int)(prev_a * 2.0f);
    int prev_dy = (int)(prev_b * 2.0f);
    int curr_dx = (int)(a * 2.0f);
    int curr_dy = (int)(b * 2.0f);
    
    // Combined bounding box
    int x_min = max(0, min(prev_ix, ix) - max(prev_dx, curr_dx) - 5);
    int x_max = min(SCREEN_WIDTH-1, max(prev_ix, ix) + max(prev_dx, curr_dx) + 5);
    int y_min = max(0, min(prev_iy, iy) - max(prev_dy, curr_dy) - 5);
    int y_max = min(SCREEN_HEIGHT-1, max(prev_iy, iy) + max(prev_dy, curr_dy) + 5);
    
    // Pre-calculate eyeball boundary check values
    int r_squared = R*R;
    
    // Use startWrite/endWrite for batch processing
    gfx.startWrite();
    
    // Update only the differences within the combined bounding box
    for (int y = y_min; y <= y_max; y++) {
#ifdef OPTIMIZE_MEMORY
      for (int x = x_min; x <= x_max; x++) {
        // Check if this pixel is inside the eyeball
        int dx_eye = x - CENTER_X;
        int dy_eye = y - CENTER_Y;
        if (dx_eye*dx_eye + dy_eye*dy_eye > r_squared) continue;
        
        bool prev_blue = getPixel(previousIrisPixels, x, y, SCREEN_WIDTH);
        bool curr_blue = getPixel(currentIrisPixels, x, y, SCREEN_WIDTH);
        
        // Only update if there's a change
        if (prev_blue != curr_blue) {
          // Use writePixel instead of drawPixel for better performance
          gfx.writePixel(x, y, curr_blue ? TFT_BLUE : TFT_WHITE);
        }
      }
#else
      // Pre-calculate the row offset to avoid multiplication in inner loop
      int row_offset = y * SCREEN_WIDTH;
      
      for (int x = x_min; x <= x_max; x++) {
        int index = row_offset + x;
        
        // Check if this pixel is inside the eyeball
        int dx_eye = x - CENTER_X;
        int dy_eye = y - CENTER_Y;
        if (dx_eye*dx_eye + dy_eye*dy_eye > r_squared) continue;
        
        bool prev_blue = previousIrisPixels[index];
        bool curr_blue = currentIrisPixels[index];
        
        // Only update if there's a change
        if (prev_blue != curr_blue) {
          // Use writePixel instead of drawPixel for better performance
          gfx.writePixel(x, y, curr_blue ? TFT_BLUE : TFT_WHITE);
        }
      }
#endif
    }
    
    gfx.endWrite();
#endif
    
    // Swap buffers for next frame
#ifdef OPTIMIZE_MEMORY
    uint8_t* tempIris = previousIrisPixels;
    previousIrisPixels = currentIrisPixels;
    currentIrisPixels = tempIris;
#else
    bool* tempIris = previousIrisPixels;
    previousIrisPixels = currentIrisPixels;
    currentIrisPixels = tempIris;
#endif
  }

  /* --- remember for next frame ---------------------------------- */
  prev_ix = ix; prev_iy = iy; prev_a = a; prev_b = b; prev_phi = phi;
  frame++;

#ifdef DEBUG
  if (now - tPrevLog >= 1000) {      // 1-sec log
    Serial.print(F("t=")); Serial.print(now);
    Serial.print(F("  fps=")); Serial.println(frame);
    frame = 0; tPrevLog = now;
  }
#endif
}

// Add cleanup function to Arduino lifecycle
void cleanup() {
  // Free allocated memory
  if (currentIrisPixels) free(currentIrisPixels);
  if (previousIrisPixels) free(previousIrisPixels);
  
#ifdef FRAME_BUFFER
  if (frameBuffer) free(frameBuffer);
#endif
  
#ifdef USE_DMA
  if (dmaBuffer) free(dmaBuffer);
#endif
}
