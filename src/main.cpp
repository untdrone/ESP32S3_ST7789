/*
 * ESP32-S3FN8  +  ST7789 (FSPI, LovyanGFX)  +  MCP3208 (HSPI)
 *
 *   核心 0：界面任务（LovyanGFX 中文界面：ADC 页 + 右摇杆操作的设置菜单）
 *   核心 1：ADC 采集任务（MCP3208 通道 0~4，过采样平均）
 *
 * 所有硬件参数（屏幕 + ADC）都在本文件顶部配置，platformio.ini 里没有屏幕参数。
 * 两个核心之间用长度为 1 的 FreeRTOS 队列（“邮箱”）传递最新一帧数据。
 * 中文字库由 tools/make_fonts.py 生成到 include/ 目录。
 */

#include <Arduino.h>
#include <SPI.h>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#include "font_cn20.h"    // 20px 中文 + ASCII（标题、标签、原始值）
#include "font_num30.h"   // 30px 粗体数字（电压，行高 >= 40 的布局）
#include "font_num24.h"   // 24px 粗体数字（电压，170x320 横屏的紧凑布局）

// ==========================================================
//                     硬件参数配置
// ==========================================================

// 📺 硬件总线 1：屏幕专属引脚 (走 FSPI = SPI2_HOST 硬件控制器)
#define LCD_SCLK      1
#define LCD_MOSI      2
#define LCD_CS        9
#define LCD_DC        12
#define LCD_RES       5
#define LCD_BL        6

// 📺 屏幕参数
#define LCD_SPI_HOST    SPI2_HOST   // FSPI
#define LCD_SPI_MODE    0           // 无 CS 引脚的模块一般要改成 3
#define LCD_FREQ_WRITE  40000000    // 40MHz；花屏就降到 26666666 或 20000000，稳定的话可试 80000000
#define LCD_WIDTH       170         // 屏幕物理宽度（1.9 寸 170x320）
#define LCD_HEIGHT      320         // 屏幕物理高度
#define LCD_OFFSET_X    35          // 170 宽的屏在 ST7789 240 宽显存中居中：(240-170)/2 = 35
#define LCD_OFFSET_Y    0
#define LCD_INVERT      true        // 颜色反相（黑底变白底）就改成 false
#define LCD_RGB_ORDER   false       // 红蓝颠倒就改成 true
#define LCD_ROTATION    3           // 0~3：0=竖屏，3=逆时针 90° 横屏；方向反了改成 1
                                    // 界面会按横屏 / 竖屏自动切换布局
#define LCD_BRIGHTNESS  255         // 背光亮度 0~255（PWM）

// 🎛️ 硬件总线 2：ADC 专属引脚 (走 HSPI = SPI3 硬件控制器)
#define ADC_SCLK      7
#define ADC_MOSI      8
#define ADC_MISO      4
#define ADC_CS        10

#define ADC_VREF      3300          // mV，MCP3208 VREF 引脚实测电压
#define ADC_CLK       1000000       // ADC 保持 1.0MHz 的稳定通信频率

// ---------------- 采集 / 显示参数 ----------------
#define ADC_CH_COUNT   5            // 显示通道 0~4
#define ADC_OVERSAMPLE 8            // 每通道过采样平均次数
#define ADC_PERIOD_MS  20           // 采集周期（50Hz）
#define LCD_PERIOD_MS  100          // 屏幕刷新周期（10Hz）
#define ADC_TIMEOUT_MS 500          // 超过这个时间没收到数据就提示超时

// ---------------- 右摇杆 / 菜单操作参数 ----------------
#define STICK_RX_CH      2          // 右摇杆左右（X）接 MCP3208 的通道号
#define STICK_RY_CH      3          // 右摇杆上下（Y）接 MCP3208 的通道号
#define STICK_RX_INVERT  false      // 向右推时读数变小，就改成 true
#define STICK_RY_INVERT  false      // 向上推时读数变小，就改成 true
#define STICK_CENTER     2048       // 摇杆中位原始值（以后可由“摇杆校准”写入）
#define STICK_TH_ON      1200       // 偏离中位超过这个值 = 推动
#define STICK_TH_OFF     600        // 回到这个范围以内 = 松开（迟滞，防抖）
#define REPEAT_DELAY_MS  400        // 按住上下：多久后开始连续移动
#define REPEAT_RATE_MS   150        // 连续移动的间隔
#define MENU_HOLD_MS     800        // ADC 页面：向右推保持多久进入菜单
#define UI_PERIOD_MS     20         // 界面任务循环周期（摇杆响应速度）

// ==========================================================
//                 LovyanGFX 屏幕驱动配置
// ==========================================================
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel;
  lgfx::Bus_SPI      _bus;
  lgfx::Light_PWM    _light;

public:
  LGFX() {
    { // ---- SPI 总线 ----
      auto cfg = _bus.config();
      cfg.spi_host    = LCD_SPI_HOST;
      cfg.spi_mode    = LCD_SPI_MODE;
      cfg.freq_write  = LCD_FREQ_WRITE;
      cfg.freq_read   = 16000000;
      cfg.spi_3wire   = true;        // 只有 MOSI，没有 MISO
      cfg.use_lock    = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk    = LCD_SCLK;
      cfg.pin_mosi    = LCD_MOSI;
      cfg.pin_miso    = -1;
      cfg.pin_dc      = LCD_DC;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    { // ---- 面板 ----
      auto cfg = _panel.config();
      cfg.pin_cs          = LCD_CS;
      cfg.pin_rst         = LCD_RES;
      cfg.pin_busy        = -1;
      cfg.panel_width     = LCD_WIDTH;
      cfg.panel_height    = LCD_HEIGHT;
      cfg.memory_width    = 240;     // ST7789 显存固定为 240x320
      cfg.memory_height   = 320;
      cfg.offset_x        = LCD_OFFSET_X;
      cfg.offset_y        = LCD_OFFSET_Y;
      cfg.offset_rotation = 0;
      cfg.readable        = false;   // 没接 MISO，不能读屏
      cfg.invert          = LCD_INVERT;
      cfg.rgb_order       = LCD_RGB_ORDER;
      cfg.dlen_16bit      = false;
      cfg.bus_shared      = false;   // 屏幕独占 FSPI
      _panel.config(cfg);
    }
    { // ---- 背光（PWM 调光）----
      auto cfg = _light.config();
      cfg.pin_bl      = LCD_BL;
      cfg.invert      = false;       // 低电平点亮的背光改成 true
      cfg.freq        = 44100;
      cfg.pwm_channel = 7;
      _light.config(cfg);
      _panel.setLight(&_light);
    }
    setPanel(&_panel);
  }
};

// ---------------- 颜色（RGB565） ----------------
#define C_BG      TFT_BLACK
#define C_TITLE   0x2124            // 深灰标题栏
#define C_GRID    0x4208            // 灰色分隔线 / 条形图边框
#define C_DIM     0x8410            // 暗灰文字
static const uint16_t CH_COLOR[ADC_CH_COUNT] = {
  TFT_YELLOW, TFT_CYAN, TFT_GREENYELLOW, TFT_MAGENTA, TFT_ORANGE
};

// ---------------- 对象 ----------------
static LGFX         tft;
static LGFX_Sprite  sprVolt(&tft);   // 电压数值离屏缓冲（防闪烁）
static LGFX_Sprite  sprRaw(&tft);    // 原始值离屏缓冲
static SPIClass     adcSPI(HSPI);    // ADC 独占 HSPI，与屏幕互不干扰

struct AdcFrame {
  uint16_t raw[ADC_CH_COUNT];   // 0~4095
  uint16_t mv[ADC_CH_COUNT];    // 毫伏
  uint32_t seq;                 // 帧计数
};
static QueueHandle_t adcQueue;

// ==========================================================
//                     MCP3208 驱动（核心 1 使用）
// ==========================================================
static uint16_t mcp3208Read(uint8_t ch) {
  // 单端模式：起始位 1、SGL=1、D2 D1 D0 = 通道号
  uint8_t tx0 = 0x06 | (ch >> 2);
  uint8_t tx1 = (ch & 0x03) << 6;

  adcSPI.beginTransaction(SPISettings(ADC_CLK, MSBFIRST, SPI_MODE0));
  digitalWrite(ADC_CS, LOW);
  adcSPI.transfer(tx0);
  uint8_t hi = adcSPI.transfer(tx1);
  uint8_t lo = adcSPI.transfer(0x00);
  digitalWrite(ADC_CS, HIGH);
  adcSPI.endTransaction();

  return ((uint16_t)(hi & 0x0F) << 8) | lo;
}

// ==========================================================
//                 核心 1：ADC 采集任务
// ==========================================================
static void adcTask(void *arg) {
  pinMode(ADC_CS, OUTPUT);
  digitalWrite(ADC_CS, HIGH);
  adcSPI.begin(ADC_SCLK, ADC_MISO, ADC_MOSI, -1);   // CS 由软件控制

  AdcFrame f = {};
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    for (uint8_t ch = 0; ch < ADC_CH_COUNT; ch++) {
      uint32_t sum = 0;
      for (uint8_t n = 0; n < ADC_OVERSAMPLE; n++) sum += mcp3208Read(ch);
      uint16_t raw = (sum + ADC_OVERSAMPLE / 2) / ADC_OVERSAMPLE;
      f.raw[ch] = raw;
      f.mv[ch]  = (uint32_t)raw * ADC_VREF / 4095;
    }
    f.seq++;
    xQueueOverwrite(adcQueue, &f);                   // 覆盖旧数据，永不阻塞
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(ADC_PERIOD_MS));
  }
}

// ==========================================================
//                 核心 0：LCD 显示任务
// ==========================================================
// 布局（竖屏 240x320）：
//   标题栏 36px，下面 5 行，每行两层：
//   第一层：“通道X” 标签 + 大号电压
//   第二层：条形图 + 原始值
static int16_t TITLE_H;                           // 标题栏高度：屏高 >= 240 时 36，否则 28
static int16_t TOP;                               // ADC 行区域起点
static int16_t scrW, scrH, rowH;

static const int16_t RAW_W  = 52, RAW_H = 22;     // 原始值 sprite 尺寸
static const int16_t BAR_H  = 8;                  // 条形图高度

// 每一行里各元素的位置（y 是相对本行顶部的偏移），按横屏 / 竖屏在运行时设定
struct RowLayout {
  int16_t labelY;
  int16_t voltX, voltY, voltW, voltH, voltTextY;
  const uint8_t *voltFont;
  int16_t rawX,  rawY;
  int16_t barX,  barY,  barW;
};
static RowLayout L;

static int16_t MENU_TOP;

static void setupLayout() {
  bool shortScr = scrH < 240;                     // 170 高的横屏
  TITLE_H  = shortScr ? 28 : 36;
  TOP      = TITLE_H + (shortScr ? 2 : 4);
  MENU_TOP = TITLE_H + 2;
  rowH     = (scrH - TOP) / ADC_CH_COUNT;         // 320x170 横屏 28，320x240 横屏 40，240x320 竖屏 56

  L.voltFont = font_num30;  L.voltH = 26;  L.voltTextY = 2;
  if (scrW > scrH && rowH < 40) {
    // 紧凑横屏 320x170：每行高 28，一行排开  [通道X] [ 电压 ] [条形图] [原始值]
    L.voltFont = font_num24;  L.voltH = rowH - 2;  L.voltTextY = 2;
    L.labelY = (rowH - 24) / 2;
    L.voltX  = 72;  L.voltY = 1;  L.voltW = 96;
    L.rawX   = scrW - RAW_W - 8;  L.rawY = (rowH - RAW_H) / 2;
    L.barX   = L.voltX + L.voltW + 8;  L.barY = (rowH - BAR_H) / 2;
    L.barW   = L.rawX - L.barX - 8;
  } else if (scrW > scrH) {
    // 横屏 320x240：每行高 40，一行排开  [通道X] [ 电压 ] [条形图] [原始值]
    L.labelY = 8;
    L.voltX  = 72;  L.voltY = 7;  L.voltW = 112;
    L.rawX   = scrW - RAW_W - 8;  L.rawY = 9;
    L.barX   = L.voltX + L.voltW + 8;  L.barY = 16;
    L.barW   = L.rawX - L.barX - 8;
  } else {
    // 竖屏 240x320：每行高 56，两层  [通道X  电压] / [条形图  原始值]
    L.labelY = 5;
    L.voltX  = 76;  L.voltY = 2;  L.voltW = scrW - 76 - 8;
    L.rawX   = scrW - RAW_W - 8;  L.rawY = 31;
    L.barX   = 8;   L.barY  = 38;
    L.barW   = scrW - 8 - RAW_W - 16;
  }
}

static int16_t rowY(uint8_t i) { return TOP + i * rowH; }

static void drawStatus(bool ok) {
  // 标题栏右侧的状态文字，只在状态改变时调用
  const int16_t w = 48;
  tft.fillRect(scrW - w - 4, 4, w, TITLE_H - 8, C_TITLE);
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(ok ? TFT_GREEN : TFT_RED, C_TITLE);
  tft.drawString(ok ? "正常" : "超时", scrW - 6, TITLE_H / 2);
}

static void drawStaticUI() {
  tft.fillScreen(C_BG);
  tft.fillRect(0, 0, scrW, TITLE_H, C_TITLE);

  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_WHITE, C_TITLE);
  tft.drawString("MCP3208 采集", 8, TITLE_H / 2);

  tft.setTextDatum(TL_DATUM);
  char label[16];
  for (uint8_t i = 0; i < ADC_CH_COUNT; i++) {
    int16_t y = rowY(i);
    snprintf(label, sizeof(label), "通道%u", (unsigned)i);
    tft.setTextColor(CH_COLOR[i], C_BG);
    tft.drawString(label, 8, y + L.labelY);
    tft.drawRect(L.barX, y + L.barY, L.barW, BAR_H, C_GRID);           // 条形图外框
    if (i < ADC_CH_COUNT - 1)
      tft.drawFastHLine(4, y + rowH - 1, scrW - 8, C_GRID);
  }
}

static void drawRow(uint8_t i, uint16_t raw, uint16_t mv) {
  int16_t y = rowY(i);
  char buf[16];

  // ---- 电压：先画进 sprite 再整块推到屏幕，数字跳动不闪烁 ----
  snprintf(buf, sizeof(buf), "%u.%03u V", mv / 1000, mv % 1000);
  sprVolt.fillSprite(C_BG);
  sprVolt.setTextColor(CH_COLOR[i], C_BG);   // 指定背景色，抗锯齿边缘才正确
  sprVolt.drawString(buf, L.voltW - 2, L.voltTextY);     // 右对齐（datum 在初始化时设为 TR）
  sprVolt.pushSprite(L.voltX, y + L.voltY);

  // ---- 原始值 ----
  snprintf(buf, sizeof(buf), "%u", raw);
  sprRaw.fillSprite(C_BG);
  sprRaw.setTextColor(C_DIM, C_BG);
  sprRaw.drawString(buf, RAW_W - 1, 0);
  sprRaw.pushSprite(L.rawX, y + L.rawY);

  // ---- 条形图：先画填充部分，再画剩余部分，不先清空，所以不闪 ----
  int16_t inner = L.barW - 2;
  int16_t fill  = (int32_t)inner * raw / 4095;
  if (fill > 0)     tft.fillRect(L.barX + 1, y + L.barY + 1, fill, BAR_H - 2, CH_COLOR[i]);
  if (fill < inner) tft.fillRect(L.barX + 1 + fill, y + L.barY + 1, inner - fill, BAR_H - 2, C_BG);
}

static bool createSprite(LGFX_Sprite &spr, int16_t w, int16_t h, const uint8_t *font) {
  spr.setColorDepth(16);
  if (spr.createSprite(w, h) == nullptr) return false;   // 16 位色：w*h*2 字节
  if (!spr.loadFont(font)) return false;                 // 每个 sprite 各自加载一次字体
  spr.setTextDatum(TR_DATUM);
  spr.setTextWrap(false, false);
  return true;
}

// ==========================================================
//                 右摇杆输入：把模拟量变成按键事件
// ==========================================================
enum StickEvt : uint8_t { EVT_NONE, EVT_UP, EVT_DOWN, EVT_LEFT, EVT_RIGHT, EVT_RIGHT_HOLD };

struct Axis {
  int8_t   dir      = 0;       // -1 / 0 / +1
  uint32_t pressT   = 0;       // 本次推动开始的时间
  uint32_t repeatT  = 0;       // 上一次连发的时间
  bool     holdDone = false;   // 本次推动是否已经触发过“长推”
  bool     locked   = false;   // 页面切换后，必须先回中才能再次触发
};
static Axis axX, axY;

// 带迟滞的方向判断：推过 TH_ON 才算推动，回到 TH_OFF 以内才算松开
static int8_t axisDir(const Axis &a, uint16_t raw, bool invert) {
  int32_t off = (int32_t)raw - STICK_CENTER;
  if (invert) off = -off;
  if (a.dir == 0) {
    if (off >  STICK_TH_ON) return  1;
    if (off < -STICK_TH_ON) return -1;
    return 0;
  }
  return (abs(off) < STICK_TH_OFF) ? 0 : a.dir;
}

// 上下：推动时触发一次，按住后自动连发
static StickEvt pollY(uint16_t raw, uint32_t now) {
  int8_t d = axisDir(axY, raw, STICK_RY_INVERT);
  StickEvt e = EVT_NONE;
  if (d != axY.dir) {
    axY.dir = d;
    if (d == 0) axY.locked = false;
    else if (!axY.locked) { axY.pressT = axY.repeatT = now; e = (d > 0) ? EVT_UP : EVT_DOWN; }
  } else if (d != 0 && !axY.locked &&
             now - axY.pressT >= REPEAT_DELAY_MS && now - axY.repeatT >= REPEAT_RATE_MS) {
    axY.repeatT = now;
    e = (d > 0) ? EVT_UP : EVT_DOWN;
  }
  return e;
}

// 左右：推动时触发一次；向右保持 MENU_HOLD_MS 再触发一次“长推”
static StickEvt pollX(uint16_t raw, uint32_t now) {
  int8_t d = axisDir(axX, raw, STICK_RX_INVERT);
  StickEvt e = EVT_NONE;
  if (d != axX.dir) {
    axX.dir = d;
    axX.holdDone = false;
    if (d == 0) axX.locked = false;
    else if (!axX.locked) { axX.pressT = now; e = (d > 0) ? EVT_RIGHT : EVT_LEFT; }
  } else if (d > 0 && !axX.locked && !axX.holdDone && now - axX.pressT >= MENU_HOLD_MS) {
    axX.holdDone = true;
    e = EVT_RIGHT_HOLD;
  }
  return e;
}

// 切换页面时调用：摇杆没回中之前不再产生事件，避免一推连跳两页
static void lockSticks() {
  if (axX.dir) axX.locked = true;
  if (axY.dir) axY.locked = true;
}

// ==========================================================
//                        菜单
// ==========================================================
static const char *const MENU_ITEMS[] = {
  "1. 控制模式", "2. 飞行模式", "3. 飞行速度", "4. 翻滚使能", "5. 扩展模块",
  "6. 摇杆校准", "7. 匹配四轴", "8. 语言选择", "9. 系统重置", "10. 退出菜单",
};
static const uint8_t MENU_COUNT = sizeof(MENU_ITEMS) / sizeof(MENU_ITEMS[0]);
static const uint8_t MENU_EXIT  = MENU_COUNT - 1;
static int16_t       menuCols, menuRows;        // 竖屏 1 列 x 10 行；横屏 2 列 x 5 行
static int16_t       menuItemW, menuItemH;

#define C_SEL     0x03BF      // 选中行背景（蓝）
#define C_OK      0x05E0      // 确认闪烁（绿）

static uint8_t menuSel = 0;

static void drawMenuItem(uint8_t i, uint16_t bg) {
  int16_t x = (i / menuRows) * menuItemW;       // 先排满第一列，再排第二列
  int16_t y = MENU_TOP + (i % menuRows) * menuItemH;
  tft.fillRect(x, y, menuItemW, menuItemH, bg);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_WHITE, bg);
  tft.drawString(MENU_ITEMS[i], x + 14, y + menuItemH / 2);
}

static void drawMenu() {
  tft.fillScreen(C_BG);
  tft.fillRect(0, 0, scrW, TITLE_H, C_TITLE);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_WHITE, C_TITLE);
  tft.drawString("设置菜单", 8, TITLE_H / 2);
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(C_DIM, C_TITLE);
  tft.drawString("▶确认", scrW - 6, TITLE_H / 2);
  for (uint8_t i = 0; i < MENU_COUNT; i++) drawMenuItem(i, i == menuSel ? C_SEL : C_BG);
}

static void menuMove(int8_t step) {
  uint8_t old = menuSel;
  menuSel = (menuSel + MENU_COUNT + step) % MENU_COUNT;   // 首尾循环
  tft.startWrite();
  drawMenuItem(old, C_BG);                                // 只重画两行，不闪
  drawMenuItem(menuSel, C_SEL);
  tft.endWrite();
}

// 各菜单项的功能：在这里填写（index 0 = “1. 控制模式” …… 8 = “9. 系统重置”）
static void onMenuSelect(uint8_t index) {
  Serial.printf("菜单选择: %s\n", MENU_ITEMS[index]);
  switch (index) {
    case 0: /* TODO: 控制模式 */ break;
    case 1: /* TODO: 飞行模式 */ break;
    case 2: /* TODO: 飞行速度 */ break;
    case 3: /* TODO: 翻滚使能 */ break;
    case 4: /* TODO: 扩展模块 */ break;
    case 5: /* TODO: 摇杆校准 */ break;
    case 6: /* TODO: 匹配四轴 */ break;
    case 7: /* TODO: 语言选择 */ break;
    case 8: /* TODO: 系统重置 */ break;
    default: break;
  }
}

// ==========================================================
//                 核心 0：界面任务（ADC 页 / 菜单页）
// ==========================================================
enum Page : uint8_t { PAGE_ADC, PAGE_MENU };

static void lcdTask(void *arg) {
  tft.init();
  tft.setRotation(LCD_ROTATION);
  tft.setBrightness(LCD_BRIGHTNESS);
  tft.setTextWrap(false, false);
  tft.loadFont(font_cn20);             // 屏幕本体一直使用 20px 中文字体

  scrW  = tft.width();
  scrH  = tft.height();
  setupLayout();
  menuCols  = (scrW > scrH) ? 2 : 1;
  menuRows  = (MENU_COUNT + menuCols - 1) / menuCols;
  menuItemW = scrW / menuCols;
  menuItemH = (scrH - MENU_TOP) / menuRows;      // 320x170 横屏 28，320x240 横屏 40，240x320 竖屏 28

  if (!createSprite(sprVolt, L.voltW, L.voltH, L.voltFont) ||
      !createSprite(sprRaw,  RAW_W, RAW_H,  font_cn20)) {
    tft.fillScreen(C_BG);
    tft.setTextColor(TFT_RED, C_BG);
    tft.drawString("内存不足", 10, 10);
    vTaskDelete(NULL);
  }

  Page       page = PAGE_ADC;
  uint16_t   lastRaw[ADC_CH_COUNT];
  int8_t     status = -1;                       // -1 未显示, 0 超时, 1 正常
  AdcFrame   f = {};
  bool       haveData = false;
  TickType_t lastRx   = xTaskGetTickCount();
  TickType_t lastDraw = 0;

  auto showAdcPage = [&]() {
    page = PAGE_ADC;
    drawStaticUI();
    for (auto &r : lastRaw) r = 0xFFFF;         // 强制所有行重画
    status = -1;
    lockSticks();
  };
  auto showMenuPage = [&]() {
    page = PAGE_MENU;
    menuSel = 0;
    drawMenu();
    lockSticks();
  };

  showAdcPage();

  for (;;) {
    // 等待新数据，最多等 UI_PERIOD_MS；摇杆以 ADC 采样速率（50Hz）处理
    if (xQueueReceive(adcQueue, &f, pdMS_TO_TICKS(UI_PERIOD_MS)) == pdTRUE) {
      lastRx = xTaskGetTickCount();
      haveData = true;
    }
    uint32_t now = millis();
    bool ok = (xTaskGetTickCount() - lastRx) < pdMS_TO_TICKS(ADC_TIMEOUT_MS);

    StickEvt ex = EVT_NONE, ey = EVT_NONE;
    if (haveData && ok) {
      ex = pollX(f.raw[STICK_RX_CH], now);
      ey = pollY(f.raw[STICK_RY_CH], now);
    }

    if (page == PAGE_ADC) {
      if (ex == EVT_RIGHT_HOLD) { showMenuPage(); continue; }

      // ADC 数值按 LCD_PERIOD_MS 限速刷新
      if (haveData && xTaskGetTickCount() - lastDraw >= pdMS_TO_TICKS(LCD_PERIOD_MS)) {
        lastDraw = xTaskGetTickCount();
        tft.startWrite();
        for (uint8_t i = 0; i < ADC_CH_COUNT; i++) {
          if (f.raw[i] != lastRaw[i]) {         // 只重画变化的行
            drawRow(i, f.raw[i], f.mv[i]);
            lastRaw[i] = f.raw[i];
          }
        }
        tft.endWrite();
      }
      if ((int8_t)ok != status) { drawStatus(ok); status = ok; }

    } else { // PAGE_MENU
      if (ey == EVT_UP)   menuMove(-1);
      if (ey == EVT_DOWN) menuMove(+1);

      if (ex == EVT_LEFT) { showAdcPage(); continue; }           // 向左推 = 退出
      if (ex == EVT_RIGHT) {                                      // 向右推 = 确认
        if (menuSel == MENU_EXIT) { showAdcPage(); continue; }
        drawMenuItem(menuSel, C_OK);                              // 绿色闪一下作为反馈
        vTaskDelay(pdMS_TO_TICKS(120));
        drawMenuItem(menuSel, C_SEL);
        onMenuSelect(menuSel);
      }
    }
  }
}

// ==========================================================
void setup() {
  Serial.begin(115200);

  adcQueue = xQueueCreate(1, sizeof(AdcFrame));

  xTaskCreatePinnedToCore(lcdTask, "lcd", 8192, NULL, 1, NULL, 0);  // 核心 0
  xTaskCreatePinnedToCore(adcTask, "adc", 4096, NULL, 2, NULL, 1);  // 核心 1
}

void loop() {
  vTaskDelete(NULL);   // 所有工作都在两个任务里，删除 Arduino 默认的 loop 任务
}
