/* u8g2_d_setup.c */
/* generated code, codebuild, u8g2 project */

#include "u8g2.h"


/* ssd1306 */
/* ssd1306 1 */
extern uint8_t *u8g2_m_ssd1306_16_f(uint8_t *page_cnt);
void u8g2_Setup_ssd1306_i2c_128x32_univision_f(u8g2_t *u8g2, const u8g2_cb_t *rotation, u8x8_msg_cb byte_cb, u8x8_msg_cb gpio_and_delay_cb)
{
  uint8_t tile_buf_height;
  uint8_t *buf;
  u8g2_SetupDisplay(u8g2, u8x8_d_ssd1306_128x32_univision, u8x8_cad_ssd13xx_i2c, byte_cb, gpio_and_delay_cb);
  buf = u8g2_m_ssd1306_16_f(&tile_buf_height);
  u8g2_SetupBuffer(u8g2, buf, tile_buf_height, u8g2_ll_hvline_vertical_top_lsb, rotation);
}


/* ----------------------------------------------------------------------
 * 本项目新增：0.96" SSD1306 128x64，4 线 SPI（U8g2 标准写法）
 * 复用出厂已生成的 u8g2_m_ssd1306_16_f（1024 字节 / 8 页）
 * cad 用 u8x8_cad_001（4 线 SPI 的通用命令/数据时序）
 * ---------------------------------------------------------------------- */
void u8g2_Setup_ssd1306_128x64_noname_f(u8g2_t *u8g2, const u8g2_cb_t *rotation,
                                       u8x8_msg_cb byte_cb, u8x8_msg_cb gpio_and_delay_cb)
{
  uint8_t tile_buf_height;
  uint8_t *buf;
  u8g2_SetupDisplay(u8g2, u8x8_d_ssd1306_128x64_noname, u8x8_cad_001,
                    byte_cb, gpio_and_delay_cb);
  buf = u8g2_m_ssd1306_16_f(&tile_buf_height);
  u8g2_SetupBuffer(u8g2, buf, tile_buf_height, u8g2_ll_hvline_vertical_top_lsb, rotation);
}

/* end of generated code */
