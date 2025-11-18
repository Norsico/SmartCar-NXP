#ifndef _KEY_H_
#define _KEY_H_

#define KEY1_PIN    P37 //返回
#define KEY2_PIN    P46 //上
#define KEY3_PIN    P45 //下
#define KEY4_PIN    P36 //减
#define KEY5_PIN    P51 //加
#define KEY6_PIN    P52 //确定

extern uint8 Model;
extern uint8 grade_flag;
extern uint8 ips114_show_flag;
extern uint8 lose;
extern uint8 plus;
extern uint8 Augle_flag; //二级菜单标志
extern uint8 key_flag;
extern uint8 ips114_show_element_flag;
extern uint8 ips114_show_ring_flag;
extern uint8 data_buff[85];

void key(void);
void ParameterExchange(void);
void EepromWrite(void);
void EepromRead(void);

#endif