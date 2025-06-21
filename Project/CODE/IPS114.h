#ifndef _IPS114_H_
#define _IPS114_H_

extern uint8 cls_flag;

void IPS114Init();
void ips114_show();
void ips114_show_start();
void ips114_show_pid();
void ips114_show_ccd();
void ips114_show_element();

void ips114_show_Ring();
void ips114_Ring_num();
void ips114_Ring_Can();
void ips114_Ring_50();
void ips114_Ring_60();
void ips114_Ring_90();
void ips114_show_Ramp();
void ips114_show_Cross();
#endif