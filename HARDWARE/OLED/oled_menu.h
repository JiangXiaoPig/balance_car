#ifndef __OLED_MENU_H
#define __OLED_MENU_H	 
#include "main.h"

	
typedef struct
{
    u8 current;//当前状态索引号
    u8 next; //向下一个
    u8 enter; //确定
    void (*current_operation)(void); //当前状态应该执行的操作
} Menu_table;
extern uint8_t func_index;
extern  Menu_table  table[26];

void Menu_key_set(void);
void show0(void);
void show1(void);
void show2(void);
void show3(void);
void show4(void);
void show5(void);
void show6(void);
void show7(void);
void show8(void);
void show9(void);
void show10(void);
void show11(void);
void show12(void);
void show13(void);
void show14(void);
void show15(void);
void show16(void);

#endif	 //__OLED_
