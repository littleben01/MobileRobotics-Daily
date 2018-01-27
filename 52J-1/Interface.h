#ifndef __INTERFACE_H
#define __INTERFACE_H

#include <avr/io.h>

#define SLA             0x07


void MCU_init(void);
void Interface_init(void);
char getchar1(void);
void putchar1(char data);


void lcd_write_data(unsigned char data);
void lcd_display_str(unsigned char Y_line, unsigned char X_line,char *string);
void write_lcd_data(unsigned char Y_line, unsigned char X_line, long data);
void lcd_clear_screen(void);
void display_char(unsigned char line, unsigned char col, unsigned char data);

volatile extern unsigned char rx1_flg, rx1_buff;

//volatile extern long INT_Encoder[];

#endif		// __INTERFACE_H
