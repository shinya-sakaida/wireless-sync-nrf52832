/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
 
//無線通信同期システムプログラム 250msインターバル
//Wireless_Sync_Proprietery_Simple_nRF52832_20260210
 
//nRF52832用

#define rtc0_count_max_VALUE 40	//0.25s×40=10s 10秒毎に受信

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf_delay.h"
#include "boards.h"
#include <stdio.h>

volatile unsigned char wlp_ok=0;
volatile unsigned  char wlp_ok2=0;
volatile unsigned int rtc0_count=0;
volatile unsigned char time_up2=0;
char s1[200]={""};//UART出力の出力文字数より十分大きくする
volatile unsigned char rcv_ok=0;
volatile unsigned char send_ok=0;
volatile unsigned char send_only=0;//wp送信のみ時1、wpの送信受信時0

volatile unsigned char uart_disp_mode=0;
volatile unsigned char uart_disp_mode_all=1;//デフォルトはオール表示

//UART0受信用
volatile unsigned char read_data[50];
volatile unsigned char data_num_i=0;
volatile unsigned char rcv_buff=0;

volatile unsigned char sync=0;
volatile unsigned char received_sync=96;//同期用表示カウンターの受信値 初期値=96
volatile uint32_t start_rtc_counter=0;
volatile uint32_t received_start_rtc_counter=0;

volatile unsigned char wp_wait_time_up=0;//定期的な受信時に、受信待ちする時間を設定し、時間経過後に=1
volatile unsigned char rtc0_wlp_ok_ng=0;//rtc0のカウントアップ内にwlp_ok=1になるかチェックするためのフラグ
volatile unsigned char rtc0_wlp_ng_restart=0;//rtc0のカウントアップ内にwlp_ok=1にならなかった時(rtc0_wlp_ok_ng=1の時)rtc0_wlp_ng_restart=1
volatile unsigned char sync_not_equal_count=0;//sync<>received_syncの回数をカウント、一定回数連続で両者不一致であれば、sync=received_sync
volatile unsigned char sync_not_equal_count_thresh=1;//上記sync_not_equal_countの一定回数のこと

volatile unsigned char lighting_pattern=0;//発光パターンと周期の設定値
volatile unsigned char input_lighting_pattern=0;//自分の無線機への発光パターン入力値
volatile unsigned char received_lighting_pattern=0;//他の無線機からの発光パターン受信値
volatile unsigned char led_on_flag=0;//=1の時LEDをON

volatile unsigned int wp_tx_sum=0;//wpデータのrf_tx_data[5]～[11]のサムデータ(char)wp_check_sumをrf_tx_data[12]に代入
volatile unsigned int wp_rx_sum=0;//受信データのチェックサム確認用

volatile unsigned char start_rtc_not_random_send_count=0;
volatile unsigned char input_lighting_pattern_old=0;//発光パターンのSW入力変化を検出するための旧のSW入力値
volatile unsigned short receive_error_count=0;
volatile unsigned short pattern_error_count=0;

volatile unsigned int seed_random_number=0;
volatile signed char random_number_start_rtc=0;//start_rtc_counter設定のランダム値
volatile signed char random_number_start_rtc2=0;//random_number_start_rtcを元に修正したランダム値
volatile signed char prev_random_number_start_rtc2=0;//random_number_start_rtc2の前回値 -1が連続するのを避ける

volatile unsigned char radio_strength=0xff;
volatile unsigned int rtc0_count_max=40;//rtc0_count_max値を乱数によって設定
volatile unsigned int seed_rtc0_count_crystal=0;//rtc0_count_max値の初期値を決める乱数のシード値は16MHzクリスタルのウェイト値

volatile uint8_t rf_tx_data[64];
volatile uint8_t rf_rx_data[64];
volatile uint8_t rf_tx_data2[72];//rf_tx_dataにpayloadのLengthとPid(PacketID)を追加してrf_tx_data2を作る
volatile uint8_t rf_rx_data2[72];//rf_rx_dataにpayloadのLengthとPid(PacketID)を追加してrf_rx_data2を作る

void led_on_off_control(unsigned char);
void led_output(void);
void uart_process_init(void);
void uart_data_read_process(void);//void UART0_IRQHandler(void)にて呼ばれる
void print_version(void);
void read_data_clr(void);//UART受信用


//UART表示関数
void write2(char data3){
		NRF_UART0->TXD=data3;
		data3=0;
		NRF_UART0->TASKS_STARTTX;
		while(NRF_UART0->EVENTS_TXDRDY != 1){}
		NRF_UART0->EVENTS_TXDRDY = 0;	
}
	
void write_string(char *data2){
	while(*data2!=0x00){
		write2(*data2++);
	}
}

void RADIO_IRQHandler(void){
		NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_DisableIRQ(RADIO_IRQn);
		if(uart_disp_mode==1){
			write_string("R");
		}
		rcv_ok=1;
		if((strncmp((const char*)rf_rx_data2+2,"@wp1@",5)==0)){		
			for(unsigned char ij=0;ij<18;ij++)rf_rx_data[ij]=rf_rx_data2[ij+2];
			wp_rx_sum=0;
			for(unsigned char jj= 5; jj<=15; jj++){
				wp_rx_sum=wp_rx_sum+rf_rx_data[jj];
			}
			if(rf_rx_data[16]==(char)(wp_rx_sum&0x00ff)){
					radio_strength=NRF_RADIO->RSSISAMPLE;//何故か、本受信チェックバージョンではradio_strengthの値が正しく表示されない
					received_start_rtc_counter=((rf_rx_data[7]<<24)|(rf_rx_data[8]<<16)|(rf_rx_data[9]<<8)|(rf_rx_data[10]));
					if(received_start_rtc_counter>=start_rtc_counter){
						//rand()関数のシード生成
						if(seed_random_number==0){
							 seed_random_number=(unsigned int)((start_rtc_counter&0x000000ff)+(received_start_rtc_counter&0x000000ff));
							 srand(seed_random_number);
						}
						//発光パターンのSW入力変化時は、受信回数=6(10秒×5=50秒)になるまで、及び
						//発光パターンの入力がない場合でもリセット後、受信回数=5(50秒)になるまで
						//=received_start_rtc_counter+3;とする
						if(++start_rtc_not_random_send_count>=5)start_rtc_not_random_send_count=5;
						if(start_rtc_not_random_send_count<5){
							 random_number_start_rtc=10,random_number_start_rtc2=3;
							 start_rtc_counter=received_start_rtc_counter+random_number_start_rtc2;//received_start_rtc_counter+3
						}
						else {
							 random_number_start_rtc=(-(rand()%5)+3);//←(-(rand()%3)+1);//-1,0,1,2,3の5種類
							 if((random_number_start_rtc>=0)||(prev_random_number_start_rtc2==-1))random_number_start_rtc2=2;
								//-1が連続するのを避ける
								else random_number_start_rtc2=-1;
								//random_number_start_rtc2は-1と+2と+3の3種類
								start_rtc_counter=received_start_rtc_counter+random_number_start_rtc2;
								prev_random_number_start_rtc2=random_number_start_rtc2;
						}
						received_sync=(*(rf_rx_data+5)-0x30)*10+(*(rf_rx_data+6)-0x30);
						received_lighting_pattern=*(rf_rx_data+11);
						wlp_ok=1;
				}
			}
		}	
}
			
void rf_init(void){
		NRF_RADIO->BASE0=0xa94b02f6;
		NRF_RADIO->BASE1=0x0df61030;
		NRF_RADIO->PREFIX1=0;
		NRF_RADIO->PREFIX0=0;
		NRF_RADIO->PCNF0=0x00000006;
		NRF_RADIO->PCNF1=0x00040020;
		NRF_RADIO->CRCCNF = 2;//CRC length=2bytes
		NRF_RADIO->CRCINIT=0;
		NRF_RADIO->CRCPOLY=0x8d;//CRC poly
		NRF_RADIO->MODE=2;//Nrf250Kbit データレート250kbps 　MODEはUARTにて変更可能
		NRF_RADIO->TASKS_START=1;//PCNF0,PCNF1,BASE0,BASE1の設定値はNRF_RADIO->TASKS_START=1によって反映される
		NVIC_ClearPendingIRQ(RADIO_IRQn);
		NVIC_SetPriority(RADIO_IRQn, 1);
    NVIC_EnableIRQ(RADIO_IRQn);
}

void rf_start_tx(void){
		NRF_RADIO->SHORTS=0x113;
		NRF_RADIO->INTENSET=0x10;
		NRF_RADIO->TXADDRESS=0;//Logical address=0
		NRF_RADIO->RXADDRESSES=0x01;//Logocal address0はEenabled　他はDisabled
		rf_tx_data2[0]=0x20;//Payload.Length			
		rf_tx_data2[1]=0;
		for(unsigned char jj=0;jj<18;jj++)rf_tx_data2[jj+2]=rf_tx_data[jj];
		NRF_RADIO->PACKETPTR=(uint32_t)rf_tx_data2;		
		
		NVIC_ClearPendingIRQ(RADIO_IRQn);
		NVIC_DisableIRQ(RADIO_IRQn);	
		NRF_RADIO->EVENTS_ADDRESS=0;
		NRF_RADIO->EVENTS_PAYLOAD=0;
		NRF_RADIO->EVENTS_DISABLED=0;
	
		NRF_RADIO->TASKS_TXEN=1;
}

void rf_start_rx(void){
		NRF_RADIO->INTENCLR=0xFFFFFFFF;
		NRF_RADIO->EVENTS_DISABLED=0;

		NRF_RADIO->SHORTS=0x117;
		NRF_RADIO->INTENSET=0x10;
		NRF_RADIO->RXADDRESSES=0x01;//Logocal address0はEenabled　他はDisabled
	
		NRF_RADIO->PACKETPTR=(uint32_t)rf_rx_data2;
		
		NVIC_ClearPendingIRQ(RADIO_IRQn);
		NVIC_EnableIRQ(RADIO_IRQn);			
		NRF_RADIO->EVENTS_ADDRESS=0;
		NRF_RADIO->EVENTS_PAYLOAD=0;
		NRF_RADIO->EVENTS_DISABLED=0;
				
		NRF_RADIO->TASKS_RXEN=1;	
}

void rf_stop(void){
		NRF_RADIO->SHORTS=0;
		NRF_RADIO->INTENCLR=0xFFFFFFFF;
		NRF_RADIO->EVENTS_DISABLED=0;
		NRF_RADIO->TASKS_DISABLE=1;
		while(NRF_RADIO->EVENTS_DISABLED==0);
}

//RTC動作
void RTC0_IRQHandler(void){
	if(++start_rtc_counter>=0x01000000)start_rtc_counter=0;
	NRF_RTC0->EVENTS_COMPARE[0]=0;//RTC動作において必ず必要。
	//上記がないとRTC0インタラプトを抜けてもまた、すぐインタラプトがかかる
	NRF_RTC0->TASKS_CLEAR=1;
	NRF_CLOCK->EVENTS_HFCLKSTARTED=0;
	NRF_CLOCK->TASKS_HFCLKSTART=1;
	while(NRF_CLOCK->EVENTS_HFCLKSTARTED==0);
		
	if(++rtc0_count==rtc0_count_max){
		rtc0_count=0;
		rtc0_count_max=rtc0_count_max_VALUE+(rand()%3);//rtc0_count_maxの値を乱数によって設定
		NRF_RTC0->CC[0]=50;//5ms×50=250ms
	}
	else if(rtc0_count==1)NRF_RTC0->CC[0]=50-4;//5ms×50-4=230ms
	else	NRF_RTC0->CC[0]=50;//5ms×50=250ms
	
	NRF_TIMER0->TASKS_START=1;
	wp_wait_time_up=0;
	rtc0_wlp_ok_ng=1;
	//rtc0のカウントアップ内にwlp_ok=1になるかチェックするためのフラグ
	if(++sync>=96)sync=0;
	led_on_off_control(lighting_pattern);
	led_output();
	time_up2=1;
}

//WPウェイトタイマー設定
void TIMER0_IRQHandler(void){
	if(NRF_TIMER0->EVENTS_COMPARE[0]!=0){
			NRF_TIMER0->EVENTS_COMPARE[0]=0;
			NRF_TIMER0->TASKS_CLEAR=1;
			NRF_TIMER0->TASKS_STOP=1;
			wp_wait_time_up=1;
	}
}	

//プリント表示/動作モード変更用 P07入力=HによりUART0動作
void UART0_IRQHandler(void){
	if(NRF_UART0->EVENTS_RXDRDY!=0){
			NRF_UART0->EVENTS_RXDRDY=0;
			rcv_buff=NRF_UART0->RXD;//RXD 8bit
			if(data_num_i<50)read_data[data_num_i++]=rcv_buff;
			if((rcv_buff=='\r')||(rcv_buff=='\n'))data_num_i=0;
			else if(rcv_buff=='@')data_num_i=0,uart_data_read_process();
	}
}

void uart_data_read_process(void){
	if(uart_disp_mode==1){
		if(strncmp((char*)(read_data),"ver@",4)==0){
			read_data_clr();
			print_version();
		}
		else if(strncmp((char*)(read_data),"21@",3)==0){
			read_data_clr();
			write_string("\n\rUART	0x21");
		}
		else if(strncmp((char*)(read_data),"31@",3)==0){
			read_data_clr();
			uart_disp_mode_all=1;
			write_string("\n\rUART	0x31 uart_disp_mode_all=1");
		}
		else if(strncmp((char*)(read_data),"30@",3)==0){
			read_data_clr();
			uart_disp_mode_all=0;
			write_string("\n\rUART	0x30 uart_disp_mode_all=0");
		}
		else if(strncmp((char*)(read_data),"sync_not@",9)==0){
				read_data_clr();
				sprintf(s1,"\n\rsync_not_thresh=%2x\n\r",sync_not_equal_count_thresh);
				write_string(s1);
		}
		else if((read_data[0]=='s')&&(read_data[1]=='y')&&(read_data[2]=='n')&&(read_data[3]=='c')
			&&(read_data[4]=='_')&&(read_data[5]=='n')&&(read_data[6]=='o')&&(read_data[7]=='t')&&(read_data[9]=='@')){
				if((read_data[8]>=0x30)&&(read_data[8]<=0x36))sync_not_equal_count_thresh=(read_data[8]-0x30);
				sprintf(s1,"\n\rsync_not_thresh=%2x\n\r",sync_not_equal_count_thresh);
				write_string(s1);
		}
		else if(strncmp((char*)(read_data),"mode@",5)==0){
				read_data_clr();
				sprintf(s1,"\n\rmode=%2x\n\r",NRF_RADIO->MODE);
				write_string(s1);
		}
		else if((read_data[0]=='m')&&(read_data[1]=='o')&&(read_data[2]=='d')&&(read_data[3]=='e')&&(read_data[5]=='@')){
					if((read_data[4]>=0x30)&&(read_data[4]<=0x36))NRF_RADIO->MODE=(read_data[4]-0x30);
					else if(read_data[4]=='f')NRF_RADIO->MODE=15;
					sprintf(s1,"\n\rmode=%2x\n\r",NRF_RADIO->MODE);
					write_string(s1);
		}
	}
}

void print_version(void){
	sprintf(s1,"\n\rWireless_Sync_Propritery_Simple_20260210   ");
	write_string(s1);
}

void read_data_clr(void){
	for(unsigned short i=0;i<50;i++)*(read_data+i)=0x00,data_num_i=0;
}

void ioport(void){
	for(unsigned char jx=17;jx<=20;jx++){
		NRF_GPIO->PIN_CNF[jx]=0x03;
	}
	NRF_GPIO->OUT=0x001E0000;//nRF52DK開発KITの4個のLED消灯(P17-P20=H)
}

// ボタンが押された瞬間の処理
bool button_was_pressed = false;
void button_check(void)
{
		bool button_is_pressed = (nrf_gpio_pin_read(BUTTON_1) == 0);
    if (button_is_pressed && !button_was_pressed)
    {
			input_lighting_pattern=lighting_pattern+1;
			if(input_lighting_pattern==15)input_lighting_pattern=11;
		}
		button_was_pressed = button_is_pressed;
}

void buttons_init(void)
{
    nrf_gpio_cfg_input(BUTTON_1, NRF_GPIO_PIN_PULLUP);
}

void uart_process_init(void){
		uart_disp_mode=0;
		//P07入力により、UART表示ON/OFFの設定
		if(((NRF_GPIO->IN)&0x000080)==0x080){
			nrf_delay_us(10);
			if(((NRF_GPIO->IN)&0x000080)==0x080){
				nrf_delay_us(10);
				if(((NRF_GPIO->IN)&0x000080)==0x080){
					uart_disp_mode=1;
				}
			}
		}
	
		//UARTオフ時はポート設定しない
		if(uart_disp_mode==1){				
			NRF_GPIO->PIN_CNF[8]=0x0E;//LowPower RXDピンをプルアップ　プルアップしても動作UART動作OK
			NRF_GPIO->PIN_CNF[6]=0x01;//TXD 出力ピンに設定
				
		 //UART0
		 NRF_UART0->ENABLE=0;
		 NRF_UART0->PSELTXD=0x00000006;//nRF52DK
		 NRF_UART0->PSELRXD=0x00000008;//nRF52DK
		 
		 NRF_UART0->BAUDRATE=0x01D7E000;//115.2kbps
		 NRF_UART0->CONFIG=0;
		 NRF_UART0->ENABLE=4;
		
		 NRF_UART0->INTENSET= 4;//enable interrupt on RXRDY event
		 NRF_UART0->TASKS_STARTTX=1;
		 
		 NRF_UART0->TASKS_STARTRX=1;
		 NVIC_ClearPendingIRQ(UART0_IRQn);
		 NVIC_SetPriority(UART0_IRQn, 3);
		 NVIC_EnableIRQ(UART0_IRQn);
		 read_data_clr();
	}
}


int main(void)
{
		ioport();
	
		NRF_GPIO->PIN_CNF[7]=0x04;//pull down　P07=Hの場合はUART表示する
		NRF_GPIO->PIN_CNF[8]=0x0E;//LowPower RXDピンをプルアップ　プルアップしても動作UART動作OK
		NRF_GPIO->PIN_CNF[6]=0x01;//TXD 出力ピンに設定
		
		seed_rtc0_count_crystal=0;//rtc0_count_max値の乱数のシード設定用
		// Set 16 MHz crystal as our 16 MHz clock source (as opposed to internal RCOSC)
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Wait
			 if(++seed_rtc0_count_crystal>=100)seed_rtc0_count_crystal=0;
				//rtc0_count_max値の乱数のシード設定用
    }
    NRF_CLOCK->LFCLKSRC=1;//32.768kHz crystal oscillator
		
		//TIMER0設定
		NRF_TIMER0->MODE=0;//Timer mode
		NRF_TIMER0->BITMODE=3;//32bit
		NRF_TIMER0->PRESCALER=9;//2~9/16000000=32us
		NRF_TIMER0->CC[0]=8750;//250ms+30ms=280ms//WPウェイトタイマー設定
		
		NVIC_ClearPendingIRQ(TIMER0_IRQn);
    NVIC_SetPriority(TIMER0_IRQn, 2);
    NVIC_EnableIRQ(TIMER0_IRQn);//wpウェイトタイマーイネーブル
		NRF_TIMER0->INTENSET=0x00010000;//Compare0のインタラプトイネーブル
		NRF_TIMER0->TASKS_STOP=1;
		NRF_TIMER0->TASKS_CLEAR=1;
		
		//RTC動作
		NRF_CLOCK->TASKS_LFCLKSTART=1;//RTC動作のためには絶対必要
		NRF_RTC0->PRESCALER=164;//198.6Hz 5.035ms
		NRF_RTC0->CC[0]=50;//5ms×50=250ms
		NRF_RTC0->INTENSET=0x00010000;//COMPARE[0]
		NRF_RTC0->EVTENSET=0x00010000;//COMPARE[0]
		NVIC_ClearPendingIRQ(RTC0_IRQn);
		NVIC_SetPriority(RTC0_IRQn,2);
		NVIC_EnableIRQ(RTC0_IRQn);
		NRF_RTC0->TASKS_CLEAR=1;
		NRF_RTC0->TASKS_START=1;
		NRF_POWER->TASKS_LOWPWR=1;
		
		//WDT
		NRF_WDT->CRV=32768*10;//WDT=10秒<30秒 	
		NRF_WDT->RR[0]=0x6E524635;
		NRF_WDT->TASKS_START=1;//WDTスタート
		
		uart_process_init();
		
		NRF_POWER->DCDCEN = 1;//DCDCconverterEnabl
    rf_init();
		NRF_RADIO->FREQUENCY=50;//2450MHz//設定周波数
		NRF_RADIO->TXPOWER=0xF8;//-8dbm//設定出力レベル
		
		wlp_ok=0;
		wlp_ok2=0;
		time_up2=0;
		*rf_rx_data=0x00;
		send_only=0;
		rf_start_rx();
		
		input_lighting_pattern=0;
		input_lighting_pattern_old=0;
		received_lighting_pattern=0;
		lighting_pattern=14;
		
		start_rtc_not_random_send_count=0;
		receive_error_count=0;
		pattern_error_count=0;
		start_rtc_counter=0;
		received_start_rtc_counter=0;
		input_lighting_pattern_old=input_lighting_pattern;
		
		seed_random_number=0;
		srand(seed_rtc0_count_crystal);//rtc0_count_maxの初期値を決める乱数のシード値を設定
		rtc0_count_max=rtc0_count_max_VALUE +rand()%3;//初期値
		radio_strength=0xff;//初期値
		if(uart_disp_mode==1){
			sprintf(s1,",seed_rtc0_count_crystal=%3d rtc0_count_max=%3d",
				seed_rtc0_count_crystal,rtc0_count_max);
			write_string(s1),write_string("\r\n");
		}

    //Initialize buttons
    buttons_init();
		
		while (true){
			
			if((wlp_ok==1)||(wp_wait_time_up==1)){
				//受信エラーが3回続く(最大30秒×3=1.5分+10秒(WDT周期))とWDTリセットする
				if(rtc0_wlp_ok_ng==1){
					rtc0_wlp_ng_restart=1;
					if(++receive_error_count>=6)receive_error_count=6;//30秒×6=3分
				}
				else if(rtc0_count==2)receive_error_count=0;
				//定期的な受信NG時に、受信待ち時間経過後に、
				//強制的にwlp_ok=1にする時、wp_wait_time_up_restart=1		
				wp_wait_time_up=0;
				wlp_ok=0;
				wlp_ok2=1;
				if(uart_disp_mode==1){
					if((uart_disp_mode_all==1)||((uart_disp_mode_all==0)&&(rtc0_count==2))){
						write_string("\r\n");
						write_string((char*)rf_rx_data);//受信データ
						sprintf(s1,"%4d%3d%3d %6x%3d%3d%3d%3d%3d %6x %2x %d %d ",rtc0_count,sync,received_sync
							,start_rtc_counter,random_number_start_rtc,random_number_start_rtc2,start_rtc_not_random_send_count
							,lighting_pattern,received_lighting_pattern,received_start_rtc_counter,radio_strength
							,receive_error_count,pattern_error_count);
						write_string(s1);
					}
				}
				
				if((send_only==0)&&(rtc0_wlp_ng_restart!=1))NRF_RTC0->TASKS_CLEAR=1;
				rf_stop();
				
				NRF_RADIO->EVENTS_READY=0;//ないとNG									
				rf_tx_data[0]='@';
				rf_tx_data[1]='w';
				rf_tx_data[2]='p';
				rf_tx_data[3]='1';
				rf_tx_data[4]='@';
				rf_tx_data[5]=0x30+(sync/10);
				rf_tx_data[6]=0x30+(sync%10);
				rf_tx_data[7]=(unsigned char)((start_rtc_counter>>24)&0x000000ff);
				rf_tx_data[8]=(unsigned char)((start_rtc_counter>>16)&0x000000ff);
				rf_tx_data[9]=(unsigned char)((start_rtc_counter>>8)&0x000000ff);
				rf_tx_data[10]=(unsigned char)(start_rtc_counter&0x000000ff);
				rf_tx_data[11]=lighting_pattern;
				rf_tx_data[12]=0;//共有データ領域
				rf_tx_data[13]=0;//共有データ領域
				rf_tx_data[14]=0;//共有データ領域	
				rf_tx_data[15]=0;//共有データ領域
				wp_tx_sum=0;
				for(unsigned char jj= 5; jj<=15; jj++)wp_tx_sum=wp_tx_sum+rf_tx_data[jj];
				rf_tx_data[16]=(char)(wp_tx_sum&0x00ff);
				rf_tx_data[17]=0x00;
						
				rf_start_tx();
				while(NRF_RADIO->EVENTS_READY!=1);
				
				send_ok=1;
				if(uart_disp_mode==1)write_string("S");
				nrf_delay_ms(2);
				
				rf_stop();
				radio_strength=0xff;
				*rf_tx_data=0x00;
				*rf_rx_data=0x00;
			}
				
			if ((wlp_ok2==1)&&(send_ok==1)){
					if(received_sync<96){	      //LEDのduty=1/8dutyのため 96
						//パラメータはsyncとreceived_syncの差の値と両者が差以上の連続回数
						if(((sync>received_sync)&&(sync-received_sync>=1))
							||((sync+96>received_sync)&&(sync+96-received_sync>=1))
								||((received_sync>sync)&&(received_sync-sync>=1))
									||((received_sync+96>sync)&&(received_sync+96-sync>=1))){
										if(++sync_not_equal_count>=sync_not_equal_count_thresh){
											sync_not_equal_count=sync_not_equal_count_thresh,sync=received_sync;
										}
							}
						else if(received_sync==sync)sync_not_equal_count=0,sync=received_sync;
						received_sync=96;
					}
					
					button_check(); // ボタンの状態
					
					if(input_lighting_pattern==0){ 
						if(received_lighting_pattern!=0)lighting_pattern=received_lighting_pattern;
					}
					else lighting_pattern=input_lighting_pattern;
					
					if(input_lighting_pattern!=0){ 
						if(received_lighting_pattern!=input_lighting_pattern){
							if(++pattern_error_count>=(12*rtc0_count_max))pattern_error_count=(12*rtc0_count_max);
						}
						else{
							input_lighting_pattern=0;
							pattern_error_count=0;
						}
					}
					
					//発光パターンのSW入力変化検出
					if((input_lighting_pattern!=0)&&(input_lighting_pattern!=input_lighting_pattern_old)){
						start_rtc_not_random_send_count=0;
						input_lighting_pattern_old=input_lighting_pattern;
					}
					
					rtc0_wlp_ng_restart=0;
					//rtc0のカウントアップ内にwlp_ok=1にならなかった時(rtc0_wlp_ok_ng=1の時)1
					wlp_ok2=0;
					rcv_ok=0;
					send_ok=0;
					
					//TIMER0設定
					NRF_TIMER0->TASKS_CLEAR=1;
					NRF_TIMER0->TASKS_STOP=1;//sleep時低消費電流化
					rf_stop();			
					NRF_CLOCK->TASKS_HFCLKSTOP=1;
					do{
							__WFE();
							__SEV();
							__WFE();
					}while(time_up2!=1);
					
					rtc0_wlp_ok_ng=0;
					if(rtc0_count!=2){
						wlp_ok=1;
						send_only=1;
					}
					else {
						send_only=0;
						rf_start_rx();
					}
					
					if(uart_disp_mode==1){
						//P07入力により、UART表示ON/OFFの設定
						if(((NRF_GPIO->IN)&0x000080)!=0x080){
							nrf_delay_us(10);
							if(((NRF_GPIO->IN)&0x000080)!=0x080){	
							}
						}
						else{
							//受信エラーが6回続くとWDTリセットをかける
							if((receive_error_count<6)&&(pattern_error_count<(12*rtc0_count_max))){
								NRF_WDT->RR[0]=0x6E524635;//この設定がないとWDTリセットがかかる
							}
							NRF_WDT->CRV=32768*10;//WDT周期=10秒 	
						}
					}
					else{
						//P07入力により、UART表示ON/OFFの設定
						if(((NRF_GPIO->IN)&0x000080)==0x080){
							nrf_delay_us(10);
							if(((NRF_GPIO->IN)&0x000080)==0x080){
								nrf_delay_us(10);
								if(((NRF_GPIO->IN)&0x000080)==0x080){
								}
							}
						}
						
						//受信エラーが3回続くとWDTリセットをかける
						else{
							if((receive_error_count<6)&&(pattern_error_count<(12*rtc0_count_max))){
								NRF_WDT->RR[0]=0x6E524635;//この設定がないとWDTリセットがかかる
							}
							NRF_WDT->CRV=32768*10;//WDT周期=10秒
						}
					}
			}
			time_up2=0;
		}
}


void led_on_off_control(unsigned char pattern){
	switch (pattern){
		case 14:
			if(sync%2==0)led_on_flag=1;
			else led_on_flag=0;
			break;
		case 13:
			if((sync%4==0)||(sync%4==1))led_on_flag=1;
			else led_on_flag=0;
			break;	
		case 12:
			if((sync%8==0)||(sync%8==1)||(sync%8==2)||(sync%8==3))led_on_flag=1;
			else led_on_flag=0;
			break;
		case 11:
			if((sync%16==1)||(sync%16==3)||(sync%16==5))led_on_flag=1;
			else led_on_flag=0;
			break;
		default:
			if(sync%2==0)led_on_flag=1;
			else led_on_flag=0;
			break;
	}	
}

void led_output(void){
		if((led_on_flag)==1){	
					NRF_GPIO->OUT=NRF_GPIO->OUT&0xFFF7FFEF;//P31=H P30=H P19=L P04=L
					NRF_GPIO->OUT=NRF_GPIO->OUT|0xC0000000;//
		}
		else{
					NRF_GPIO->OUT=NRF_GPIO->OUT&0x3FFFFFFF;//P31=L P30=L P19=H P04=H
					NRF_GPIO->OUT=NRF_GPIO->OUT|0x00080010;//
		}
		if(receive_error_count!=0){
			NRF_GPIO->OUT=NRF_GPIO->OUT&0xFFEFFFFF;//P20=L
		}
		else{
			NRF_GPIO->OUT=NRF_GPIO->OUT|0x00100000;//P20=H
		}
		if(random_number_start_rtc2<0){
			NRF_GPIO->OUT=NRF_GPIO->OUT&0xFFFBFFFF;//P18=L
		}
		else{
			NRF_GPIO->OUT=NRF_GPIO->OUT|0x00040000;//P18=H
		}
}

