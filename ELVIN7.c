/*22 23 24
New BaudRate = 9600bod
<--O  2F 3F 21 0D 0A                                    /?!..
-->O  2F 45 4C 56 35 31 32 33 34 20 55 32 52 58 20 56   /ELV51234 U2RX V
-->O  31 2E 30 20 20 0D 0A                              1.0  ..
<--O  06 30 35 31 0D 0A                                 .051..
-->O  07 01 50 30 02 28 30 30 30 30 29 03 16            ..P0.(0000)..
<--O  01 52 32 02 30 30 30 30 28 29 03 1A               .R2.0000()..
-->O  02 28 39 35 36 35 30 30 30 30 30 30 30 30 30 30   .(95650000000000
-->O  30 30 29 03 54                                    00).T
<--O  01 42 30 03 75                                    .B0.u

<--O  2F 3F 30 30 30 31 21 0D 0A                        /?0001!..
-->O  2F 45 4C 56 35 31 32 33 34 20 55 32 52 58 20 56   /ELV51234 U2RX V
-->O  31 2E 30 20 20 0D 0A                              1.0  ..
<--O  06 30 35 31 0D 0A                                 .051..
-->O  #  01 50 30 02 28 30 30 30 30 29 03 16            ..P0.(0000)..
<--O  01 52 32 02 30 30 31 30 28 29 03 1B               .R2.0010()..
-->O  02 28 30 30 30 30 30 30 30 30 29 03 54            .(00000000).T
<--O  01 42 30 03 75                                    .B0.u
*/
#include "p24Fxxxx.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "DEE Emulation 16-bit.h"
#include "serial2.c"

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & BKBUG_OFF & ICS_PGx1 & FWDTEN_ON & WINDIS_OFF & FWPSA_PR128	 & WDTPS_PS16384);
_CONFIG2( IESO_OFF & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF & IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_HS)

#define GSM_ON		_LATB10
#define RTS			_LATB11

#define IN1 		_RB6
#define IN2 		_RB7
#define IN3			_RB8
#define IN4			_RA4
#define OUT1		_LATB9
#define OUT2		_LATB15

void
send_cmds(const char *cmd);

char pass[5] = "0000";
unsigned char ExIP[4] = {195,248,234,41};
//char APN[21] = "www.umc.ua";
//char APN[21] = "www.djuice.com.ua";
//char APN[21] = "3g.utel.ua";
char APN[21] = "www.kyivstar.net";
//char APN[21] = "www.jeans.ua";
char APN_user[11] = "";
char APN_pass[11] = "";
//char m_nbr[11] = "*100#";
char m_nbr[11] = "*111#";
char memo[31] = "5";
char nbr1[11];
char nbr2[11];
char nbr3[11];
unsigned char alrm1_day[10] = {0,0,0,0,0,0,0,0,0,0};
unsigned char alrm1_hour[10] = {0,0,0,0,0,0,0,0,0,0};
unsigned char alrm1_min[10] = {0,0,0,0,0,0,0,0,0,0};
unsigned int alrm1_do[10] = {0,0,0,0,0,0,0,0,0,0};
unsigned char alrm2_hour[10] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
unsigned char alrm2_min[10] = {0,0,0,0,0,0,0,0,0,0};
unsigned int alrm2_do[10] = {0,0,0,0,0,0,0,0,0,0};
unsigned char alrm_done[10] = {0,0,0,0,0,0,0,0,0,0};
unsigned int day_done = 0;
unsigned int Settings = 0b0000000000000000;
volatile unsigned int Period = 30;
unsigned int Templ = 45;
unsigned int SendSMS = 0;
//bit0 - немедленная отправка состояния
//bit1 - СМС отправляет модуль
//bit2 - отправка подтверждающих СМС
unsigned int nbrs[16] = {8845,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

char IMEI[16];
char in_call_nbr[11];
char in_sms_nbr[11];
char out_sms_nbr[11];
char ib[1000];
volatile char cb[1024];
char out_gprs[1024];
char tmp_buf[255];
char out_sms[255];
char *tmpp1, *out_sms_pos;
volatile char *cbp, *ibp;
volatile char in_byte;
volatile char ibs;
char in_sms = 1, in_rng = 0;
char *movptr1, *movptr2;
unsigned int Year = 7, Month = 1, Day = 1, Hour = 0, Minute = 0, Money = 0;
volatile unsigned int cnt100ms, cnt100ms2;
volatile unsigned int alrm_cnt = 0, mon_cnt = 0, out_m_cnt = 0;
volatile unsigned int alrm_f = 0, mon_f = 1, out_m_f = 1, time_sync_ok = 0;
unsigned int cnt1, cnt2;
unsigned int mcnt, rs_cnt, IMEI_ok = 0;
volatile unsigned int mrc;
unsigned int tmscag, of_cnt;
unsigned int send_gpscnt, cmd_cnt, cmd_cnt1, ret_f, out_gprsp;
unsigned char status_a1, status_a2, status_d, getdata = 0, reset_f = 0;
volatile unsigned char tmr3_f, p_cntm, p_cnts;
signed char Tm = 0, Tm300 = 0;
double P, Ea, Er;
unsigned int exp1, mn;
volatile unsigned int u2cnt;
unsigned char SMS_ret = 0;

const char *cmd[9] = {"0000()","0010()","0040(0000)","0040(0001)","0100()","0050(0000)","0200(000000080000)","0020()","0060(0000)"};

void
Delay100ms(unsigned int ms100)
{
	TMR2 = 0;
	cnt100ms = ms100;
	while(cnt100ms != 0);
}

void
read_param(void)
{
	ExIP[0] = (unsigned char) DataEERead(0x02);
	ExIP[1] = (unsigned char) (DataEERead(0x02) >> 8);
	ExIP[2] = (unsigned char) DataEERead(0x03);
	ExIP[3] = (unsigned char) (DataEERead(0x03) >> 8);
	Settings = DataEERead(0x06);
	for(cnt1 = 0; cnt1 < 4; cnt1 = cnt1 + 2)
	{
		pass[cnt1] = (unsigned char) DataEERead(cnt1/2 + 0x07);
		pass[cnt1 + 1] = (unsigned char) (DataEERead(cnt1/2 + 0x07) >> 8);
	}
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
	{
		nbr1[cnt1] = (unsigned char) DataEERead(cnt1/2 + 0x09);
		nbr1[cnt1 + 1] = (unsigned char) (DataEERead(cnt1/2 + 0x09) >> 8);
	}
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
	{
		nbr2[cnt1] = (unsigned char) DataEERead(cnt1/2 + 0x0E);
		nbr2[cnt1 + 1] = (unsigned char) (DataEERead(cnt1/2 + 0x0E) >> 8);
	}
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
	{
		nbr3[cnt1] = (unsigned char) DataEERead(cnt1/2 + 0x13);
		nbr3[cnt1 + 1] = (unsigned char) (DataEERead(cnt1/2 + 0x13) >> 8);
	}
	for(cnt1 = 0; cnt1 < 20; cnt1 = cnt1 + 2)
	{
		APN[cnt1] = (unsigned char) DataEERead(cnt1/2 + 0x18);
		APN[cnt1 + 1] = (unsigned char) (DataEERead(cnt1/2 + 0x18) >> 8);
	}
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
	{
		alrm1_day[cnt1] = (unsigned char) DataEERead(cnt1/2 + 0x22);
		alrm1_day[cnt1 + 1] = (unsigned char) (DataEERead(cnt1/2 + 0x22) >> 8);
	}
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
	{
		alrm1_hour[cnt1] = (unsigned char) DataEERead(cnt1/2 + 0x27);
		alrm1_hour[cnt1 + 1] = (unsigned char) (DataEERead(cnt1/2 + 0x27) >> 8);
	}
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
	{
		alrm1_min[cnt1] = (unsigned char) DataEERead(cnt1/2 + 0x2C);
		alrm1_min[cnt1 + 1] = (unsigned char) (DataEERead(cnt1/2 + 0x2C) >> 8);
	}
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
	{
		alrm2_hour[cnt1] = (unsigned char) DataEERead(cnt1/2 + 0x31);
		alrm2_hour[cnt1 + 1] = (unsigned char) (DataEERead(cnt1/2 + 0x31) >> 8);
	}
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
	{
		alrm2_min[cnt1] = (unsigned char) DataEERead(cnt1/2 + 0x36);
		alrm2_min[cnt1 + 1] = (unsigned char) (DataEERead(cnt1/2 + 0x36) >> 8);
	}
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
	{
		alrm1_do[cnt1] = (unsigned char) DataEERead(cnt1/2 + 0x3B);
		alrm1_do[cnt1 + 1] = (unsigned char) (DataEERead(cnt1/2 + 0x3B) >> 8);
	}
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
	{
		alrm2_do[cnt1] = (unsigned char) DataEERead(cnt1/2 + 0x40);
		alrm2_do[cnt1 + 1] = (unsigned char) (DataEERead(cnt1/2 + 0x40) >> 8);
	}
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
	{
		alrm_done[cnt1] = (unsigned char) DataEERead(cnt1/2 + 0x45);
		alrm_done[cnt1 + 1] = (unsigned char) (DataEERead(cnt1/2 + 0x45) >> 8);
	}
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
	{
		m_nbr[cnt1] = (unsigned char) DataEERead(cnt1/2 + 0x4A);
		m_nbr[cnt1 + 1] = (unsigned char) (DataEERead(cnt1/2 + 0x4A) >> 8);
	}
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
	{
		APN_user[cnt1] = (unsigned char) DataEERead(cnt1/2 + 0x4F);
		APN_user[cnt1 + 1] = (unsigned char) (DataEERead(cnt1/2 + 0x4F) >> 8);
	}
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
	{
		APN_pass[cnt1] = (unsigned char) DataEERead(cnt1/2 + 0x54);
		APN_pass[cnt1 + 1] = (unsigned char) (DataEERead(cnt1/2 + 0x54) >> 8);
	}
	day_done = DataEERead(0x59);
	Period = DataEERead(0x5A);
	Templ = DataEERead(0x5B);
	SendSMS = DataEERead(0x5C);
	for(cnt1 = 0; cnt1 < 30; cnt1 = cnt1 + 2)
	{
		memo[cnt1] = (unsigned char) DataEERead(cnt1/2 + 0x5D);
		memo[cnt1 + 1] = (unsigned char) (DataEERead(cnt1/2 + 0x5D) >> 8);
	}
}

void
write_param(void)
{
	DataEEWrite(ExIP[0] + (ExIP[1] << 8), 0x02);
	DataEEWrite(ExIP[2] + (ExIP[3] << 8), 0x03);
	DataEEWrite(Settings, 0x06);
	for(cnt1 = 0; cnt1 < 4; cnt1 = cnt1 + 2)
		DataEEWrite(pass[cnt1] + (pass[cnt1 + 1] << 8), cnt1/2 + 0x07);
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
		DataEEWrite(nbr1[cnt1] + (nbr1[cnt1 + 1] << 8), cnt1/2 + 0x09);
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
		DataEEWrite(nbr2[cnt1] + (nbr2[cnt1 + 1] << 8), cnt1/2 + 0x0E);
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
		DataEEWrite(nbr3[cnt1] + (nbr3[cnt1 + 1] << 8), cnt1/2 + 0x13);
	for(cnt1 = 0; cnt1 < 20; cnt1 = cnt1 + 2)
		DataEEWrite(APN[cnt1] + (APN[cnt1 + 1] << 8), cnt1/2 + 0x18);
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
		DataEEWrite(alrm1_day[cnt1] + (alrm1_day[cnt1 + 1] << 8), cnt1/2 + 0x22);
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
		DataEEWrite(alrm1_hour[cnt1] + (alrm1_hour[cnt1 + 1] << 8), cnt1/2 + 0x27);
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
		DataEEWrite(alrm1_min[cnt1] + (alrm1_min[cnt1 + 1] << 8), cnt1/2 + 0x2C);
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
		DataEEWrite(alrm2_hour[cnt1] + (alrm2_hour[cnt1 + 1] << 8), cnt1/2 + 0x31);
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
		DataEEWrite(alrm2_min[cnt1] + (alrm2_min[cnt1 + 1] << 8), cnt1/2 + 0x36);
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
		DataEEWrite(alrm1_do[cnt1] + (alrm1_do[cnt1 + 1] << 8), cnt1/2 + 0x3B);
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
		DataEEWrite(alrm2_do[cnt1] + (alrm2_do[cnt1 + 1] << 8), cnt1/2 + 0x40);
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
		DataEEWrite(alrm_done[cnt1] + (alrm_done[cnt1 + 1] << 8), cnt1/2 + 0x45);
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
		DataEEWrite(m_nbr[cnt1] + (m_nbr[cnt1 + 1] << 8), cnt1/2 + 0x4A);
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
		DataEEWrite(APN_user[cnt1] + (APN_user[cnt1 + 1] << 8), cnt1/2 + 0x4F);
	for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
		DataEEWrite(APN_pass[cnt1] + (APN_pass[cnt1 + 1] << 8), cnt1/2 + 0x54);
	DataEEWrite(day_done, 0x59);
	DataEEWrite(Period, 0x5A);
	DataEEWrite(Templ, 0x5B);
	DataEEWrite(SendSMS, 0x5C);
	for(cnt1 = 0; cnt1 < 30; cnt1 = cnt1 + 2)
		DataEEWrite(memo[cnt1] + (memo[cnt1 + 1] << 8), cnt1/2 + 0x5D);
}

void
read_nbrs(void)
{
	for(cnt1 = 0; cnt1 < 16; cnt1++)
		nbrs[cnt1] = DataEERead(cnt1 + 0x80);
}

void
write_nbrs(void)
{
	for(cnt1 = 0; cnt1 < 16; cnt1++)
		DataEEWrite(nbrs[cnt1], cnt1 + 0x80);
}

void
write_outport(void)
{
	unsigned int outp = 0;
	if(OUT1)
		outp = outp | 1;
	if(OUT2)
		outp = outp | 2;
	DataEEWrite(outp, 0x01);
}

void
read_outport(void)
{
	unsigned int outp;
	outp = DataEERead(0x01);
	if(outp & 1)
		OUT1 = 1;
	else
		OUT1 = 0;
	if(outp & 2)
		OUT2 = 1;
	else
		OUT2 = 0;
}

unsigned int
find_pnbr(char *dest)
{
	*dest = 0;
	ibp = strchr(ibp, '0');
	if(ibp == NULL)
		return 0;
	tmpp1 = ibp;
	while(isdigit(*tmpp1))
		tmpp1++;
	*tmpp1 = 0x00;
	if((tmpp1 - ibp) != 10)
		return 0;
	strcpy(dest, ibp);
	ibp = tmpp1;
	return 1;
}

unsigned int
find_nbr(unsigned int fnbr)
{
	for(cnt1 = 0; cnt1 < 16; cnt1++)
		if(nbrs[cnt1] == fnbr)
			return cnt1;
	return 0xFFFF;
}

unsigned int
add_nbr(unsigned int anbr)
{
	if(find_nbr(anbr) != 0xFFFF)
		return 0;
	if(find_nbr(0) == 0xFFFF)
		return 0;
	nbrs[(unsigned int)find_nbr(0)] = anbr;
	return 1;
}

unsigned int
del_nbr(unsigned int dnbr)
{
	if(find_nbr(dnbr) == 0xFFFF)
		return 0;
	nbrs[(unsigned int)find_nbr(dnbr)] = 0;
	return 1;
}

void
send_cmds(const char *cmd)
{
	while(*cmd)
	{
		U1TXREG = *cmd++;
		while(!U1STAbits.TRMT);
	}
}

unsigned int
ans_t(unsigned int tmscaa)
{
	ibs = 0;
	ibp = &ib[0];
	ib[0] = 0;
	RTS = 0;
	TMR2 = 0;
	cnt100ms = tmscaa;
	while(ib[0] == 0)
	{
		if(cnt100ms == 0)
		{
			RTS = 1;
			return 0;
		}
	}
	TMR3 = 0;
	tmr3_f = 0;
	while(!tmr3_f);
	if(!ibs)
	{
		RTS = 1;
		*ibp++ = 0x00;
		ibp = &ib[0];
	}

	if(strcmp(ibp, "ERROR") == 0)
		return 1;
	if(memcmp(ibp, "+CMTI", 5) == 0)
	{
		in_sms = 1;
	}
	if(strcmp(ibp, "RING") == 0)
	{
		in_rng = 1;
	}
	if(strcmp(ibp, "NO CARRIER") == 0)
		in_rng = 0;
	if(memcmp(ibp, "+CLIP", 5) == 0)
	{
		if(!find_pnbr(in_call_nbr))
			return 1;
		if(strcmp(in_call_nbr, nbr1) == 0)
			mrc = 1;
		if(strcmp(in_call_nbr, nbr2) == 0)
			mrc = 1;
		if(strcmp(in_call_nbr, nbr3) == 0)
			mrc = 1;
	}
	return 10;
}

unsigned int
send_cmds_a(const char *cmd, unsigned int tmsca,const char *ret)
{
	cnt100ms2 = tmsca;
	while(ans_t(1) != 0);
	send_cmds(cmd);
	do{
		if(ans_t(tmsca) == 1)
			return 0;
		if(cnt100ms2 == 0)
			return 0;
	}while(memcmp(&ib[0], ret,strlen(ret)) != 0);
	return 1;
}

unsigned int
get_IMEI(void)
{
	unsigned int ret, ret1;

	send_cmds("AT+GSN\r");
	do{
		ret = ans_t(10);
		ret1 = 0;
		for(cnt1 = 0; cnt1 < 15; cnt1++)
		{
			if(!isdigit(*ibp++))
				ret1 = 1;
		}
		if(ret1 == 0)
		{
			strcpy(IMEI, ib);
			return 1;
		}
	}while(ret != 0);
	return 0;
}

unsigned int
gsm_off_t(void)
{
	unsigned int ret;

	ret = send_cmds_a("AT+CFUN?\r", 10, "+CFUN");
	if(ret == 0)
		return 1;
	if(ib[7] == '0')
		return 1;
	GSM_ON = 0;
	Delay100ms(15);
	GSM_ON = 1;
	of_cnt = 300;
	do{
		ans_t(1);
		of_cnt--;
		if(of_cnt == 0)
			return 0;
	}while(strcmp(ibp, "NORMAL POWER DOWN") != 0);
	of_cnt = 300;
	do{
		ans_t(1);
		of_cnt--;
		if(of_cnt == 0)
			return 1;
	}while(strcmp(ibp, "RDY") != 0);
	Delay100ms(100);
	return 1;
}

unsigned int
gsm_on(void)
{
	send_cmds_a("\x1A", 20, "SEND OK");
	if(send_cmds_a("AT+CFUN?\r", 20, "+CFUN") == 1)
	{
		if(ib[7] == '1')
			return 1;
	}
	if(send_cmds_a("AT+CFUN?\r", 20, "+CFUN") == 1)
	{
		if(ib[7] == '1')
			return 1;
	}
on1:
	GSM_ON = 0;
	Delay100ms(25);
	GSM_ON = 1;

	of_cnt = 100;
	do{
		ans_t(1);
		of_cnt--;
		if(of_cnt == 0)
			goto on1;
	}while((strcmp(ibp, "RDY") != 0) && (strcmp(ibp, "NORMAL POWER DOWN") != 0) && (memcmp(ibp, "From", 4) != 0));
	if(strcmp(ibp, "NORMAL POWER DOWN") == 0)
		goto on1;
	of_cnt = 150;
	do{
		ans_t(1);
		of_cnt--;
		if(of_cnt == 0)
			break;
	}while(strcmp(ibp, "Call Ready") != 0);
	Delay100ms(50);
	of_cnt = 0;
	while(1)
	{
		send_cmds_a("at+creg?\r", 10, "+CREG: 0,1");
		if((ib[9] == '1') || (ib[9] == '5'))
			break;	
		of_cnt++;
		Delay100ms(50);
		if(of_cnt == 6)
			break;
	}
	return 1;
}

unsigned int
get_temp(void)
{
	if(send_cmds_a("AT+CMTE?\r", 10, "+CMTE") == 0)
		return 0;
	ibp = strchr(ibp, ',');
	if(ibp == NULL)
		return 0;
	ibp++;
	if(*ibp == '-')
	{
		ibp++;
		if(!isdigit(*ibp))
			return 0;
		Tm300 = atoi(ibp);
		Tm300 = 0 - Tm300;
	}
	else
	{
		if(!isdigit(*ibp))
			return 0;
		Tm300 = atoi(ibp);
	}
	return 1;
}

void
get_adc(void)
{
	unsigned int cnt, Vtmp = 0;

	AD1CHS = 0x0001;
	Vtmp = 0;
	for(cnt = 0; cnt < 8; cnt++)
	{
		AD1CON1bits.SAMP = 1;
		Delay100ms(1);
		AD1CON1bits.SAMP = 0;
		while (!AD1CON1bits.DONE);
		Vtmp = Vtmp + (ADC1BUF0 >> 2);
	}
	Tm = (signed char)((((Vtmp / 8) * 9.8039) - 500) / 10);

	AD1CHS = 0x0004;
	Vtmp = 0;
	for(cnt = 0; cnt < 8; cnt++)
	{
		AD1CON1bits.SAMP = 1;
		Delay100ms(1);
		AD1CON1bits.SAMP = 0;
		while (!AD1CON1bits.DONE);
		Vtmp = Vtmp + (ADC1BUF0 >> 2);
	}
	status_a1 = Vtmp / 8;

	AD1CHS = 0x0005;
	Vtmp = 0;
	for(cnt = 0; cnt < 8; cnt++)
	{
		AD1CON1bits.SAMP = 1;
		Delay100ms(1);
		AD1CON1bits.SAMP = 0;
		while (!AD1CON1bits.DONE);
		Vtmp = Vtmp + (ADC1BUF0 >> 2);
	}
	status_a2 = Vtmp / 8;
}

unsigned int
get_cash(void)
{
	sprintf(tmp_buf, "ATD%s\r", m_nbr);
	if(send_cmds_a(tmp_buf, 300, "+CUSD") == 0)
		return 0;
	ibp = strchr(ibp, '\"');
	if(ibp == 0)
		return 0;
	ibp++;
	while(!isdigit(*ibp))
	{
		ibp++;
		if(*ibp == 0)
			return 0;
	}
	Money = atoi(ibp);
	Money = Money * 100;
	while(isdigit(*ibp))
		ibp++;
	if((*ibp != '.') && (*ibp != ','))
		return 1;
	ibp++;
	if(!isdigit(*ibp))
		return 1;
	Money = Money + atoi(ibp);
	return 1;
}

unsigned int
send_gprs(void)
{
	send_cmds_a("at+cipclose\r", 10, "CLOSE OK");
	if(send_cmds_a("at+cgatt?\r", 10, "+CGATT:") == 0)
		return 0;
	if(ib[8] != '1')
		return 10;
	if(send_cmds_a("at+cipstatus\r", 10, "STATE:") == 0)
		return 0;
	ibp = ibp + 7;
	if((strcmp(ibp, "IP STATUS") != 0) && (strcmp(ibp, "IP CLOSE") != 0))
	{
		send_cmds_a("at+cipshut\r", 20, "SHUT OK");
		if(APN_user[0] == 0)
			sprintf(tmp_buf, "at+cipcsgp=1,\"%s\"\r",APN);
		else
			sprintf(tmp_buf, "at+cipcsgp=1,\"%s\",\"%s\",\"%s\"\r", APN, APN_user, APN_pass);
		if(send_cmds_a(tmp_buf, 10, "OK") == 0)
			return 0;
		if(send_cmds_a("at+cstt\r", 10, "OK") == 0)
			return 0;
		if(send_cmds_a("at+ciicr\r", 200, "OK") == 0)
			return 0;
		send_cmds_a("at+cifsr\r", 20, "88");
	}

	sprintf(tmp_buf, "at+cipstart=\"tcp\",\"%u.%u.%u.%u\",\"80\"\r", ExIP[0], ExIP[1], ExIP[2], ExIP[3]);
	if(send_cmds_a(tmp_buf, 150, "CONNECT") == 1)
	{
		if(strcmp(ibp, "CONNECT OK") != 0)
			return 10;
	}
	else
		return 10;

	if(send_cmds_a("at+cipsend\r", 10, "> ") == 0)
		return 0;
	send_cmds(out_gprs);
	send_cmds("\x1A");
	tmscag = 250;
	do{
		if(ans_t(1) == 1)
			return 10;
		tmscag--;
		if(tmscag == 0)
			return 10;
	}while(strcmp(&ib[0], "RECIEVE OK") != 0);
	send_cmds_a("at+cipclose\r", 10, "CLOSE OK");
	return 1;
}

unsigned int
set_time_gsm(void)
{
	sprintf(tmp_buf, "at+cclk=\"%2.2u/%2.2u/%2.2u,%2.2u:%2.2u:00+00\"\r", Year, Month, Day, Hour, Minute);
	if(send_cmds_a(tmp_buf,10,"OK") == 0)
		return 0;
	return 1;
}

unsigned int
get_time_gsm(void)
{
	unsigned int Yeart, Montht, Dayt, Hourt;

	send_cmds_a("AT+CCLK?\r",10,"+CCLK");
	ibp = strchr(ibp, '"');
	if(ibp == NULL)
		return 0;
	ibp++;
	if(!isdigit(*ibp))
		return 0;
	Yeart = atoi(ibp);
	ibp = strchr(ibp, '/');
	if(ibp == NULL)
		return 0;
	ibp++;
	if(!isdigit(*ibp))
		return 0;
	Montht = atoi(ibp);
	ibp = strchr(ibp, '/');
	if(ibp == NULL)
		return 0;
	ibp++;
	if(!isdigit(*ibp))
		return 0;
	Dayt = atoi(ibp);
	ibp = strchr(ibp, ',');
	if(ibp == NULL)
		return 0;
	ibp++;
	if(!isdigit(*ibp))
		return 0;
	Hourt = atoi(ibp);
	ibp = strchr(ibp, ':');
	if(ibp == NULL)
		return 0;
	ibp++;
	if(!isdigit(*ibp))
		return 0;
	Minute = atoi(ibp);
	Year = Yeart; Month = Montht; Day = Dayt; Hour = Hourt;
	return 1;
}

unsigned int
get_t_inet(void)
{
	sprintf(out_gprs, "POST http://www.elvin.com.ua/gsm/insert1.php?a=%s&b=%u&c=%d&d=%u&e=%u&f=%u&g=%d HTTP/1.0\r\n\r\n", IMEI, Money, Tm, status_d, status_a1, status_a2, Tm300);
	send_cmds_a("at+cipclose\r", 10, "CLOSE OK");
	if(send_cmds_a("at+cgatt?\r", 10, "+CGATT:") == 0)
		return 0;
	if(ib[8] != '1')
		return 10;
	if(send_cmds_a("at+cipstatus\r", 10, "STATE:") == 0)
		return 0;
	ibp = ibp + 7;
	if((strcmp(ibp, "IP STATUS") != 0) && (strcmp(ibp, "IP CLOSE") != 0))
	{
		send_cmds_a("at+cipshut\r", 20, "SHUT OK");
		if(APN_user[0] == 0)
			sprintf(tmp_buf, "at+cipcsgp=1,\"%s\"\r",APN);
		else
			sprintf(tmp_buf, "at+cipcsgp=1,\"%s\",\"%s\",\"%s\"\r", APN, APN_user, APN_pass);
		if(send_cmds_a(tmp_buf, 10, "OK") == 0)
			return 0;
		if(send_cmds_a("at+cstt\r", 10, "OK") == 0)
			return 0;
		if(send_cmds_a("at+ciicr\r", 200, "OK") == 0)
			return 0;
		send_cmds_a("at+cifsr\r", 20, "88");
	}

	sprintf(tmp_buf, "at+cipstart=\"tcp\",\"%u.%u.%u.%u\",\"80\"\r", ExIP[0], ExIP[1], ExIP[2], ExIP[3]);
	if(send_cmds_a(tmp_buf, 150, "CONNECT") == 1)
	{
		if(strcmp(ibp, "CONNECT OK") != 0)
			return 10;
	}
	else
		return 10;

	if(send_cmds_a("at+cipsend\r", 10, "> ") == 0)
		return 0;
	send_cmds(out_gprs);
	send_cmds("\x1A");
	tmscag = 250;
	do{
		if(ans_t(1) == 1)
			return 10;
		tmscag--;
		if(tmscag == 0)
			return 10;
	}while(memcmp(&ib[0], "Time ", 5) != 0);

	ibp = strchr(ibp, ' ');
	if(ibp == NULL)
		return 0;
	ibp++;
	if(!isdigit(*ibp))
		return 10;
	Year = atoi(ibp);
	Year = Year % 100;
	ibp = strchr(ibp, '/');
	if(ibp == NULL)
		return 0;
	ibp++;
	if(!isdigit(*ibp))
		return 10;
	Month = atoi(ibp);
	ibp = strchr(ibp, '/');
	if(ibp == NULL)
		return 0;
	ibp++;
	if(!isdigit(*ibp))
		return 10;
	Day = atoi(ibp);
	ibp = strchr(ibp, ' ');
	if(ibp == NULL)
		return 0;
	ibp++;
	if(!isdigit(*ibp))
		return 10;
	Hour = atoi(ibp);
	ibp = strchr(ibp, ':');
	if(ibp == NULL)
		return 0;
	ibp++;
	if(!isdigit(*ibp))
		return 10;
	Minute = atoi(ibp);

	set_time_gsm();
	return 1;
}

unsigned char
send_sms(void)
{
	if(out_sms_nbr[0] != 0)
	{
		send_cmds("at+cmgs=\"");
		send_cmds(out_sms_nbr);
		if(send_cmds_a("\"\r", 10, "> ") == 0)
			return 0;
		send_cmds(out_sms);
		if(send_cmds_a("\x1A", 300, "+CMGS") == 0)
			return 0;
		return 1;
	}
	return 0;
}

void
send_list_nbrs(void)
{
	tmpp1 = out_sms;
	tmpp1 = tmpp1 + sprintf(tmpp1, "Numbers: ");
	for(cnt1 = 0; cnt1 < 16; cnt1++)
	{
		if(nbrs[cnt1] != 0)
			tmpp1 = tmpp1 + sprintf(tmpp1, "%u,", nbrs[cnt1]);
	}
	if(tmpp1 == out_sms + 9)
		sprintf(out_sms, "List empty");
	else
	{
		tmpp1--;
		*tmpp1 = 0;
	}
	strcpy(out_sms_nbr, in_sms_nbr);
	send_sms();
}

void
send_list_pnbrs(void)
{
	tmpp1 = out_sms;
	tmpp1 = tmpp1 + sprintf(tmpp1, "Phone numbers:\r\n");
	tmpp1 = tmpp1 + sprintf(tmpp1, "1-");
	if(nbr1[0] != 0)
		tmpp1 = tmpp1 + sprintf(tmpp1, " %s\r\n", nbr1);
	else
		tmpp1 = tmpp1 + sprintf(tmpp1, "Empty\r\n");

	tmpp1 = tmpp1 + sprintf(tmpp1, "2-");
	if(nbr2[0] != 0)
		tmpp1 = tmpp1 + sprintf(tmpp1, " %s\r\n", nbr2);
	else
		tmpp1 = tmpp1 + sprintf(tmpp1, "Empty\r\n");

	tmpp1 = tmpp1 + sprintf(tmpp1, "3-");
	if(nbr3[0] != 0)
		tmpp1 = tmpp1 + sprintf(tmpp1, " %s", nbr3);
	else
		tmpp1 = tmpp1 + sprintf(tmpp1, "Empty");

	strcpy(out_sms_nbr, in_sms_nbr);
	send_sms();
}

unsigned int
read_SMS_c(void)
{
	unsigned char IPt[4];
	unsigned char tmpd, tmph, tmpm;
	char tmpnbr[11];
	unsigned int tmpsms;
	char m_nbrt[11], APNt[21], APN_usert[11], APN_passt[11];

	ibp = strchr(ibp, '#');
	if(ibp == NULL)
		return 0;
	ibp++;
	if(memcmp(ibp, "spass##", 7) == 0)
	{
		strcpy(out_sms_nbr, in_sms_nbr);
		sprintf(out_sms, "pass:\n%s", pass);
		send_sms();
		return 10;
	}
	if(memcmp(ibp, pass, 4) != 0)
		return 1;
	ibp = ibp + 4;
	if(*ibp != '#')
		return 2;
	ibp++;

	if(memcmp(ibp, "setparam#", 9) == 0)
	{
		ibp = ibp + 9;
		SMS_ret = SMS_ret | 8;
		if(!isdigit(*ibp))
			return 2;
		IPt[0] = atoi(ibp);
		if(IPt[0] > 255)
			return 2;
		ibp = strchr(ibp, '.');
		if(ibp == NULL)
			return 2;
		ibp++;
		if(!isdigit(*ibp))
			return 2;
		IPt[1] = atoi(ibp);
		if(IPt[1] > 255)
			return 2;
		ibp = strchr(ibp, '.');
		if(ibp == NULL)
			return 2;
		ibp++;
		if(!isdigit(*ibp))
			return 2;
		IPt[2] = atoi(ibp);
		if(IPt[2] > 255)
			return 2;
		ibp = strchr(ibp, '.');
		if(ibp == NULL)
			return 2;
		ibp++;
		if(!isdigit(*ibp))
			return 2;
		IPt[3] = atoi(ibp);
		if(IPt[3] > 255)
			return 2;
		while(isdigit(*ibp))
			ibp++;
		if(*ibp != '#')
			return 2;
		ibp++;
		if(!isdigit(*ibp))
			return 2;
		tmpp1 = ibp;		
		while(isdigit(*tmpp1))
			tmpp1++;
		if(*tmpp1 != '#')
			return 2;
		if((tmpp1 - ibp) > 10)
			return 2;
		memcpy(m_nbrt, ibp, (tmpp1 - ibp));
		m_nbrt[tmpp1 - ibp] = 0;
		ibp = tmpp1 + 1;
		tmpp1 = strchr(ibp, '#');
		if(tmpp1 == NULL)
			return 2;
		if((tmpp1 - ibp) > 20)
			return 2;
		memcpy(APNt, ibp, (tmpp1 - ibp));
		APNt[tmpp1 - ibp] = 0;
		ibp = tmpp1 + 1;
		if(*ibp != '#')
		{
			tmpp1 = strchr(ibp, '#');
			if(tmpp1 == NULL)
				return 2;
			if((tmpp1 - ibp) > 10)
				return 2;
			memcpy(APN_usert, ibp, (tmpp1 - ibp));
			APN_usert[tmpp1 - ibp] = 0;
			ibp = tmpp1 + 1;
			tmpp1 = strchr(ibp, '#');
			if(tmpp1 == NULL)
				return 2;
			if((tmpp1 - ibp) > 10)
				return 2;
			memcpy(APN_passt, ibp, (tmpp1 - ibp));
			APN_passt[tmpp1 - ibp] = 0;
			ibp = tmpp1 + 1;
			if(*ibp != '#')
				return 2;
		}
		else
		{
			APN_usert[0] = 0;
			APN_passt[0] = 0;
		}
		ExIP[0] = IPt[0];ExIP[1] = IPt[1];ExIP[2] = IPt[2];ExIP[3] = IPt[3];
		sprintf(m_nbr, "*%s#", m_nbrt);
		strcpy(APN, APNt);
		strcpy(APN_user, APN_usert);
		strcpy(APN_pass, APN_passt);
		if(Settings & 4)
		{
			strcpy(out_sms_nbr, in_sms_nbr);
			sprintf(out_sms, "IP:\n%u.%u.%u.%u\nM_nbr:%s\nAPN:\n%s\nAPNusr:%s\nAPNpsd:%s", ExIP[0], ExIP[1], ExIP[2], ExIP[3], m_nbr, APN, APN_user, APN_pass);
			send_sms();
		}
		write_param();
		return 10;
	}

	if(memcmp(ibp, "setnbr#", 7) == 0)
	{
		ibp = ibp + 7;
		SMS_ret = SMS_ret | 1;
		if(*ibp == '#')
		{
			nbr1[0] = 0;
			nbr2[0] = 0;
			nbr3[0] = 0;
		}
		else
		{
			if(find_pnbr(tmpnbr))
			{
				strcpy(nbr1, tmpnbr);
				ibp++;
				if(find_pnbr(tmpnbr))
				{
					strcpy(nbr2, tmpnbr);
					ibp++;
					if(find_pnbr(tmpnbr))
						strcpy(nbr3, tmpnbr);
					else
						nbr3[0] = 0;
				}
				else
				{
					nbr2[0] = 0;
					nbr3[0] = 0;
				}
			}
			else
				return 2;
		}
		if(Settings & 4)
			send_list_pnbrs();
		write_param();
		return 10;
	}

	if(memcmp(ibp, "setpass#", 8) == 0)
	{
		ibp = ibp + 8;
		SMS_ret = SMS_ret | 2;
		tmpp1 = ibp + 4;
		if(*tmpp1 != '#')
			return 2;
		tmpp1++;
		if(*tmpp1 != '#')
			return 2;
		memcpy(pass, ibp, 4);
		if(Settings & 4)
		{
			strcpy(out_sms_nbr, in_sms_nbr);
			sprintf(out_sms, "New pass:\n%s", pass);
			send_sms();
		}
		write_param();
		return 10;
	}

	if(memcmp(ibp, "status##", 8) == 0)
	{
		strcpy(out_sms_nbr, in_sms_nbr);
		sprintf(out_sms, "IP:\n%u.%u.%u.%u\nSetings: %u\nM_nbr:%s\nAPN:\n%s\nAPN username:%s\nAPN password:%s", ExIP[0], ExIP[1], ExIP[2], ExIP[3], Settings, m_nbr, APN, APN_user, APN_pass);
		send_sms();
		return 10;
	}

	if(memcmp(ibp, "settings#", 9) == 0)
	{
		ibp = ibp + 9;
		SMS_ret = SMS_ret | 32;
		if(!isdigit(*ibp))
			return 2;
		tmpp1 = ibp;
		while(isdigit(*tmpp1))
			tmpp1++;
		if((tmpp1 - ibp) > 5)
			return 2;
		if(*tmpp1 != '#')
			return 2;
		tmpp1++;
		if(*tmpp1 != '#')
			return 2;
		Settings = atoi(ibp);
		if(Settings & 4)
		{
			strcpy(out_sms_nbr, in_sms_nbr);
			sprintf(out_sms, "Settings: %u", Settings);
			send_sms();
		}
		write_param();
		return 10;
	}

	if(memcmp(ibp, "ussd#", 5) == 0)
	{
		ibp = ibp + 5;
		if(*ibp != '(')
			return 2;
		ibp++;
		tmpp1 = strchr(ibp, ')');
		if(tmpp1 == NULL)
			return 2;
		*tmpp1 = 0;
		tmpp1++;
		if(*tmpp1 != '#')
			return 2;
		tmpp1++;
		if(*tmpp1 != '#')
			return 2;
		sprintf(tmp_buf, "ATD%s\r", ibp);
		if(send_cmds_a(tmp_buf, 300, "+CUSD") == 0)
			return 2;
		ibp = strchr(ibp, '\"');
		if(ibp == 0)
			return 2;
		ibp++;
		tmpp1 = ibp;
		tmpp1 = strchr(tmpp1, '\"');
		if(tmpp1 == NULL)
			return 2;
		*tmpp1 = 2;
		tmpp1 = ibp;
		while(*tmpp1)
		{
			if(iscntrl(*tmpp1))
				*tmpp1 = 0x20;
			tmpp1++;
		}
		strcpy(out_sms_nbr, in_sms_nbr);
		strcpy(out_sms, ibp);
		send_sms();
		return 10;
	}

	if(memcmp(ibp, "setperiod#", 10) == 0)
	{
		ibp = ibp + 10;
		SMS_ret = SMS_ret | 4;
		if(!isdigit(*ibp))
			return 2;
		tmpp1 = ibp;
		while(isdigit(*tmpp1))
			tmpp1++;
		if((tmpp1 - ibp) > 5)
			return 2;
		if(*tmpp1 != '#')
			return 2;
		tmpp1++;
		if(*tmpp1 != '#')
			return 2;
		Period = atoi(ibp);
		if(Settings & 4)
		{
			strcpy(out_sms_nbr, in_sms_nbr);
			sprintf(out_sms, "Period: %u min", Settings);
			send_sms();
		}
		write_param();
		p_cnts = 0;
		p_cntm = 0;
		mrc = 1;
		return 10;
	}

	if(memcmp(ibp, "setout#", 7) == 0)
	{
		ibp = ibp + 7;
		SMS_ret = SMS_ret | 16;
		tmpp1 = ibp;
		if((*tmpp1 != '0') && (*tmpp1 != '1'))
			return 2;
		tmpp1++;
		if((*tmpp1 != '0') && (*tmpp1 != '1'))
			return 2;
		tmpp1++;
		if(*tmpp1 != '#')
			return 2;
		tmpp1++;
		if(*tmpp1 != '#')
			return 2;
		if(*ibp == '1')
			OUT1 = 1;
		else
			OUT1 = 0;
		ibp++;
		if(*ibp == '1')
			OUT2 = 1;
		else
			OUT2 = 0;
		if(Settings & 4)
		{
			strcpy(out_sms_nbr, in_sms_nbr);
			sprintf(out_sms, "Out1:%u;\r\nOut2:%u;", OUT1, OUT2);
			send_sms();
		}
		write_outport();
		return 10;
	}

	if(memcmp(ibp, "getdata##", 9) == 0)
	{
		strcpy(out_sms_nbr, in_sms_nbr);
		getdata = 1;
		return 10;
	}

	if(memcmp(ibp, "fullreset##", 11) == 0)
	{
		if(Settings & 4)
		{
			strcpy(out_sms_nbr, in_sms_nbr);
			strcpy(out_sms, "Full reset done");
			send_sms();
		}
		DataEEWrite(0x0000, 0x00);
		reset_f = 1;
		return 10;
	}

	if(memcmp(ibp, "reset##", 7) == 0)
	{
		if(Settings & 4)
		{
			strcpy(out_sms_nbr, in_sms_nbr);
			strcpy(out_sms, "Reset done");
			send_sms();
		}
		reset_f = 1;
		return 10;
	}

	if(memcmp(ibp, "setsms#", 7) == 0)
	{
		ibp = ibp + 7;
		if(!isdigit(*ibp))
			return 2;
		tmpp1 = ibp;
		while(isdigit(*tmpp1))
			tmpp1++;
		if((tmpp1 - ibp) > 5)
			return 2;
		if(*tmpp1 != '#')
			return 2;
		tmpsms = atoi(ibp);
		tmpp1++;
		ibp = tmpp1;
		while(isdigit(*tmpp1))
			tmpp1++;
		if((tmpp1 - ibp) > 5)
			return 2;
		if(*tmpp1 != '#')
			return 2;
		tmpp1++;
		if(*tmpp1 != '#')
			return 2;
		Templ = atoi(ibp);
		SendSMS = tmpsms;
		if(Settings & 4)
		{
			strcpy(out_sms_nbr, in_sms_nbr);
			sprintf(out_sms, "SendSMS:%u\n\rTemp limit: %u", SendSMS, Templ);
			send_sms();
		}
		write_param();
		return 10;
	}

	if(memcmp(ibp, "setmemo#", 8) == 0)
	{
		ibp = ibp + 8;
		tmpp1 = strchr(ibp, '#');
		if(tmpp1 == NULL)
			return 2;
		*tmpp1 = 0;
		if((tmpp1 - ibp) > 30)
			return 2;
		strcpy(memo, ibp);
		if(Settings & 4)
		{
			strcpy(out_sms_nbr, in_sms_nbr);
			sprintf(out_sms, "New memo:\n\r%s", memo);
			send_sms();
		}
		write_param();
		return 10;
	}

	if(memcmp(ibp, "addnbr#", 7) == 0)
	{
		ibp = ibp + 7;
		while(isdigit(*ibp))
		{
			add_nbr(atoi(ibp));
			ibp = strchr(ibp, '#');
			if(ibp == NULL)
				break;
			ibp++;
		}
		if(Settings & 4)
		{
			send_list_nbrs();
		}
		write_nbrs();
		return 10;
	}

	if(memcmp(ibp, "delnbr#", 7) == 0)
	{
		ibp = ibp + 7;
		while(isdigit(*ibp))
		{
			del_nbr(atoi(ibp));
			ibp = strchr(ibp, '#');
			if(ibp == NULL)
				break;
			ibp++;
		}
		if(Settings & 4)
		{
			send_list_nbrs();
		}
		write_nbrs();
		return 10;
	}

	if(memcmp(ibp, "delnbrs##", 9) == 0)
	{
		for(cnt1 = 0; cnt1 < 16; cnt1++)
			nbrs[cnt1] = 0;
		if(Settings & 4)
		{
			strcpy(out_sms, "All numbers deleted");
			strcpy(out_sms_nbr, in_sms_nbr);
			send_sms();
		}
		write_nbrs();
		return 10;
	}

	if(memcmp(ibp, "addnbrs#", 8) == 0)
	{
		ibp = ibp + 8;
		for(cnt1 = 0; cnt1 < 16; cnt1++)
			nbrs[cnt1] = 0;
		while(isdigit(*ibp))
		{
			add_nbr(atoi(ibp));
			ibp = strchr(ibp, '#');
			if(ibp == NULL)
				break;
			ibp++;
		}
		if(Settings & 4)
		{
			send_list_nbrs();
		}
		write_nbrs();
		return 10;
	}

	if(memcmp(ibp, "getnbrs##", 7) == 0)
	{
		send_list_nbrs();
		return 10;
	}

	if(memcmp(ibp, "setalrm1#", 9) == 0)
	{
		ibp = ibp + 9;
		for(cnt1 = 0; cnt1 < 10; cnt1++)
		{
			if(!isdigit(*ibp))
				break;
			tmpd = atoi(ibp);
			ibp = strchr(ibp, '/');
			if(ibp == NULL)
				break;
			ibp++;
			if(!isdigit(*ibp))
				break;
			tmph = atoi(ibp);
			ibp = strchr(ibp, ':');
			if(ibp == NULL)
				break;
			ibp++;
			if(!isdigit(*ibp))
				break;
			tmpm = atoi(ibp);
			alrm1_day[cnt1] = tmpd;
			alrm1_hour[cnt1] = tmph;
			alrm1_min[cnt1] = tmpm;
			alrm_done[cnt1] = alrm_done[cnt1] & 0b11111110;
			ibp = strchr(ibp, '#');
			if(ibp == NULL)
				break;
			ibp++;
		}
		for(; cnt1 < 10; cnt1++)
		{
			alrm_done[cnt1] = alrm_done[cnt1] & 0b11111110;
			alrm1_day[cnt1] = 0;
		}

		tmpp1 = out_sms;
		tmpp1 = tmpp1 + sprintf(tmpp1, "Alarm dates:");
		for(cnt1 = 0; cnt1 < 10; cnt1++)
		{
			if(alrm1_day[cnt1] == 0)
				break;
			tmpp1 = tmpp1 + sprintf(tmpp1, " %u/%u:%u-%u", alrm1_day[cnt1], alrm1_hour[cnt1], alrm1_min[cnt1], alrm1_do[cnt1]);
		}
		if(cnt1 == 0)
			sprintf(out_sms, "List empty");
		day_done = 0;
		strcpy(out_sms_nbr, in_sms_nbr);
		send_sms();
		write_param();
		return 10;
	}

	if(memcmp(ibp, "getalrm1##", 10) == 0)
	{
		tmpp1 = out_sms;
		tmpp1 = tmpp1 + sprintf(tmpp1, "Alarm dates:");
		for(cnt1 = 0; cnt1 < 10; cnt1++)
		{
			if(alrm1_day[cnt1] == 0)
				break;
			tmpp1 = tmpp1 + sprintf(tmpp1, " %u/%u:%u", alrm1_day[cnt1], alrm1_hour[cnt1], alrm1_min[cnt1]);
		}
		if(cnt1 == 0)
			sprintf(out_sms, "List empty");
		strcpy(out_sms_nbr, in_sms_nbr);
		send_sms();
		return 10;
	}

	if(memcmp(ibp, "setalrm2#", 9) == 0)
	{
		ibp = ibp + 9;
		for(cnt1 = 0; cnt1 < 10; cnt1++)
		{
			if(!isdigit(*ibp))
				break;
			tmph = atoi(ibp);
			ibp = strchr(ibp, ':');
			if(ibp == NULL)
				break;
			ibp++;
			if(!isdigit(*ibp))
				break;
			tmpm = atoi(ibp);
			alrm2_hour[cnt1] = tmph;
			alrm2_min[cnt1] = tmpm;
			alrm_done[cnt1] = alrm_done[cnt1] & 0b11111101;
			ibp = strchr(ibp, '#');
			if(ibp == NULL)
				break;
			ibp++;
		}
		for(; cnt1 < 10; cnt1++)
		{
			alrm_done[cnt1] = alrm_done[cnt1] & 0b11111101;
			alrm2_hour[cnt1] = 0xFF;
		}

		tmpp1 = out_sms;
		tmpp1 = tmpp1 + sprintf(tmpp1, "Alarm times:");
		for(cnt1 = 0; cnt1 < 10; cnt1++)
		{
			if(alrm2_hour[cnt1] == 0xFF)
				break;
			tmpp1 = tmpp1 + sprintf(tmpp1, " %u:%u-%u", alrm2_hour[cnt1], alrm2_min[cnt1], alrm2_do[cnt1]);
		}
		if(cnt1 == 0)
			sprintf(out_sms, "List empty");
		day_done = 0;
		strcpy(out_sms_nbr, in_sms_nbr);
		send_sms();
		write_param();
		return 10;
	}

	if(memcmp(ibp, "getalrm2##", 10) == 0)
	{
		tmpp1 = out_sms;
		tmpp1 = tmpp1 + sprintf(tmpp1, "Alarm times:");
		for(cnt1 = 0; cnt1 < 10; cnt1++)
		{
			if(alrm2_hour[cnt1] == 0xFF)
				break;
			tmpp1 = tmpp1 + sprintf(tmpp1, " %u:%u", alrm2_hour[cnt1], alrm2_min[cnt1]);
		}
		if(cnt1 == 0)
			sprintf(out_sms, "List empty");
		strcpy(out_sms_nbr, in_sms_nbr);
		send_sms();
		return 10;
	}

	return 3;
}

unsigned int
read_SMS_t(void)
{
	unsigned int ret;

	send_cmds_a("at+cmgr=1\r", 5, "+CMGR");
	find_pnbr(in_sms_nbr);
	ans_t(1);
	ret = read_SMS_c();
	if(Settings & 4)
	{
		strcpy(out_sms_nbr, in_sms_nbr);
		if(ret == 1)
		{
			strcpy(out_sms, "Wrong password");
			send_sms();
		}
		if(ret == 2)
		{
			strcpy(out_sms, "Error in command");
			send_sms();
		}
		if(ret == 3)
		{
			strcpy(out_sms, "Uknown command");
			send_sms();
		}
	}
	send_cmds_a("AT+CMGD=1\r", 10, "OK");

	send_cmds_a("at+cmgr=2\r", 5, "+CMGR");
	find_pnbr(in_sms_nbr);
	ans_t(1);
	ret = read_SMS_c();
	if(Settings & 4)
	{
		strcpy(out_sms_nbr, in_sms_nbr);
		if(ret == 1)
		{
			strcpy(out_sms, "Wrong password");
			send_sms();
		}
		if(ret == 2)
		{
			strcpy(out_sms, "Error in command");
			send_sms();
		}
		if(ret == 3)
		{
			strcpy(out_sms, "Uknown command");
			send_sms();
		}
	}
	send_cmds_a("AT+CMGD=2\r", 10, "OK");

	send_cmds_a("at+cmgr=3\r", 5, "+CMGR");
	find_pnbr(in_sms_nbr);
	ans_t(1);
	ret = read_SMS_c();
	if(Settings & 4)
	{
		strcpy(out_sms_nbr, in_sms_nbr);
		if(ret == 1)
		{
			strcpy(out_sms, "Wrong password");
			send_sms();
		}
		if(ret == 2)
		{
			strcpy(out_sms, "Error in command");
			send_sms();
		}
		if(ret == 3)
		{
			strcpy(out_sms, "Uknown command");
			send_sms();
		}
	}
	send_cmds_a("AT+CMGD=3\r", 10, "OK");

	send_cmds_a("at+cmgr=4\r", 5, "+CMGR");
	find_pnbr(in_sms_nbr);
	ans_t(1);
	ret = read_SMS_c();
	if(Settings & 4)
	{
		strcpy(out_sms_nbr, in_sms_nbr);
		if(ret == 1)
		{
			strcpy(out_sms, "Wrong password");
			send_sms();
		}
		if(ret == 2)
		{
			strcpy(out_sms, "Error in command");
			send_sms();
		}
		if(ret == 3)
		{
			strcpy(out_sms, "Uknown command");
			send_sms();
		}
	}
	send_cmds_a("AT+CMGD=4\r", 10, "OK");

	send_cmds_a("at+cmgr=5\r", 5, "+CMGR");
	find_pnbr(in_sms_nbr);
	ans_t(1);
	ret = read_SMS_c();
	if(Settings & 4)
	{
		strcpy(out_sms_nbr, in_sms_nbr);
		if(ret == 1)
		{
			strcpy(out_sms, "Wrong password");
			send_sms();
		}
		if(ret == 2)
		{
			strcpy(out_sms, "Error in command");
			send_sms();
		}
		if(ret == 3)
		{
			strcpy(out_sms, "Uknown command");
			send_sms();
		}
	}
	send_cmds_a("AT+CMGD=5\r", 10, "OK");

	send_cmds_a("at+cmgr=6\r", 5, "+CMGR");
	find_pnbr(in_sms_nbr);
	ans_t(1);
	ret = read_SMS_c();
	if(Settings & 4)
	{
		strcpy(out_sms_nbr, in_sms_nbr);
		if(ret == 1)
		{
			strcpy(out_sms, "Wrong password");
			send_sms();
		}
		if(ret == 2)
		{
			strcpy(out_sms, "Error in command");
			send_sms();
		}
		if(ret == 3)
		{
			strcpy(out_sms, "Uknown command");
			send_sms();
		}
	}
	send_cmds_a("AT+CMGD=6\r", 10, "OK");

	send_cmds_a("at+cmgr=7\r", 5, "+CMGR");
	find_pnbr(in_sms_nbr);
	ans_t(1);
	ret = read_SMS_c();
	if(Settings & 4)
	{
		strcpy(out_sms_nbr, in_sms_nbr);
		if(ret == 1)
		{
			strcpy(out_sms, "Wrong password");
			send_sms();
		}
		if(ret == 2)
		{
			strcpy(out_sms, "Error in command");
			send_sms();
		}
		if(ret == 3)
		{
			strcpy(out_sms, "Uknown command");
			send_sms();
		}
	}
	send_cmds_a("AT+CMGD=7\r", 10, "OK");

	send_cmds_a("at+cmgr=8\r", 5, "+CMGR");
	find_pnbr(in_sms_nbr);
	ans_t(1);
	ret = read_SMS_c();
	if(Settings & 4)
	{
		strcpy(out_sms_nbr, in_sms_nbr);
		if(ret == 1)
		{
			strcpy(out_sms, "Wrong password");
			send_sms();
		}
		if(ret == 2)
		{
			strcpy(out_sms, "Error in command");
			send_sms();
		}
		if(ret == 3)
		{
			strcpy(out_sms, "Uknown command");
			send_sms();
		}
	}
	send_cmds_a("AT+CMGD=8\r", 10, "OK");

	send_cmds_a("at+cmgr=9\r", 5, "+CMGR");
	find_pnbr(in_sms_nbr);
	ans_t(1);
	ret = read_SMS_c();
	if(Settings & 4)
	{
		strcpy(out_sms_nbr, in_sms_nbr);
		if(ret == 1)
		{
			strcpy(out_sms, "Wrong password");
			send_sms();
		}
		if(ret == 2)
		{
			strcpy(out_sms, "Error in command");
			send_sms();
		}
		if(ret == 3)
		{
			strcpy(out_sms, "Uknown command");
			send_sms();
		}
	}
	send_cmds_a("AT+CMGD=9\r", 10, "OK");

	send_cmds_a("at+cmgr=10\r", 5, "+CMGR");
	find_pnbr(in_sms_nbr);
	ans_t(1);
	ret = read_SMS_c();
	if(Settings & 4)
	{
		strcpy(out_sms_nbr, in_sms_nbr);
		if(ret == 1)
		{
			strcpy(out_sms, "Wrong password");
			send_sms();
		}
		if(ret == 2)
		{
			strcpy(out_sms, "Error in command");
			send_sms();
		}
		if(ret == 3)
		{
			strcpy(out_sms, "Uknown command");
			send_sms();
		}
	}
	send_cmds_a("AT+CMGD=10\r", 10, "OK");
	if(reset_f == 1)
		asm("RESET");

	return 1;
}

unsigned int
read_u(unsigned int ext_nbr, unsigned char cmd_nbr)
{
	sprintf(tmp_buf, "/?%4.4u!\r\n", ext_nbr);
	uart_puts2(tmp_buf);
	u2cnt = 0;
	TMR2 = 0;
	cnt100ms = 7;
	while(cb[u2cnt-1] != 0x2F)
	{
		if(cnt100ms == 0)
			return 0;
	}
	u2cnt = 0;
	TMR2 = 0;
	cnt100ms = 7;
	while(cb[u2cnt-1] != 0x0A)
	{
		if(cnt100ms == 0)
			return 0;
	}
	Delay100ms(3);
	uart_puts2("\x06\x30\x35\x31\r\n");
	u2cnt = 0;
	TMR2 = 0;
	cnt100ms = 7;
	while(cb[u2cnt-1] != 0x03)
	{
		if(cnt100ms == 0)
			return 0;
	}
	Delay100ms(3);

	if(cmd_nbr == 0)
		uart_puts2("\x01R2\x02\x30\x30\x30\x30()\x03\x1A");
	else
		uart_puts2("\x01R2\x02\x30\x30\x31\x30()\x03\x1B");
	u2cnt = 0;
	TMR2 = 0;
	cnt100ms = 7;
	while(cb[u2cnt-1] != 0x02)
	{
		if(cnt100ms == 0)
			return 0;
	}
	u2cnt = 0;
	TMR2 = 0;
	cnt100ms = 7;
	while(cb[u2cnt-1] != 0x03)
	{
		if(cnt100ms == 0)
			return 0;
	}

	cbp = strchr((char*)&cb[0], '(');
	cbp++;
	tmpp1 = strchr((char*)cbp, ')');
	*tmpp1 = 0;
	strcpy(&out_gprs[out_gprsp], (char*)cbp);
	Delay100ms(5);
	uart_puts2("\x01\x42\x30\x03\x75");
	Delay100ms(5);
	return 1;
}

void
T1set(void)
{
	T1CON = 0x00; //Stops the Timer1 and reset control reg.
	TMR1 = 0x00; //Clear contents of the timer register
	PR1 = 0xF424; //Load the Period register with the value 0xFFFF
	IPC0bits.T1IP = 0x01; //Setup Timer1 interrupt for desired priority level
	//(This example assigns level 1 priority)
	IFS0bits.T1IF = 0; //Clear the Timer1 interrupt status flag
	IEC0bits.T1IE = 1; //Enable Timer1 interrupts
	T1CONbits.TCKPS1 = 1;
	T1CONbits.TCKPS0 = 1;
	T1CONbits.TON = 1; //Start Timer1 with prescaler settings at 1:1 and
	//clock source set to the internal instruction cycle
}

void
T2set(void)
{
	T2CON = 0x00; //Stops the Timer2 and reset control reg.
	TMR2 = 0x00; //Clear contents of the timer register
	PR2 = 0x61A7; //Load the Period register with the value 0xFFFF
	IPC1bits.T2IP = 0x01; //Setup Timer1 interrupt for desired priority level
	//(This example assigns level 1 priority)
	IFS0bits.T2IF = 0; //Clear the Timer2 interrupt status flag
	IEC0bits.T2IE = 1; //Enable Timer2 interrupts
	T2CONbits.TCKPS1 = 1;
	T2CONbits.TCKPS0 = 0;
	T2CONbits.TON = 1; //Start Timer2 with prescaler settings at 1:256 and
	//clock source set to the internal instruction cycle
}

void
T3set(void)
{
	T3CON = 0x00; //Stops the Timer3 and reset control reg.
	TMR3 = 0x00; //Clear contents of the timer register
	PR3 = 0x61A7; //Load the Period register with the value 0xFFFF
	IPC2bits.T3IP = 0x01; //Setup Timer3 interrupt for desired priority level
	//(This example assigns level 1 priority)
	IFS0bits.T3IF = 0; //Clear the Timer3 interrupt status flag
	IEC0bits.T3IE = 1; //Enable Timer3 interrupts
	T3CONbits.TCKPS1 = 1;
	T3CONbits.TCKPS0 = 0;
	T3CONbits.TON = 1; //Start Timer3 with prescaler settings at 1:256 and
	//clock source set to the internal instruction cycle
}

void
U1set_2400(void)
{
	RPINR18 = 0x1F0C;
	RPOR6 = 0x0300;
	U1MODE = 0b1100000000000000;
	U1STA = 0b0010010100010000;
	U1BRG = 416;				//скорость по GSM
 	IFS0bits.U1RXIF = 0;
	IFS0bits.U1TXIF = 0;
    IEC0bits.U1RXIE = 1;
    IEC0bits.U1TXIE = 0;
}

void
U1set_38400(void)
{
	RPINR18 = 0x1F0C;
	RPOR6 = 0x0300;
	U1MODE = 0b1100000000000000;
	U1STA = 0b0010010100010000;
	U1BRG = 25;				//скорость по GSM
 	IFS0bits.U1RXIF = 0;
	IFS0bits.U1TXIF = 0;
    IEC0bits.U1RXIE = 1;
    IEC0bits.U1TXIE = 0;
}

void U1close(void)
{  
    U1MODEbits.UARTEN = 0;
	
    IEC0bits.U1RXIE = 0;
    IEC0bits.U1TXIE = 0;
	
    IFS0bits.U1RXIF = 0;
    IFS0bits.U1TXIF = 0;
}

void
U2set(void)
{
	RPINR19 = 0x1F05;
	RPOR2 = 0x0005;
	U2MODE = 0b1100000000000000;
	U2STA = 0b0010010100010000;
	U2BRG = 103;                  //скорость по счетчику
	IFS1bits.U2RXIF = 0;
	IFS1bits.U2TXIF = 0;
	IEC1bits.U2RXIE = 1;
	IEC1bits.U2TXIE = 0;
}

void
ADCset(void)
{
	AD1PCFG = 0b0001111111001100; 	// AN1 as analog, all other pins are digital
	AD1CON1 = 0x0000; 				// SAMP bit = 0 ends sampling
									// and starts converting
	AD1CHS = 0x0001; 				// Connect AN1 as CH0 input
									// in this example AN1 is the input
	AD1CSSL = 0;
	AD1CON3 = 0x0002; 				// Manual Sample, Tad = 2 Tcy
	AD1CON2 = 0b0010000000000000;
	AD1CON1bits.ADON = 1; 			// turn ADC ON
}

int main (void)
{
	ODCB = 0b0010111000000000;
	TRISA = 0b0000000000011111;
	TRISB = 0b0101000111101111;
	CNPU1 = 0b0000000000000001;
	CNPU2 = 0b0000000111000000;
	AD1PCFG = 0b0001111111111111;
	GSM_ON = 1;

	RTS = 1;

	T1set();
	T2set();
	T3set();
	U1set_2400();
	U2set();
	ADCset();

	Delay100ms(10);

	DataEEInit();
    dataEEFlags.val = 0;

	if(DataEERead(0x00) == 0xA0B3)
	{
		read_param();
		read_nbrs();
		read_outport();
	}
	else
	{
		write_param();
		write_nbrs();
		write_outport();
		DataEEWrite(0xA0B3, 0x00);
	}

	gsm_on();

send_cmds_a("ATE1\r",4,"OK");
//send_cmds_a("at+ipr=2400\r",4,"OK");
//U1close();
//U1set_2400();
//send_cmds_a("AT&W\r",4,"OK");
//for(;;)
//ClrWdt();
//	send_cmds_a("ATS0=0\r",10,"OK");
//	send_cmds_a("AT&W\r",10,"OK");

	for(;;)
	{
		ClrWdt();
		ans_t(1);
		if(IMEI_ok == 0)
		{
			if(get_IMEI() == 1)
				IMEI_ok = 1;
		}

		if(IN1)
		{
		 	if(!(status_d & 1))
			{
				status_d = status_d | 1;
				if(Settings & 1)
					out_m_f = 1;
				if((Settings & 2) && (SendSMS & 1))
				{
					sprintf(out_sms, "Vhod 1 otktrit, modul \"%s\"\n\r%2.2u:%2.2u", memo, Hour, Minute);
					if(nbr1[0] != 0)
					{
						strcpy(out_sms_nbr, nbr1);
						send_sms();
					}
					if(nbr2[0] != 0)
					{
						strcpy(out_sms_nbr, nbr2);
						send_sms();
					}
					if(nbr3[0] != 0)
					{
						strcpy(out_sms_nbr, nbr3);
						send_sms();
					}
				}
			}
		}
		else
		{
			if(status_d & 1)
			{
				status_d = status_d & 0xFE;
				if(Settings & 1)
					out_m_f = 1;
				if((Settings & 2) && (SendSMS & 2))
				{
					sprintf(out_sms, "Vhod 1 zakrit, modul \"%s\"\n\r%2.2u:%2.2u", memo, Hour, Minute);
					if(nbr1[0] != 0)
					{
						strcpy(out_sms_nbr, nbr1);
						send_sms();
					}
					if(nbr2[0] != 0)
					{
						strcpy(out_sms_nbr, nbr2);
						send_sms();
					}
					if(nbr3[0] != 0)
					{
						strcpy(out_sms_nbr, nbr3);
						send_sms();
					}
				}
			}
		}

		if(IN2)
		{
			if(!(status_d & 2))
			{
				status_d = status_d | 2;
				if(Settings & 1)
					out_m_f = 1;
				if((Settings & 2) && (SendSMS & 4))
				{
					sprintf(out_sms, "Vhod 2 otktrit, modul \"%s\"\n\r%2.2u:%2.2u", memo, Hour, Minute);
					if(nbr1[0] != 0)
					{
						strcpy(out_sms_nbr, nbr1);
						send_sms();
					}
					if(nbr2[0] != 0)
					{
						strcpy(out_sms_nbr, nbr2);
						send_sms();
					}
					if(nbr3[0] != 0)
					{
						strcpy(out_sms_nbr, nbr3);
						send_sms();
					}
				}
			}
		}
		else
		{
			if(status_d & 2)
			{
				status_d = status_d & 0xFD;
				if(Settings & 1)
					out_m_f = 1;
				if((Settings & 2) && (SendSMS & 8))
				{
					sprintf(out_sms, "Vhod 2 zakrit, modul \"%s\"\n\r%2.2u:%2.2u", memo, Hour, Minute);
					if(nbr1[0] != 0)
					{
						strcpy(out_sms_nbr, nbr1);
						send_sms();
					}
					if(nbr2[0] != 0)
					{
						strcpy(out_sms_nbr, nbr2);
						send_sms();
					}
					if(nbr3[0] != 0)
					{
						strcpy(out_sms_nbr, nbr3);
						send_sms();
					}
				}
			}
		}

		if(IN3)
		{
			if(!(status_d & 4))
			{
				status_d = status_d | 4;
				if(Settings & 1)
					out_m_f = 1;
				if((Settings & 2) && (SendSMS & 16))
				{
					sprintf(out_sms, "Vhod 3 otktrit, modul \"%s\"\n\r%2.2u:%2.2u", memo, Hour, Minute);
					if(nbr1[0] != 0)
					{
						strcpy(out_sms_nbr, nbr1);
						send_sms();
					}
					if(nbr2[0] != 0)
					{
						strcpy(out_sms_nbr, nbr2);
						send_sms();
					}
					if(nbr3[0] != 0)
					{
						strcpy(out_sms_nbr, nbr3);
						send_sms();
					}
				}
			}
		}
		else
		{
			if(status_d & 4)
			{
				status_d = status_d & 0xFB;
				if(Settings & 1)
					out_m_f = 1;
				if((Settings & 2) && (SendSMS & 32))
				{
					sprintf(out_sms, "Vhod 3 zakrit, modul \"%s\"\n\r%2.2u:%2.2u", memo, Hour, Minute);
					if(nbr1[0] != 0)
					{
						strcpy(out_sms_nbr, nbr1);
						send_sms();
					}
					if(nbr2[0] != 0)
					{
						strcpy(out_sms_nbr, nbr2);
						send_sms();
					}
					if(nbr3[0] != 0)
					{
						strcpy(out_sms_nbr, nbr3);
						send_sms();
					}
				}
			}
		}

		if(IN4)
		{
			if(!(status_d & 8))
			{
				status_d = status_d | 8;
				if(Settings & 1)
					out_m_f = 1;
				if((Settings & 2) && (SendSMS & 64))
				{
					sprintf(out_sms, "Vhod 4 otktrit, modul \"%s\"\n\r%2.2u:%2.2u", memo, Hour, Minute);
					if(nbr1[0] != 0)
					{
						strcpy(out_sms_nbr, nbr1);
						send_sms();
					}
					if(nbr2[0] != 0)
					{
						strcpy(out_sms_nbr, nbr2);
						send_sms();
					}
					if(nbr3[0] != 0)
					{
						strcpy(out_sms_nbr, nbr3);
						send_sms();
					}
				}
			}
		}
		else
		{
			if(status_d & 8)
			{
				status_d = status_d & 0xF7;
				if(Settings & 1)
					out_m_f = 1;
				if((Settings & 2) && (SendSMS & 128))
				{
					sprintf(out_sms, "Vhod 4 zakrit, modul \"%s\"\n\r%2.2u:%2.2u", memo, Hour, Minute);
					if(nbr1[0] != 0)
					{
						strcpy(out_sms_nbr, nbr1);
						send_sms();
					}
					if(nbr2[0] != 0)
					{
						strcpy(out_sms_nbr, nbr2);
						send_sms();
					}
					if(nbr3[0] != 0)
					{
						strcpy(out_sms_nbr, nbr3);
						send_sms();
					}
				}
			}
		}

		get_adc();
		if(Tm > Templ)
		{
			if(!(status_d & 16))
			{
				status_d = status_d | 16;
				if(Settings & 1)
					out_m_f = 1;
				if((Settings & 2) && (SendSMS & 256))
				{
					sprintf(out_sms, "Temperatura %dC, modul \"%s\"\n\r%2.2u:%2.2u", Tm, memo, Hour, Minute);
					if(nbr1[0] != 0)
					{
						strcpy(out_sms_nbr, nbr1);
						send_sms();
					}
					if(nbr2[0] != 0)
					{
						strcpy(out_sms_nbr, nbr2);
						send_sms();
					}
					if(nbr3[0] != 0)
					{
						strcpy(out_sms_nbr, nbr3);
						send_sms();
					}
				}
			}
		}
		else
			status_d = status_d & 0xEF;

		if(in_rng == 1)
		{
			send_cmds_a("ATH0\r",10,"OK");
			in_rng = 0;
		}

		if(mon_f == 1)
		{
			get_cash();
			mon_f = 0;
		}

		if(alrm_f && time_sync_ok)
		{
/*
sprintf(tmp_buf, "Per%u IP%u.%u.%u.%u APN:%s APN_u:%s APN_p:%s pss:%s nbr1:%s nbr2:%s nbr3:%s st:%u m_nbr:%s\r", Period, ExIP[0], ExIP[1], ExIP[2], ExIP[3], APN, APN_user, APN_pass, pass, nbr1, nbr2, nbr3, Settings, m_nbr);
send_cmds(tmp_buf);
send_cmds_a("AT\r",10,"OK");

sprintf(tmp_buf, "memo:%s\r", memo);
send_cmds(tmp_buf);
send_cmds_a("AT\r",10,"OK");

sprintf(tmp_buf, "SMS:%u Tl:%u\r", SendSMS, Templ);
send_cmds(tmp_buf);
send_cmds_a("AT\r",10,"OK");

tmpp1 = tmp_buf;
*tmpp1 = 0;
for(cnt1 = 0; cnt1 < 10; cnt1++)
{
	if(alrm1_day[cnt1] != 0)
	{
		tmpp1 = tmpp1 + sprintf(tmpp1, "%u/%u:%u-%u %u,", alrm1_day[cnt1], alrm1_hour[cnt1], alrm1_min[cnt1], alrm1_do[cnt1], (alrm_done[cnt1]&1));
	}
}
tmpp1 = tmpp1 + sprintf(tmpp1, "\r");
send_cmds(tmp_buf);
send_cmds_a("AT\r",10,"OK");

tmpp1 = tmp_buf;
*tmpp1 = 0;
for(cnt1 = 0; cnt1 < 10; cnt1++)
{
	if(alrm2_hour[cnt1] != 0xFF)
	{
		tmpp1 = tmpp1 + sprintf(tmpp1, "%u:%u-%u %u,", alrm2_hour[cnt1], alrm2_min[cnt1], alrm2_do[cnt1], (alrm_done[cnt1]&2));
	}
}
tmpp1 = tmpp1 + sprintf(tmpp1, "\r");
send_cmds(tmp_buf);
send_cmds_a("AT\r",10,"OK");

tmpp1 = tmp_buf;
*tmpp1 = 0;
for(cnt1 = 0; cnt1 < 16; cnt1++)
{
	if(nbrs[cnt1] != 0)
	{
		tmpp1 = tmpp1 + sprintf(tmpp1, "%u,", nbrs[cnt1]);
	}
}
tmpp1 = tmpp1 + sprintf(tmpp1, "\r");
send_cmds(tmp_buf);
send_cmds_a("AT\r",10,"OK");
*/
			gsm_on();
			get_time_gsm();
			if(day_done != Day)
			{
				day_done = Day;
				for(mcnt = 0; mcnt < 10; mcnt++)
					alrm_done[mcnt] = 0;
				DataEEWrite(day_done, 0x59);
			}
			alrm_f = 0;

			for(mcnt = 0; mcnt < 10; mcnt++)
			{
				if((alrm1_day[mcnt] == Day) && ((alrm_done[mcnt] & 1) == 0))
				{
					if(((alrm1_hour[mcnt] * 60) + alrm1_min[mcnt]) <= ((Hour * 60) + Minute))
					{
						mrc = 1;
						alrm_done[mcnt] = alrm_done[mcnt] | 1;
						for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
							DataEEWrite(alrm_done[cnt1] + (alrm_done[cnt1 + 1] << 8), cnt1/2 + 0x45);
					}
				}
			}
			for(mcnt = 0; mcnt < 10; mcnt++)
			{
				if((alrm_done[mcnt] & 2) == 0)
				{
					if(((alrm2_hour[mcnt] * 60) + alrm2_min[mcnt]) <= ((Hour * 60) + Minute))
					{
						mrc = 1;
						alrm_done[mcnt] = alrm_done[mcnt] | 2;
						for(cnt1 = 0; cnt1 < 10; cnt1 = cnt1 + 2)
							DataEEWrite(alrm_done[cnt1] + (alrm_done[cnt1 + 1] << 8), cnt1/2 + 0x45);
					}
				}
			}
		}

		if(getdata == 1)
		{
/*
send_cmds("getdata\r");
send_cmds_a("AT\r",10,"OK");
sprintf(tmp_buf, "out_sms_nbr:%s\r", out_sms_nbr);
send_cmds(tmp_buf);
send_cmds_a("AT\r",10,"OK");
*/
			out_sms_pos = out_sms;
			*out_sms_pos = 0;
			out_sms_pos = out_sms_pos + sprintf(out_sms_pos, "Number-P,E+\r\n");
			for(cnt2 = 0; cnt2 < 16; cnt2++)
			{
				if(nbrs[cnt2] != 0)
				{
/*
sprintf(tmp_buf, "read%u\r", nbrs[cnt2]);
send_cmds(tmp_buf);
send_cmds_a("AT\r",10,"OK");
*/
					out_sms_pos = out_sms_pos + sprintf(out_sms_pos, "%u-", nbrs[cnt2]);
					out_gprsp = 0;
					for(mcnt = 0; mcnt < 5; mcnt++)
					{
						if(read_u(nbrs[cnt2], 1))
							break;
					}
					Delay100ms(20);
					if(mcnt != 5)
					{
						tmp_buf[0] = out_gprs[6];
						tmp_buf[1] = out_gprs[7];
						tmp_buf[2] = 0;
						exp1 = atoi(&tmp_buf[0]);
						if(exp1 > 40)
							exp1 = exp1 - 40;
						tmp_buf[0] = out_gprs[4];
						tmp_buf[1] = out_gprs[5];
						tmp_buf[2] = out_gprs[2];
						tmp_buf[3] = out_gprs[3];
						tmp_buf[4] = out_gprs[0];
						tmp_buf[5] = out_gprs[1];
						tmp_buf[6] = 0;
						P = atol(&tmp_buf[0]);
						P = (P / 100000);
						mn = 1;
						for(mcnt = 0; mcnt < exp1; mcnt++)
							mn = mn * 10;
						P = P * mn;
						out_sms_pos = out_sms_pos + sprintf(out_sms_pos, "%.5f,", P);
					}
					else
						out_sms_pos = out_sms_pos + sprintf(out_sms_pos, "Error,");

					out_gprsp = 0;
					for(mcnt = 0; mcnt < 5; mcnt++)
					{
						if(read_u(nbrs[cnt2], 0))
							break;
					}
					if(mcnt != 5)
					{
						tmp_buf[0] = out_gprs[6];
						tmp_buf[1] = out_gprs[7];
						tmp_buf[2] = out_gprs[4];
						tmp_buf[3] = out_gprs[5];
						tmp_buf[4] = out_gprs[2];
						tmp_buf[5] = out_gprs[3];
						tmp_buf[6] = out_gprs[0];
						tmp_buf[7] = out_gprs[1];
						tmp_buf[8] = 0;
						Ea = atol(&tmp_buf[0]);
						Ea = Ea / 100;
						out_sms_pos = out_sms_pos + sprintf(out_sms_pos, "%.2f\r\n", Ea);
					}
					else
						out_sms_pos = out_sms_pos + sprintf(out_sms_pos, "Error\r\n");
					Delay100ms(5);
				}
			}
			getdata = 0;
/*
sprintf(tmp_buf, "out_sms:%s\r", out_sms);
send_cmds(tmp_buf);
send_cmds_a("AT\r",10,"OK");
*/
			send_sms();
		}

		if(mrc == 1)
		{
			for(cnt2 = 0; cnt2 < 16; cnt2++)
			{
				if(nbrs[cnt2] != 0)
				{
					out_gprsp = 0;
					P = -1;
					for(mcnt = 0; mcnt < 5; mcnt++)
					{
						if(read_u(nbrs[cnt2], 1))
							break;
					}
					Delay100ms(20);
					if(mcnt != 5)
					{
						tmp_buf[0] = out_gprs[6];
						tmp_buf[1] = out_gprs[7];
						tmp_buf[2] = 0;
						exp1 = atoi(&tmp_buf[0]);
						if(exp1 > 40)
							exp1 = exp1 - 40;
						tmp_buf[0] = out_gprs[4];
						tmp_buf[1] = out_gprs[5];
						tmp_buf[2] = out_gprs[2];
						tmp_buf[3] = out_gprs[3];
						tmp_buf[4] = out_gprs[0];
						tmp_buf[5] = out_gprs[1];
						tmp_buf[6] = 0;
						P = atol(&tmp_buf[0]);
						P = (P / 100000);
						mn = 1;
						for(mcnt = 0; mcnt < exp1; mcnt++)
							mn = mn * 10;
						P = P * mn;
					}

					out_gprsp = 0;
					Ea = -1;
					Er = -1;
					for(mcnt = 0; mcnt < 5; mcnt++)
					{
						if(read_u(nbrs[cnt2], 0))
							break;
					}
					if(mcnt != 5)
					{
						tmp_buf[0] = out_gprs[6];
						tmp_buf[1] = out_gprs[7];
						tmp_buf[2] = out_gprs[4];
						tmp_buf[3] = out_gprs[5];
						tmp_buf[4] = out_gprs[2];
						tmp_buf[5] = out_gprs[3];
						tmp_buf[6] = out_gprs[0];
						tmp_buf[7] = out_gprs[1];
						tmp_buf[8] = 0;
						Ea = atol(&tmp_buf[0]);
						Ea = Ea / 100;
						tmp_buf[0] = out_gprs[14];
						tmp_buf[1] = out_gprs[15];
						tmp_buf[2] = out_gprs[12];
						tmp_buf[3] = out_gprs[13];
						tmp_buf[4] = out_gprs[10];
						tmp_buf[5] = out_gprs[11];
						tmp_buf[6] = out_gprs[8];
						tmp_buf[7] = out_gprs[9];
						tmp_buf[8] = 0;
						Er = atol(&tmp_buf[0]);
						Er = Er / 100;
						sprintf(out_gprs, "POST http://www.elvin.com.ua/gsm/insert2.php?a=%s&b=%u&c=%.5f&d=%.2f&e=%.2f HTTP/1.0\r\n\r\n", IMEI, nbrs[cnt2], P, Ea, Er);
					}
					Delay100ms(5);
					cmd_cnt1 = 3;
					do{
						for(cmd_cnt = 0; cmd_cnt < 3; cmd_cnt++)
						{
							gsm_on();
							ret_f = send_gprs();
							if(ret_f == 1)
								break;
							if(ret_f == 10)
								Delay100ms(100);
						}
						if(cmd_cnt == 3)
						{
							gsm_off_t();
							gsm_on();
						}
						cmd_cnt1--;
						if(cmd_cnt1 == 0)
						{
							break;
						}
					}while(cmd_cnt == 3);
					Delay100ms(5);
				}
			}
			mrc = 0;
		}

		if(out_m_f == 1)
		{
			get_temp();
			if(get_t_inet() == 1)
			{
				out_m_f = 0;
				time_sync_ok = 1;
			}
		}

		if(in_sms == 1)
		{
			in_sms = 0;
			read_SMS_t();
		}
	}
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T1Interrupt(void)
{
/* Interrupt Service Routine code goes here */
	IFS0bits.T1IF = 0; //Reset Timer1 interrupt flag and Return from ISR
	if(Period != 0)
	{
		p_cnts++;
		if(p_cnts == 60)
		{
			p_cnts = 0;
			p_cntm++;
			if(p_cntm >= Period)
			{
				mrc = 1;
				p_cntm = 0;
			}
		}
	}
	if(alrm_cnt > 30)//проверка будильников 30сек
	{
		alrm_f = 1;
		alrm_cnt = 0;
	}
	else
		alrm_cnt++;

	if(mon_cnt > 3600)//деньги мин
	{
		mon_f = 1;
		mon_cnt = 0;
	}
	else
		mon_cnt++;

	if(out_m_cnt > 1800)//отправка кол-ва денег 30мин
	{
		out_m_f = 1;
		out_m_cnt = 0;
	}
	else
		out_m_cnt++;

}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T2Interrupt(void)
{
/* Interrupt Service Routine code goes here */
	IFS0bits.T2IF = 0; //Reset Timer1 interrupt flag and Return from ISR
	if(cnt100ms != 0)
		cnt100ms--;
	if(cnt100ms2 != 0)
		cnt100ms2--;
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T3Interrupt(void)
{
/* Interrupt Service Routine code goes here */
	IFS0bits.T3IF = 0; //Reset Timer1 interrupt flag and Return from ISR
	tmr3_f = 1;
}

void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt(void)
{
	IFS0bits.U1RXIF = 0;
	TMR3 = 0;
	in_byte = U1RXREG;
	if(in_byte > 0x1F)
		*ibp++ = in_byte;
	else
	{
		if(in_byte == 0x0D)
		{
			if(ibp != &ib[0])
			{
				RTS = 1;
				*ibp++ = 0x00;
				ibs = 1;
				ibp = &ib[0];
			}
		}
	}
}

void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt(void)
{
	IFS1bits.U2RXIF = 0;
	if(u2cnt < 1025)
	{
		cb[u2cnt] = U2RXREG & 0x7F;
		u2cnt++;
	}
}
