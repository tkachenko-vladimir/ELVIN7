void 
uart_putch2(char byte) 
{
	char par;

	byte &= 0x7f;
	par = byte;
	par ^= par >> 4;
	par ^= par >> 2;
	par ^= par >> 1;
	if ((par & 1) == 1)
		byte |= 0x80;
	/* output one byte */
	U2TXREG = byte;
	while(!U2STAbits.TRMT);
}

void 
uart_puts2(const char *str1) 
{
	while(*str1)
		uart_putch2(*str1++);
}
