#ifndef ONEWIRE_H_
#define ONEWIRE_H_
#include <stdint.h>

#define OWPORTDIR P1DIR
#define OWPORTOUT P1OUT
#define OWPORTIN P1IN
#define OWPORTREN P1REN
#define OWPORTPIN BIT0
#define OW_LO {	OWPORTDIR |= OWPORTPIN;	OWPORTREN &= ~OWPORTPIN; OWPORTOUT &= ~OWPORTPIN; }
#define OW_HI {	OWPORTDIR |= OWPORTPIN;	OWPORTREN &= ~OWPORTPIN; OWPORTOUT |= OWPORTPIN; }
#define OW_RLS { OWPORTDIR &= ~OWPORTPIN; OWPORTREN |= OWPORTPIN; OWPORTOUT |= OWPORTPIN; }


#define DS1820_SKIP_ROM             0xCC
#define DS1820_READ_SCRATCHPAD      0xBE
#define DS1820_CONVERT_T            0x44

//########################################################################

uint8 onewire_reset();
void onewire_write_bit(uint8 bit);
uint8 onewire_read_bit();
void onewire_write_byte(uint8 byte);
uint8 onewire_read_byte();
void onewire_line_low();
void onewire_line_high();
void onewire_line_release();
void StartDS18b20(void);
void GetData(void);
#endif /* ONEWIRE_H_ */