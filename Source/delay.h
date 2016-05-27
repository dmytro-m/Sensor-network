#ifndef DELAY_H_
#define DELAY_H_

#define CYCLES_PER_US 8L // depends on the CPU speed
#define DELAY_US(x) __delay_cycles((x * CYCLES_PER_US))

#endif /* DELAY_H_ */