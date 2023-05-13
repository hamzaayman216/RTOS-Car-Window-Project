#define setBit(Reg, bit) (Reg |= (1 << bit))
#define clearBit( Reg,  bit) (Reg &= ~(1 << bit))
#define toggleBit( Reg, bit) ( Reg ^= 1<< bit)
#define getBit(Reg , bit) ( (Reg &= (1 << bit)) >> bit)
