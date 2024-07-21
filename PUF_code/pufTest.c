#include "xil_io.h"


int main(){
	u32 key;
	print("Hello\n\r");
	key = Xil_In32(0x43C00000);
	xil_printf("Key:%0x",key);
}
