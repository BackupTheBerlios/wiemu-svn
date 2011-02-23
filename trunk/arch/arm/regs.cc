#include "regs.hh"
#include <iostream>
#include <iomanip>

Regs::Regs(){
	for(int i=0 ; i<=15 ; i++)
		this->r[i] = 0;
	this->cpsr = 0;
}

Regs::~Regs(){

}

void Regs::dump(){
	for(int i=0 ; i<16 ; i++){
		std::cout << "R" << std::setfill('0') << std::setw(2) << std::dec << i;
		std::cout << "=" << std::setfill('0') << std::setw(8) << std::hex << this->r[i] << "   ";
		if(!((i+1)%4))
			std::cout << std::endl;
	}
	std::cout << "CPSR=" << std::hex << std::setfill('0') << std::setw(8) << this->cpsr << std::endl;
	std::cout << std::endl;
}

