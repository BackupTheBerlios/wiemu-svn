/**
    wiemu.cc
    Copyright (C) 2011  Mohamed Aslan <maslan@maslan.info>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
**/

#include <iostream>
#include <ctime>
#include <cstdlib>
#include <unistd.h>
#include "mote/mica2/mica2.hh"

int
main(int argc, char **argv)
{
	std::cout << "WiEmu WSN Emulator" << std::endl;
	std::cout << "Copyrights (c) 2010-2011 Mohamed Aslan. All rights reserved." << std::endl;
#if defined(CXX) && defined(DATE)
	std::cout << "Compiled with: " << CXX <<  " on: " << DATE << std::endl;
#endif
	
	int opt;
	int64_t steps = -1;
	while((opt = getopt(argc, argv, "s:")) != -1){
		switch(opt){
			// Steps to emulate
			case 's':
				steps = atoi(optarg);
				break;
			// Error
			default:
				std::cerr << "Usage: " << argv[0] << " [-s steps] image" << std::endl;
				exit(EXIT_FAILURE);
		}
	}
	if(optind >= argc){
		std::cerr << "Expected argument after options" << std::endl;
		exit(EXIT_FAILURE);
	}
	std::string flash(argv[argc-1]);
           
	clock_t clk_start = clock();
	Mica2 mica2;
	mica2.load(flash);
	if(steps != -1){
		for(int i=1 ; i<=steps ; i++)
		mica2.step();
	}else{
		mica2.run();
	}
	clock_t clk_end = clock();
	
	std::cout << std::endl;
	std::cout << "Emulation Ended..." << std::endl;
	std::cout << "\t* Total CPU cycles = " << mica2.getCycles() << " cycles" << std::endl;
	std::cout << "\t* Total SIM time = " << (clk_end-clk_start)/(double)CLOCKS_PER_SEC << " sec" << std::endl;
	
	return EXIT_SUCCESS;
}
