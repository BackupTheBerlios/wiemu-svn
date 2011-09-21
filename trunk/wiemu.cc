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
#include <cstdlib>
#include <unistd.h>
#include "include/node.hh"
#include "include/thread.hh"

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

	Node *node = new Node(flash);
	node->setID(1);
	node->setLocation(10, 10, 0);
	Thread *thr = new Thread(node);
	steps > 0 ? thr->setSteps(steps) : thr->setSteps(NO_STEPS);
	thr->start();
	delete thr;
	delete node;
	
	return EXIT_SUCCESS;
}
