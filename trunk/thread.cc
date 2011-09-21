/**
    thread.cc
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

#include <string>
#include <ctime>
#include "include/node.hh"
#include "include/thread.hh"

#include <iostream>

Thread::Thread(Node *node)
{
	this->node = node;
	steps = NO_STEPS;
}

Thread::~Thread()
{
	std::cout << std::endl;
	std::cout << "Emulation Ended..." << std::endl;
	std::cout << "\t* Total CPU cycles = " << node->getMote()->getCycles() << " cycles" << std::endl;
	std::cout << "\t* Total SIM time = " << (clk_end-clk_start)/(double)CLOCKS_PER_SEC << " sec" << std::endl;
}

void*
Thread::entry(void *param)
{
	Thread *thr = (Thread *)param;
	thr->run();
	return NULL;
}

void
Thread::setSteps(int steps)
{
	this->steps = steps;
}

void
Thread::start()
{
	ret = pthread_create(&tid, NULL, entry, (void *)this);
	pthread_join(tid, NULL);
}

void
Thread::run()
{
	clk_start = clock();
	if(steps == NO_STEPS){
		node->getMote()->run();
	}
	else{
		while(steps--)
			node->getMote()->step();
	}
	clk_end = clock();
}

