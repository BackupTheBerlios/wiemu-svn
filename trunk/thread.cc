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

std::vector<Thread *> Thread::threads;

Thread::Thread()
{
	steps = NO_STEPS;
	threads.push_back(this);
}

Thread::~Thread()
{
	std::cout << std::endl;
	std::cout << "Emulation Ended..." << std::endl;
	std::cout << "\t* Total CPU cycles = " << node->getMote()->getCycles() << " cycles" << std::endl;
	std::cout << "\t* Total SIM time = " << (clk_end-clk_start)/(double)CLOCKS_PER_SEC << " sec" << std::endl;
}

void
Thread::setNode(Node *node)
{
	this->node = node;
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
	for(unsigned int i=0 ; i<threads.size() ; i++){
		threads[i]->ret = pthread_create(&(threads[i]->tid), NULL, entry, (void *)(threads[i]));
	}
	for(unsigned int i=0 ; i<threads.size() ; i++){
		pthread_join(threads[i]->tid, NULL);
	}
}

void
Thread::run()
{
	clk_start = clock();
	if(steps == NO_STEPS){
		node->getMote()->run();
	}
	else{
		for(int i=0 ; i<steps ; i++)
			node->getMote()->step();
	}
	clk_end = clock();
}

