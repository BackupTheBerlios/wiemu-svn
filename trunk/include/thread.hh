/**
    thread.hh
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

#ifndef THREAD_HH
#define THREAD_HH

#define	NO_STEPS	0

#include <pthread.h>
#include "../mote/mica2/mica2.hh"

class Thread{
private:
	pthread_t tid;
	int ret;
	Node *node;
	int steps;
	clock_t clk_start, clk_end;
public:
	Thread(Node *);
	~Thread();
	static void* entry(void *);
	void setSteps(int);
	void start();
	void run();
};

#endif
