/**
    node.cc
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

#include "include/debugger.hh"
#include "mote/mica2/mica2.hh"
#include "include/mote"
#include "include/node"

#include <iostream>

Node::Node()
{
	mote = new Mica2();
}

Node::~Node()
{
	delete mote;
}

void
Node::setFlash(std::string flash)
{
	this->flash = flash;
	mote->load(flash);
}

void
Node::setID(int node_id)
{
	this->node_id = node_id;
}

int
Node::getID()
{
	return node_id;
}

void
Node::setLocation(int x, int y, int z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

void
Node::setX(int x)
{
	this->x = x;
}

void
Node::setY(int y)
{
	this->y = y;
}

void
Node::setZ(int z)
{
	this->z = z;
}

int
Node::getX()
{
	return x;
}

int
Node::getY()
{
	return y;
}

int
Node::getZ()
{
	return z;
}

void
Node::setDebugger(Debugger *debug)
{
	mote->setDebugger(debug);
}

Mote*
Node::getMote()
{
	return mote;
}

