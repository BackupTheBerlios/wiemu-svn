/**
    node.hh
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

#ifndef NODE_HH
#define NODE_HH

class Mote;

class Node{
private:
	int node_id;
	int x, y, z;
	std::string flash;
	Mote *mote;
public:
	Node(std::string);
	~Node();
	void setID(int);
	int getID();
	void setLocation(int, int, int);
	void setX(int);
	void setY(int);
	void setZ(int);
	int getX();
	int getY();
	int getZ();
	Mote *getMote();
};

#endif