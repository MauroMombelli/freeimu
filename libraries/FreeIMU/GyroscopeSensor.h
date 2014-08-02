/*
This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

***************************************

This code have been heavily rewritten by 
Mauro Mombelli <mombelli dot mauro at gmail dot com> 
in a stormy night between 2014-08-01 and 2014-08-02

*/

#ifndef GyroscopeSensor_h
#define GyroscopeSensor_h

#include <inttypes.h>

class GyroscopeSensor
{
	public:
		virtual uint8_t getRadiant(float*, float*, float*);
		virtual void update(void);
};

#endif