// This file is part of MatrixPilot.
//
//    http://code.google.com/p/gentlenav/
//
// Copyright 2009-2011 MatrixPilot Team
// See the AUTHORS.TXT file for a list of authors of MatrixPilot.
//
// MatrixPilot is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MatrixPilot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MatrixPilot.  If not, see <http://www.gnu.org/licenses/>.


#ifndef HEARTBEAT_H
#define HEARTBEAT_H


// number of heartbeats per second
#define HEARTBEAT_HZ 50

// Read-only value increments with each heartbeat
extern uint16_t udb_heartbeat_counter;


inline void heartbeat(void);
inline uint16_t heartbeat_cnt(void);
inline boolean heartbeat_chk(uint16_t hertz);


#endif // HEARTBEAT_H
