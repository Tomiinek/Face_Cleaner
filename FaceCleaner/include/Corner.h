#ifndef FACECLEANER_CORNER_H
#define FACECLEANER_CORNER_H

/*****************************************************************************\
**                                                       _ __   ___ __ _     **
**    Face Cleaner                                      | '_ \ / __/ _` |    **
**    Created by:                                       | | | | (_| (_| |    **
**    	Tomas Nekvinda, tom(at)neqindi.cz               |_| |_|\___\__, |    **
**      Copyright (c) anno domini 2018                                |_|    **
**                                                                           **
**    This program is free software: you can redistribute it and/or modify   **
**    it under the terms of the GNU General Public License as published by   **
**    the  Free Software Foundation;  either version 3 of the License,  or   **
**    any later version.	                                                 **
**                                                                           **
**    This program is distributed in the hope that it will be useful, but    **
**    WITHOUT ANY WARRANTY; without even the implied warranty of             **
**    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the General   **
**    Public License (http://www.gnu.org/licenses/) for more details.        **
**                                                                           **
\*****************************************************************************/

/**
 * @brief   Object representing a corner of a triangular mesh.
 *
 * @file    Corner.h
 * @author  Tomas Nekvinda <tom(at)neqindi.cz>
 */

#include <cstddef>
#include <vector>
#include <algorithm>
#include <array>

class Vertex;
class Face;

struct Corner
{
	/**
	 * @brief Construct a new Corner object
	 * 
	 * @param id 			  Index of the corner into the face array of corners, must be in range 0-2.
	 * @param vertex[in, out] Pointer to the corresponding vertex of the new corner. 	
	 * @param face[in, out]   Pointer to the containing triangular face.		
	 * @param opposites 	  List of opposite corners. Can be obtained by Face::compute_opposites(...).
	 */
	Corner(size_t id, Vertex *vertex, Face *face, std::vector<Corner*> opposites);

	/**
	 * @brief 
	 * 
	 * @param c 
	 */
	void remove_opposite(const Corner* c){ o.erase(std::remove(o.begin(), o.end(), c), o.end()); }

	/**
	 * @brief Add a corner to the current corner's list of opposite cornres.
	 * 
	 * @param[in] new_opposite The corner to be added into the list of opposite corners.
	 */
	void add_opposite(Corner* new_opposite) { o.push_back(new_opposite); }

	/**
	 * @brief Get apposite corners.
	 * 
	 * @return List of opposite corners.
	 */
	const std::vector<Corner*>& get_opposite() const { return o; }

	/**
	 * @brief Get the previous corner (which are stored CW or counter-CW).
	 * 
	 * @return Previous corner. 
	 */
	const Corner& get_previous() const;

	/**
	 * @brief Get the next corner (which are stored CW or counter-CW).
	 * 
	 * @return Next corner.
	 */
	const Corner& get_next() const;

	/**
	 * @brief Calculates area of the corner (Voronoi).
	 * 
	 * @return Area of the corner.
	 */
	double get_area() const;

	// index of the corner into the face array of corners, in range 0-2
	size_t idx {};
	// pointer to the corner's vertex
	Vertex* v {};
	// pointer to the containing triangular face
	Face* f {};
	// list of opposite cornres
	std::vector<Corner*> o {};
};


#endif //FACECLEANER_CORNER_H
