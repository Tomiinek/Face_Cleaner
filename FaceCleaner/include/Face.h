#ifndef FACECLEANER_FACE_H
#define FACECLEANER_FACE_H

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
 * @brief   Object representing a triangular face of a triangular mesh.
 *
 * @file    Face.h
 * @author  Tomas Nekvinda <tom(at)neqindi.cz>
 */

#include <vector>
#include <array>
#include <algorithm>

#include "Vertex.h"
#include "Corner.h"

class Face
{
public:
	Face(size_t idx, Vertex* v1, Vertex* v2, Vertex* v3)
			: _idx(idx), _corners { Corner(0, v1, this, compute_opposites(*v2, *v3, 0)),
		                            Corner(1, v2, this, compute_opposites(*v1, *v3, 1)),
		                            Corner(2, v3, this, compute_opposites(*v1, *v2, 2)) } {}
	/**
	 * @brief Index getter.
	 * 
	 * @return Index of the face in the containing Scene object. 
	 */
	size_t idx() const { return _idx; }
	
	/**
	 * @brief Index setter. Just a setter, use with caution!
	 * 
	 * @param i New index of the face.
	 */
	void set_idx(size_t i) { _idx = i; }

	/**
	 * @brief Set the texture coordinates of the face.
	 * 
	 * @param t1 First index of the texture coordinate (see Scene object)
	 * @param t2 Second index of the texture coordinate (see Scene object) 
	 * @param t3 Third index of the texture coordinate (see Scene object)
	 */
	void set_texture_coordinates(size_t t1, size_t t2, size_t t3){ 
		_txt_coords[0] = t1;
		_txt_coords[1] = t2;
		_txt_coords[2] = t3;
	}

	/**
	 * @brief Get the texture coordinates of the face
	 * 
	 * @return A reference to an array containing corresponding corner indices. 
	 */
	const std::array<size_t, 3>& get_texture_coordinates() const { return _txt_coords; }

	/**
	 * @brief Get the texture coordinate of the corresponding corner.
	 * 
	 * @param  idx  Index of the face corner. Must be in the range 0-2.
	 * @return 		Index of the texture coordinate (see Scene object).
	 */
	size_t get_texture_coordinate(size_t idx) { return _txt_coords[idx]; }

	/**
	 * @brief Get all three vertices of the face.
	 * 
	 * @param a First vertex pointer.
	 * @param b Second vertex pointer.
	 * @param c Third vertex pointer.
	 */
	void get_vertices(Vertex*& a, Vertex*& b, Vertex*& c) const {
		a = _corners[0].v;
		b = _corners[1].v;
		c = _corners[2].v;
	}
	/**
	 * @brief Get the Corner object corresponding to the index.
	 * 
	 * @param idx Index of the corner. Must be in the range 0-2.
	 * @return 	  Reference to the corner of the corner with index @a idx .
	 */
	Corner& get_corner(size_t idx) { return _corners[idx]; }

	/**
	 * @brief Get the previous corner (which are stored CW or counter-CW).
	 * 
	 * @param[in] curr Pointer to the reference corner of the face. 
	 * @return 		   Reference to the previous corner of @a curr . 
	 */
	const Corner& get_previous(const Corner* curr) const {
		auto c = curr->idx;
		return _corners[(c == 0) ? 2 : c - 1];
	}

	/**
	 * @brief Get the next corner (which are stored CW or counter-CW).
	 * 
	 * @param[in] curr Pointer to the reference corner of the face. 
	 * @return 		   Reference to the next corner of @a curr . 
	 */
	const Corner& get_next(const Corner* curr) const {
		auto c = curr->idx;
		return _corners[(c == 2) ? 0 : c + 1];
	}

	/**
	 * @brief Get the opposite corners of the corner.
	 * 
	 * @param curr Index of the corner. Must be in the range 0-2.
	 * @return	   List of opposite corners of the corner with index @a curr .
	 */
	const std::vector<Corner*>& get_opposite(size_t curr) const {
		return _corners[curr].get_opposite();
	}

private:

	/**
	 * @brief Precomputes oppsosite corners for the corenr.
	 * 
	 * @param v1 	 First vertex of the corner's opposite edge.
	 * @param v2 	 Second vertex of the corner's opposite edge.
	 * @param curr_c Index of the corner whose opposite corners are going to be prepared.
	 * @return 		 List of opposite corners.
	 */
	std::vector<Corner*> compute_opposites(const Vertex& v1, const Vertex& v2, size_t curr_c) const {
		// get corners of the opposite edge
		auto nv = v1.get_corners();
		auto pv = v2.get_corners();
		std::vector<Corner*> cb;
		// find mathicng corners and then get the index of the third corner
		for (std::vector<Corner*>::const_iterator it = nv.begin(); it != nv.end(); it++) {
			size_t idx1 = (*it)->idx;
			auto match = std::find_if(pv.begin(), pv.end(), [&it] (Corner* item) {
				return item->f == (*it)->f;
			});
			if (match != pv.end()){
				if ((*it)->f == this) continue;
				size_t idx2 = (*match)->idx;
				cb.push_back(&(*it)->f->get_corner(3 - idx1 - idx2));
			}
		}
		return cb;
	}

	// index of the face in the Scene's vector of faces
	size_t _idx;
	// array of corners of the face
	std::array<Corner, 3> _corners;
	// array of texture coordinate indices of the face
	std::array<size_t, 3> _txt_coords;
};


#endif //FACECLEANER_FACE_H
