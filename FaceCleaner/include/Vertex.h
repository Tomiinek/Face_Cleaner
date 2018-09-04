#ifndef FACECLEANER_VERTEX_H
#define FACECLEANER_VERTEX_H

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
 * @brief   Object representing a vertex of a mesh.
 *
 * @file    Vertex.h
 * @author  Tomas Nekvinda <tom(at)neqindi.cz>
 */

#include <utility>
#include <algorithm>
#include <vector>

#include "Vector.h"
#include "Corner.h"
#include "Curvature.h"

class Vertex
{
public:
	Vertex(size_t idx, Vector3d coord) : _idx(idx), _coord(coord) {}

	/**
	 * @brief Vertex normal setter.
	 * 
	 * @param x Normal's x coordinate.
	 * @param y Normal's y coordinate.
	 * @param z Normal's z coordinate.
	 */
	void set_normal(double x, double y, double z) { set_normal(Vector3d(x, y, z)); }
	
	/**
	 * @brief Vertex normal setter.
	 * 
	 * @param normal The new normal.
	 */
	void set_normal(Vector3d normal) { _normal = normal; }

	/**
	 * @brief Vertex normal getter.
	 * 
	 * @return Vector3d The vertex normal.
	 */
	Vector3d normal() const { return _normal; }

	/**
	 * @brief Vertex location setter.
	 * 
	 * @param new_pos New location of the vertex.
	 */
	void set_pos(Vector3d new_pos) { _coord = std::move(new_pos); }

	/**
	 * @brief Vertex location getter.
	 * 
	 * @return Vector3d 
	 */
	Vector3d pos() const { return _coord; }

	/**
	 * @brief Getter of the x coordinate of the vertex's location.
	 * 
	 * @return double The vertex x coordinate.
	 */
	double x() const { return _coord.x; }

	/**
	 * @brief Getter of the y coordinate of the vertex's location.
	 * 
	 * @return double The vertex y coordinate.
	 */
	double y() const { return _coord.y; }

	/**
	 * @brief Getter of the z coordinate of the vertex's location.
	 * 
	 * @return double The vertex z coordinate.
	 */
	double z() const { return _coord.z; }

	/**
	 * @brief Getter of the raw curvature of the vertex.
	 * 
	 * @return Reference to the Curvature object holding info about the raw curvature of the vertex.
	 */
	Curvature& curv() { return _curvature; }
	
	/**
	 * @brief Getter of the vertex smooth curvature of the vertex.
	 * 
	 * @return Reference to the Curvature object holding info about the smooth curvature of vertex.
	 */
	Curvature& curv_smooth() { return _curvature_smooth; }

	/**
	 * @brief Index getter.
	 * 
	 * @return Index of the vertex in the containing Scene object. 
	 */
	size_t idx() const { return _idx; }

	/**
	 * @brief Index setter. Just a setter, use with caution!
	 * 
	 * @param i New index of the vertex.
	 */
	void set_idx(size_t i) { _idx = i; }

	/**
	 * @brief Add a neighbor to the list of vertex neighbors.
	 * 
	 * @param n[in]	Pointer to the vertex.
	 * @return 		@c true if the vertex was present in the list of neighbors, @c false otherwise.  
	 */
	bool add_neighbor(Vertex* n) {
		if (std::find(_neighbors.begin(), _neighbors.end(), n) != _neighbors.end()) return false;
		_neighbors.push_back(n);
		return true;
	}

	/**
	 * @brief Remove a neighbor from the list of vertex neighbors.
	 * 
	 * @param n[in] Pointer to the vertex to be removed.
	 */
	void remove_neighbor(const Vertex* n) {
		_neighbors.erase(std::remove(_neighbors.begin(), _neighbors.end(), n), _neighbors.end());
	}

	/**
	 * @brief Get the one ring object
	 * 
	 * @return const std::vector<Vertex*>& 
	 */
	const std::vector<Vertex*>& get_one_ring() const { return _neighbors; }

	/**
	 * @brief Get boundary neighbors of the vertex.
	 * 
	 * @param next[out] 	Pointer which will be filled to point to the next boundary vertex.
	 * @param previous[out]	Pointer which will be filled to point to the previous boundary vertex. 
	 * @return 				@c false if the vertex has no boundary neighbors, @c true otherwise. 
	 */
	bool get_boundary_neighbors(Vertex*& next, Vertex*& previous) const;

	/**
	 * @brief 
	 * 
	 * @param c 
	 */
	void add_corner(Corner* c) {
		if (std::find(_corners.begin(), _corners.end(), c) != _corners.end()) return;
		_corners.push_back(c);
	}

	/**
	 * @brief 
	 * 
	 * @param c 
	 */
	void remove_corner(const Corner* c) {
		_corners.erase(std::remove(_corners.begin(), _corners.end(), c), _corners.end());
	}

	/**
	 * @brief Get the corners object
	 * 
	 * @return const std::vector<Corner*>& 
	 */
	const std::vector<Corner*>& get_corners() const { return _corners; }

	/**
	 * @brief Get the corners copy object
	 * 
	 * @return std::vector<Corner*> 
	 */
	std::vector<Corner*> get_corners_copy() const { return _corners; }

	/**
	 * @brief Prepare the vertex normal by calculating an approximation of the normal.
	 * 
	 */
	void compute_normal();

	/**
	 * @brief Calculate the Voronoi are of the vertex.
	 * 
	 * @return Size of the vertex Voronoi area.
	 */
	double get_voroni_area() const {
		double area = 0;
		for (auto&& corner : _corners) area += corner->get_area();
		return area;
	}

private:
	// index of the vertex in Scene's vector of vertices
	size_t _idx;

	// vertex location
	Vector3d _coord {};
	// vertex normal
	Vector3d _normal {};
	// vertex raw curvature
	Curvature _curvature {};
	// vertex smooth curvature
	Curvature _curvature_smooth {};

	// list corners which are adjacent to the vertex
	std::vector<Corner*> _corners {};
	// list of neighbors of the vertex
	std::vector<Vertex*> _neighbors {};
};


#endif //FACECLEANER_VERTEX_H
