#ifndef FACECLEANER_HOLEFILLER_H
#define FACECLEANER_HOLEFILLER_H

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
 * @brief   Wrapper around functions and settings which can fill mesh holes.
 *
 * @file    HoleFiller.h
 * @author  Tomas Nekvinda <tom(at)neqindi.cz>
 */

#include <vector>
#include <unordered_set>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include "boost/functional/hash.hpp"
#include <Eigen/Eigen>

#include "Scene.h"
#include "Logger.h"

class HoleFiller
{
public:
	HoleFiller(size_t max_iters) : _max_iters(max_iters) {}

	/**
	 * @brief Fill all mesh holes, except the largest one --> creates disk topology.
	 * 
	 * @param s   Scene to be filled.
	 * @return 	  Number of added and removed vertices, respectively. 
	 */
	std::tuple<size_t, size_t> fill(Scene& s);

private:

	using textures_t = std::vector<std::array<size_t, 3>>;
	using triangles_t = std::vector<std::array<Vertex*, 3>>;
	using edges_t = std::unordered_map<std::pair<size_t, size_t>, std::vector<Vertex*>, boost::hash< std::pair<size_t, size_t>>>;

	/**
	 * @brief Creates ordered pair which can be used for edge indexing.
	 * 
	 * @param v1 First edge vertex reference.
	 * @param v2 Second edge vertex reference.
	 * @return   Edge key. 
	 */
	std::pair<size_t, size_t> make_edge(const Vertex& v1, const Vertex& v2) {
		if (v1.idx() < v2.idx()) return std::make_pair(v1.idx(), v2.idx());
		else return std::make_pair(v2.idx(), v1.idx());
	}

	/**
	 * @brief Creates ordered pair which can be used for edge indexing.
	 * 
	 * @param v1 First edge vertex reference.
	 * @param v2 Second edge vertex index.
	 * @return   Edge key. 
	 */
	std::pair<size_t, size_t> make_edge(const Vertex& v1,size_t v2) {
		if (v1.idx() < v2) return std::make_pair(v1.idx(), v2);
		else return std::make_pair(v2, v1.idx());
	}

	/**
	 * @brief Weighting of initial hole triangulations.
	 * 
	 */
	class TriangulationWeight
	{
	public:
		TriangulationWeight() : _angle(0), _ratio(0) {}
		TriangulationWeight(double angle, double ratio, bool raw = false) {
			_ratio = ratio;
			// this function causes insensitivity for angles which are roughly pi
			// and very strong penalisation for angles less than PI / 2 
			if (raw) _angle = angle;
			else _angle = std::round(std::pow(angle / 2.0, 4));
		}

		const TriangulationWeight operator+(const TriangulationWeight &r) const {
			return TriangulationWeight(std::max(_angle, r._angle), _ratio + r._ratio, true);
		}

		TriangulationWeight& operator+=(const TriangulationWeight &r) {
			_ratio += r._ratio;
			_angle = std::max(_angle, r._angle);
			return *this;
		}

		friend bool operator> (const TriangulationWeight& lhs, const TriangulationWeight& rhs) { return rhs < lhs; }
		friend bool operator<=(const TriangulationWeight& lhs, const TriangulationWeight& rhs) { return !(lhs > rhs); }
		friend bool operator>=(const TriangulationWeight& lhs, const TriangulationWeight& rhs) { return !(lhs < rhs); }
		friend bool operator< (const TriangulationWeight& lhs, const TriangulationWeight& rhs) {
			return lhs._angle < rhs._angle || (lhs._angle == rhs._angle && lhs._ratio < rhs._ratio);
		}

	private:
		double _angle;
		double _ratio;
	};

	template<typename T>
	class UniqueQueue
	{
	public:
		bool empty() const { return _set.empty(); }
		bool contains(const T& item) const { return _set.find(item) != _set.end(); }
		void enqueue(const T& item) { 
			auto insertion = _set.insert(item);
			if (insertion.second) _queue.push(item);
		}
		T dequeue() {
			auto f = _queue.front();
			_queue.pop();
			_set.erase(f);
			return f;
		}

	private:
		std::unordered_set<T, boost::hash<T>> _set{};
		std::queue<T> _queue{};
	};

	/**
	 * @brief Creates the intial span of a hole.
	 * 
	 * @param s 	Scene in which the hole is.
	 * @param hole  Hole to be filled.
	 * @return      Resulting hole triangulation (list of triangles).
	 */
	triangles_t triangulation(const Scene& s, boundary hole);

	/**
	 * @brief Add new vertices and faces to fit the span density to hole surroundings.
	 * 
	 * @param s 		Scene in which the hole is.
	 * @param triangles Initial hole span returned by triangulation method.
	 * @param textures  Correspondign textures of triangles in @a triangles .
	 * @param hole 		Hole which initial span is going to be refined.
	 * @return 			List of new vertices.
	 */
	std::vector<Vertex*> refinement(Scene& s, triangles_t& triangles, textures_t& textures, const boundary& hole);
	
	/**
	 * @brief Refine the shape of a hole spanning.
	 * 
	 * @param s 		    Scene in which the hole is.
	 * @param new_vertices  List of vertices to be refined.
	 * @return 				@c true if the refinement ended in the maximal number of steps.
	 */
	bool fairing(Scene& s, const std::vector<Vertex*>& new_vertices);

	/**
	 * @brief Calculates the weight of adding (i,j,k) triangle into the mesh. Used to find optimal initial span.
	 * 
	 * @param scene 		Scene in which the hole is.
	 * @param i 			Index of the first vertex w. r. to @ hole list.
	 * @param j 			Index of the second  vertex w. r. to @ hole list.
	 * @param k 			Index of the third vertex w. r. to @ hole list.
	 * @param hole 			The hole.
	 * @param triangulation The actual hole tringulation (indices into @a hole ).
	 * @return  			Weight of the triangle.
	 */
	TriangulationWeight get_weight(const Scene& scene, int i, int j, int k, const boundary& hole, 
										  const std::vector<std::vector<int>>& triangulation);
	
	/**
	 * @brief Extract list of triangles from the initial hole span.
	 * 
	 * @param from 				Starting index of the triangulation to be extracted (usually 0).
	 * @param to 				Ending index of the triangulation to be extracted (usually hole size -1).
	 * @param hole 				The hole.
	 * @param triangulation 	Triangulation of @a hole .
	 * @return 					List of extracted triangles.
	 */
	triangles_t triangles_from_triangulation(int from, int to, const boundary& hole,
															const std::vector<std::vector<int>>& triangulation);
	
	/**
	 * @brief Swap edge if the adjacent triangles are ill-shaped.
	 * 
	 * @param s 		Containing scene.
	 * @param v1 		First edge vertex.
	 * @param v2 		Second edge vertex.
	 * @param triangles Triangulation of a hole.
	 * @param textures  Texture coordinates corresponding to @a trinagles .
	 * @param edges 	List of edges of the hole triangulation.
	 * @return 			@c true if the edge was swapped, @c false otherwise.
	 */
	bool try_swap_edge(Scene& s, Vertex& v1, Vertex& v2, triangles_t& triangles, textures_t& textures, edges_t& edges);
	
	/**
	 * @brief Get reciprocal of the distance between two vertices.
	 * 
	 * @param v1 	First vertex.
	 * @param v2 	Second vertex.
	 * @return 		Reciprocal of @a v1 and @a v2 distance.
	 */
	double omega_weighting(const Vertex& v1, const Vertex& v2) { return 1.0 / (v1.pos() - v2.pos()).length(); }

	// the maximum number of steps which can be used to fair a hole shape
	size_t _max_iters;
};


#endif //FACECLEANER_HOLEFILLER_H
