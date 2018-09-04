
#ifndef FACECLEANER_SCENE_H
#define FACECLEANER_SCENE_H

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
 * @brief   Object representing a triangular mesh and grouping mesh manipulation methods.
 *
 * @file    Scene.h
 * @author  Tomas Nekvinda <tom(at)neqindi.cz>
 */

#include <iostream>
#include <vector>
#include <utility>
#include <memory>
#include <unordered_set>
#include <stack>

#include "omp.h"
#include "MathSupport.h"
#include "Vector.h"
#include "Face.h"
#include "Vertex.h"
#include "Corner.h"
#include "Curvature.h"

using boundary = std::vector<Vertex*>;

class Scene
{
public:
	/**
	 * @brief Add a vertex to the scene.
	 * 
	 * @param x X cordinate of the vertex location.
	 * @param y Y cordinate of the vertex location.
	 * @param z Z cordinate of the vertex location.
	 * @return 	Index of the added vertex. 
	 */
	size_t add_vertex(double x, double y, double z) { 
		return add_vertex(Vector3d(x, y, z)); 
	}

	/**
	 * @brief Add a vertex to the scene.
	 * 
	 * @param position Location of the vertex.
	 * @return size_t  Index of the added vertex.
	 */
	size_t add_vertex(Vector3d position) {
		size_t idx = _vertices.size();
		_vertices.push_back(std::make_unique<Vertex>(idx, position));
		return idx;
	}

	/**
	 * @brief Remove a vertex from the scene.
	 * 
	 * @param vertex Reference to the vertex to be removed.
	 */
	void remove_vertex(Vertex& vertex){
		auto corners = vertex.get_corners_copy();
		for (auto&& corner : corners) remove_face(*(corner->f));
		auto idx = vertex.idx();
		_vertices[idx] = std::move(_vertices.back());
		_vertices[idx]->set_idx(idx);
		_vertices.pop_back();
	}

	/**
	 * @brief Get the vertex associated with an index.
	 * 
	 * @param idx 	Index of a vertex.
	 * @return		Reference to the vertex with the index @a idx .
	 */
	Vertex& get_vertex(size_t idx) const { return *_vertices[idx].get(); }

	/**
	 * @brief Get all vertices of the mesh.
	 * 
	 * @return Reference to the list of all vertices.
	 */
	const std::vector<std::unique_ptr<Vertex>>& get_vertices() const { return _vertices; }

	/**
	 * @brief Add a face to the scene.
	 * 
	 * @param v1 Valid index of the first vertex.
	 * @param v2 Valid index of the second vertex.
	 * @param v3 Valid index of the third vertex.
	 * @return 	 Index of the added face. 
	 */
	size_t add_face(size_t v1, size_t v2, size_t v3) {
		return add_face(_vertices[v1].get(), _vertices[v2].get(), _vertices[v3].get());
	}

	/**
	 * @brief Add a face to the scene.
	 * 
	 * @param v1[in] Pointer to the first vertex.
	 * @param v2[in] Pointer to the second vertex.
	 * @param v3[in] Pointer to the third vertex.
	 * @return 	 	 Index of the added face. 
	 */
	size_t add_face(Vertex* v1, Vertex* v2, Vertex* v3);

	/**
	 * @brief Remove a face from the the scene.
	 * 
	 * @param face Reference to the face to be removed.
	 */
	void remove_face(Face& face);

	/**
	 * @brief Get the face associated with an index.
	 * 
	 * @param idx 	Index of a face.
	 * @return		Reference to the face with the index @a idx .
	 */
	Face& get_face(size_t idx) const { return *_faces[idx].get(); }

	/**
	 * @brief Get all faces of the mesh.
	 * 
	 * @return Reference to the list of all faces.
	 */
	const std::vector<std::unique_ptr<Face>>& get_faces() const { return _faces; }

	/**
	 * @brief Add a texture coordinate. The coordinate can then be used in other faces.
	 * 
	 * @param x  X coordinate.
	 * @param y  Y coordinate.
	 * @return   Index of the added texture coordinate.
	 */
	size_t add_texture_coordinate(double x, double y) { 
		size_t idx = _txt_coords.size();
		_txt_coords.push_back(std::make_pair(0, Vector2d(x, y))); 
		return idx;
	}

	/**
	 * @brief Get the texture coordinate
	 * 
	 * @param  Index of a texture coordinate. 
	 * @return Reference to the list of all faces.
	 */
	const std::pair<size_t, Vector2d>& get_texture_coordinate(size_t idx) const { return _txt_coords[idx]; }
	
	/**
	 * @brief Set all three texture coordinates of a face.
	 * 
	 * @param f  Reference to the face to be changed.
	 * @param t1 Texture coordinate of the first corner.
	 * @param t2 Texture coordinate of the second corner.
	 * @param t3 Texture coordinate of the third corner.
	 */
	void set_face_texture_coordinate(Face& f, size_t t1, size_t t2, size_t t3) {
		++_txt_coords[t1].first; ++_txt_coords[t2].first; ++_txt_coords[t3].first;
		f.set_texture_coordinates(t1, t2, t3);
	}

	/**
	 * @brief Set the face texture coordinate object
	 * 
	 * @param f  Index of the face to be changed.
	 * @param t1 Texture coordinate of the first corner.
	 * @param t2 Texture coordinate of the second corner.
	 * @param t3 Texture coordinate of the third corner.
	 */
	void set_face_texture_coordinate(size_t f, size_t t1, size_t t2, size_t t3) {
		++_txt_coords[t1].first; ++_txt_coords[t2].first; ++_txt_coords[t3].first;
		_faces[f]->set_texture_coordinates(t1, t2, t3);
	}

	/**
	 * @brief Check whether the scene normals should be managed.
	 * 
	 * @return @c true if should be, @c false otherwise.
	 */
	bool has_normals() const { return _has_normals; }

	/**
	 * @brief Check whether the scene textures should be managed.
	 * 
	 * @return @c true if should be, @c false otherwise.
	 */
	bool has_textures() const { return _has_textures; }

	/**
	 * @brief Explicitly set that the scene contains normals and other manipulators should care about that.
	 * 
	 */
	void set_has_normals() { _has_normals = true; }

	/**
	 * @brief Explicitly set that the scene contains texture and other manipulators should care about that.
	 * 
	 */
	void set_has_textures() { _has_textures = true; }

	/**
	 * @brief Get the number of scene vertices.
	 * 
	 * @return Count of vertices.
	 */
	size_t vertices_num() const { return _vertices.size(); }

	/**
	 * @brief Get the number of scene faces.
	 * 
	 * @return Count of triangles.
	 */
	size_t triangles_num() const { return _faces.size(); }

	/**
	 * @brief Get the numer of scene texture coordinates.
	 * 
	 * @return Count of texture coordinates.
	 */
	size_t textures_num() const { return _txt_coords.size(); }

	/**
	 * @brief Get mean edge length of all scene edges.
	 * 
	 * @return Mean edge length.
	 */
	double mean_edge_length() const { return _total_edge_length / _edges_count; }

	/**
	 * @brief Get vertices of the given triangle (cc or ccc order).
	 * 
	 * @param t Index of the triangle.
	 * @param a Index of the first triangle vertex.
	 * @param b Index of the second triangle vertex.
	 * @param c Index of the third triangle vertex.
	 */
	void get_triangle_vertices(size_t t, size_t& a, size_t& b, size_t& c) const {
		Vertex* av,* bv,* cv;
		_faces[t]->get_vertices(av, bv, cv);
		a = av->idx();
		b = bv->idx();
		c = cv->idx();
	}

	/**
	 * @brief Get the triangle vertices object
	 * 
	 * @param triangle 
	 * @param v1 
	 * @param v2 
	 * @param v3 
	 */
	void get_triangle_vertices(const Face& triangle, Vertex*& v1, Vertex*& v2, Vertex*& v3) const {
		triangle.get_vertices(v1, v2, v3);
	}

	/**
	 * @brief Computes vertex normals of all vertices.
	 * 
	 */
	void prepare_normals();

	/**
	 * @brief Get mesh boudaries except the largest one. Expects proper 2-manifold.
	 * 
	 * @return List of boundaries which are cyclic lists of ordered vertices.
	 */
	std::vector<boundary> get_holes() const;

	/**
	 * @brief Get the largest boudnary of all mesh boudnaries. Expects proper 2-manifold.
	 * 
	 * @return Boundary which is a cyclic list of ordered vertices.
	 */
	boundary get_outer_boundary() const;

	/**
	 * @brief Collapse an edge into a single vertex.
	 * 
	 * @param v1 	   Reference to first edge vertex.
	 * @param v2 	   Reference to second edge vertex.
	 * @param added    List of pointers to added faces (new triangles).
	 * @param removed  List of pointers to removed faces.
	 * @return 		   Index of the new added vertex.
	 */
	size_t merge_edge(Vertex& v1, Vertex& v2, std::unordered_set<Face*>& added, std::unordered_set<Face*>& removed);

	/**
	 * @brief Clear all mesh data.
	 * 
	 */
	void clear() {
		_vertices.clear();
		_faces.clear();
		_txt_coords.clear();
		_total_edge_length = 0;
		_edges_count = 0;
		_has_normals = false;
		_has_textures = false;
	}

private:

	/**
	 * @brief Get all boundaries of the mesh - includes hole boundaries and the whole mesh boundary.
	 * 		  Expects proper 2-manifold.
	 * 
	 * @return List of boundaries, which are cyclic lists of ordered vetrices.
	 */
	std::vector<boundary> get_boundaries() const;

	// list of pointers to mesh vertices, vertices have backreference (idx into this list)
	std::vector<std::unique_ptr<Vertex>> _vertices {};
	// list of pointers to mesh faces, faces have backreference (idx into this list)
	std::vector<std::unique_ptr<Face>> _faces {};
	// list of all texture coordinates used by faces, it holds also the number of usages for each coordinate 
	std::vector<std::pair<size_t, Vector2d>> _txt_coords {};

	// were normals present in the original mesh file?
	bool _has_normals = false;
	// were textures present in the original mesh file?
	bool _has_textures = false;

	// these are used just to obtain the mean edge length
	// can be updated by removals and additions
	double _total_edge_length = 0;
	size_t _edges_count = 0;
};


#endif //FACECLEANER_SCENE_H
