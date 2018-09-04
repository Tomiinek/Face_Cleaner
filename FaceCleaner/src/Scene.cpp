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
 * @author  Tomas Nekvinda <tom(at)neqindi.cz>
 * @file    Scene.cpp
 */

#include "Scene.h"

size_t Scene::add_face(Vertex *v1, Vertex *v2, Vertex *v3) {

	size_t idx = _faces.size();
	_faces.push_back(std::make_unique<Face>(idx, v1, v2, v3));
	if (v1->add_neighbor(v2)) {
		v2->add_neighbor(v1);
		_total_edge_length += (v2->pos() - v1->pos()).length();
		++_edges_count;
	}
	if (v2->add_neighbor(v3)) {
		v3->add_neighbor(v2);
		_total_edge_length += (v3->pos() - v2->pos()).length();
		++_edges_count;
	}
	if (v3->add_neighbor(v1)) {
		v1->add_neighbor(v3);
		_total_edge_length += (v3->pos() - v1->pos()).length();
		++_edges_count;
	}
	return idx;
}

void Scene::remove_face(Face& face) {

	Vertex* v1,* v2,* v3;
	face.get_vertices(v1, v2, v3);

	auto& op0 = face.get_opposite(0);
	auto& op1 = face.get_opposite(1);
	auto& op2 = face.get_opposite(2);

	// are triangles on boundary? ... edges and neighbours must be updated
	if (op0.empty()){
		_total_edge_length -= (v3->pos() - v2->pos()).length();
		--_edges_count;
		v2->remove_neighbor(v3); v3->remove_neighbor(v2);
	}
	if (op1.empty()){
		_total_edge_length -= (v3->pos() - v1->pos()).length();
		--_edges_count;
		v1->remove_neighbor(v3); v3->remove_neighbor(v1);
	}
	if (op2.empty()){
		_total_edge_length -= (v2->pos() - v1->pos()).length();
		--_edges_count;
		v2->remove_neighbor(v1); v1->remove_neighbor(v2);
	}

	// remove corners of all vertices
	v1->remove_corner(&face.get_corner(0));
	v2->remove_corner(&face.get_corner(1));
	v3->remove_corner(&face.get_corner(2));

	// update opposite corners of neighbouring triangles
	for (auto&& o : op0) o->remove_opposite(&face.get_corner(0));
	for (auto&& o : op1) o->remove_opposite(&face.get_corner(1));
	for (auto&& o : op2) o->remove_opposite(&face.get_corner(2));

	// remove texture coordinates if are not needed and are present
	if (has_textures()) {
		auto& txt = face.get_texture_coordinates();
		for (int i = 0; i < 3; ++i) --_txt_coords[txt[i]].first;
	}

	// the face removal
	auto idx = face.idx();
	_faces[idx] = std::move(_faces.back());
	_faces[idx]->set_idx(idx);
	_faces.pop_back();
}

void Scene::prepare_normals() {

	// implementation of https://pdfs.semanticscholar.org/9146/5dff3c4beecc70bad31442925aa3abe35061.pdf
	// see trimesh2 library by Szymon Rusinkiewicz: http://gfx.cs.princeton.edu/proj/trimesh2/ 

	if (_vertices.empty() || _faces.empty()) return;

	std::vector<Vector3d> new_normals(_vertices.size());
	for (auto it = _faces.begin(); it < _faces.end(); ++it) {
		auto c1 = (*it)->get_corner(0);
		auto c2 = (*it)->get_corner(1);
		auto c3 = (*it)->get_corner(2);
		auto a = c1.v->pos() - c2.v->pos();
		auto b = c2.v->pos() - c3.v->pos();
		auto c = c3.v->pos() - c1.v->pos();
		double la = a.length_squared();
		double lb = b.length_squared();
		double lc = c.length_squared();
		if (la == 0.0 || lb == 0.0 || lc == 0.0) return;
		auto n = Vector3d::cross(a, b);
		new_normals[c1.v->idx()] += n / (la * lc);
		new_normals[c2.v->idx()] += n / (lb * la);
		new_normals[c3.v->idx()] += n / (lc * lb);
	}

	for (size_t i = 0; i < _vertices.size(); ++i) {
		_vertices[i]->set_normal(new_normals[i].normalized());
	}
}

std::vector<boundary> Scene::get_holes() const {

	// get all boudnaries
	auto boundaries = get_boundaries();
	std::vector<boundary> holes {};
	if (boundaries.empty()) return holes;

	// the longest boundary is probably face border
	auto max_idx = boundaries.begin();
	size_t max_size = 0;
	for (auto it = boundaries.begin(); it != boundaries.end(); ++it) {
		if (it->size() <= max_size) continue;
		max_idx = it;
		max_size = it->size();
	}
	boundaries.erase(max_idx);
	return boundaries;
}

boundary Scene::get_outer_boundary() const {

	// get all boundaries
	auto boundaries = get_boundaries();
	std::vector<boundary> holes {};
	if (boundaries.empty()) return boundary {};

	// the longest boundary is probably face border
	auto max_idx = boundaries.begin();
	size_t max_size = 0;
	for (auto it = boundaries.begin(); it != boundaries.end(); ++it) {
		if (it->size() <= max_size) continue;
		max_idx = it;
		max_size = it->size();
	}
	return std::move(*max_idx);
}

size_t Scene::merge_edge(Vertex& v1, Vertex& v2, std::unordered_set<Face*>& added, std::unordered_set<Face*>& removed) {

	// create the new vertex
	auto middle = (v1.pos() + v2.pos()) / 2.0;
	size_t handle = add_vertex(middle);

	// merge edges of the two vertices which are going to be removed
	auto corners = v1.get_corners_copy();
	auto corners2 = v2.get_corners();
	corners.insert(corners.end(), corners2.begin(), corners2.end());
	added.clear();

	for (auto&& c : corners) {
		// remove a face adjacent to one of the vertices to be removed
		removed.insert(c->f);
		auto prev = c->get_previous();
		auto next = c->get_next();
		// is the face adjacent to edge to be removed?
		if (prev.v == &v1 || next.v == &v1 || prev.v == &v2 || next.v == &v2) continue;
		// if not, add a new face
		size_t h = add_face(prev.v, &get_vertex(handle), next.v);
		if (has_textures()) {
			auto& txt = c->f->get_texture_coordinates();
			set_face_texture_coordinate(h, txt[prev.idx], txt[prev.idx], txt[next.idx]);
		}
		added.insert(&get_face(h));
	}
	// remove the original edge vertices
	remove_vertex(v1);
	remove_vertex(v2);
	return handle;
}

std::vector<boundary> Scene::get_boundaries() const {

	std::vector<boundary> boundaries;
	if (_vertices.empty()) return boundaries;

	std::unordered_set<Vertex*> visited;
	for (auto&& v : _vertices) {
		if (visited.find(v.get()) != visited.end()) continue;
		visited.insert(v.get());
		boundary b;
		Vertex* n,* p;
		// if the vertex is on a boundary
		if (v->get_boundary_neighbors(n, p)) {
			// add the vertex to boundary
			b.push_back(v.get());
			Vertex* next = n;
			// follow the boundary and add all vertices to list
			while (visited.find(next) == visited.end()) {
				b.push_back(next);
				visited.insert(next);
				next->get_boundary_neighbors(next, p);
			}
			// a boudnary was completely found
			boundaries.push_back(std::move(b));
		}
	}
	return boundaries;
}
