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
* @file    HoleFiller.cpp
*/

#include "HoleFiller.h"

std::tuple<size_t, size_t> HoleFiller::fill(Scene & s) {

	// at first, we clean holes as described here:
	// https://pdfs.semanticscholar.org/f71e/e8d075b3561a77b04b11115f42c1201353b1.pdf
	size_t removed = 0;
	std::unordered_set<Vertex*> to_remove;
	do {
		std::vector<boundary> holes = s.get_holes();
		if (holes.empty()) return std::make_tuple(0, 0);
		to_remove.clear();
		for (auto&& hole : holes) {
			for (auto&& v : hole) {
				if (v->get_corners().size() != 1) continue;
				to_remove.insert(v);
			}
		}
		removed += to_remove.size();
		for (auto&& v : to_remove) s.remove_vertex(*v);
	}
	while (!to_remove.empty());

	// implamentation of topological filling by Peter Liepa described here:
	// https://drive.google.com/file/d/0Bz89KcgZMI0RUlhnc3NkZWV0TnM/view
	size_t added = 0;
	std::vector<boundary> holes = s.get_holes();

	// fill every hole
	for (auto&& hole : holes) {
		// get initial span
		auto triangles = triangulation(s, hole);
		// add textures according to the span
		textures_t textures;
		if (s.has_textures()) {
			for (auto&& t : triangles) {
				textures.push_back({
					t[0]->get_corners()[0]->f->get_texture_coordinate(t[0]->get_corners()[0]->idx),
					t[1]->get_corners()[0]->f->get_texture_coordinate(t[1]->get_corners()[0]->idx),
					t[2]->get_corners()[0]->f->get_texture_coordinate(t[2]->get_corners()[0]->idx)
				});
			}
		}
		// add new vertices to the span to make it similar as the hole surroundings
		auto new_vertices = refinement(s, triangles, textures, hole);
		added += new_vertices.size();
		for (size_t i = 0; i < triangles.size(); ++i) {
			size_t h = s.add_face(triangles[i][0], triangles[i][1], triangles[i][2]);
			if (s.has_textures()) s.set_face_texture_coordinate(h, textures[i][0], textures[i][1], textures[i][2]);
		}
		// fair the shape of the hole to make it better looking
		bool fr = fairing(s, new_vertices);
		if (!fr) LOG(WRN) << LOG::pad_right("Failed to optimize a hole shape. You'd better check it!");
	}

	return std::make_tuple(added, removed);
}

HoleFiller::triangles_t HoleFiller::triangulation(const Scene& s, boundary hole) {

	size_t n = hole.size();
	std::vector<std::vector<TriangulationWeight>> w;
	std::vector<std::vector<int>> t;

	// set defaults and initials
	for (size_t i = 0; i < n - 1; i++) {
		w.push_back(std::vector<TriangulationWeight>(n - i - 1));
		t.push_back(std::vector<int>(n - i - 1, -1));
	}

	// dynamic programm ...
	for (size_t j = 2; j < n; j++) {
		for (size_t i = 0; i < n - j; i++) {
			int k = (int) (i + j);
			int m = -1;
			TriangulationWeight min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
			for (int l = (int) i + 1; l < k; l++) {
				auto val = w[i][l-i-1] + w[l][k-l-1] + get_weight(s, (int) i, l, k, hole, t);
				if (val < min) {
					min = val;
					m = l;
				}
			}
			w[i][k-i-1] = min;
			t[i][k-i-1] = m;
		}
	}
	// recursively reconstruct new and optimal triangulation of the hole
	return triangles_from_triangulation(0, (int) n - 1, hole, t);
}

std::vector<Vertex*> HoleFiller::refinement(Scene & s, triangles_t& triangles, textures_t& textures, const boundary& hole) {

	std::vector<Vertex*> new_vertices;
	// sqrt(2) - controls density of resulting refinement
	constexpr double alpha = 1.41421356237;
	size_t nt = triangles.size();
	std::unordered_map<Vertex*, double> scales;
	edges_t edges;

	// prepare scales of all triangles, this tracks density of the triangulation
	for (size_t i = 0; i < hole.size(); ++i) {
		auto& neighbors = hole[i]->get_one_ring();
		auto v = hole[i]->pos();
		double sum = 0;
		for (auto&& n : neighbors) sum += (n->pos() - v).length();
		scales.insert({hole[i], sum / neighbors.size()});
	}

	// extract unordered map of edges from the triangulation, we need it because of edge swaps
	for (size_t i = 0; i < nt; ++i) {
		for (int a = 0; a < 3; ++a) {
			auto b = (a == 2 ? 0 : a + 1);
			auto c = (b == 2 ? 0 : b + 1);
			auto edge = make_edge(*triangles[i][a], *triangles[i][b]);
			edges[edge].push_back(triangles[i][c]);
		}
	}

	// iterate until no change
	while (true) {
		bool changed = false;

		// for each triangle of the triangulation
		for (size_t i = 0; i < nt; ++i) {

			// try to place new vertex into an existing triangle
			Vector3d centroid = (triangles[i][0]->pos() + triangles[i][1]->pos() + triangles[i][2]->pos()) / 3.0;
			double scale = (scales[triangles[i][0]] + scales[triangles[i][1]] + scales[triangles[i][2]]) / 3.0;
			bool verified = true;

			// is it ok to place there the new vertex?
			for (size_t j = 0; j < 3; ++j) {
				Vertex* m = triangles[i][j];
				auto d = alpha * (centroid - m->pos()).length();
				if (d > scale && d > scales[m]) continue;
				verified = false;
				break;
			}
			if (!verified) continue;

			// new vertex is valid, so let's add it to scene!
			size_t c_idx = s.add_vertex(centroid);
			Vertex& c = s.get_vertex(c_idx);

			// ... and modify our local and temporary structures
			scales.insert({&c, scale});
			new_vertices.push_back(&c);
			auto t1 = triangles[i][0];
			auto t2 = triangles[i][1];
			auto t3 = triangles[i][2];
			triangles.push_back({&c, t2, t3});
			triangles.push_back({t1, &c, t3});
			triangles.push_back({t1, t2, &c});
			triangles.erase(triangles.begin() + i);

			//
			// Future work:
			// THIS SHOULD BE MODIFIED IN ORDER TO MAKE BETTER TEXTURE HANDLING
			// HERE IS IMPLEMENTED JUST A PRIMITIVE INTERPOLATION
			//
			if (s.has_textures()) {			
				auto c_txt = (s.get_texture_coordinate(textures[i][0]).second +
							  s.get_texture_coordinate(textures[i][1]).second +
							  s.get_texture_coordinate(textures[i][2]).second) / 3.0;
				auto h = s.add_texture_coordinate(c_txt.x, c_txt.y);
				textures.push_back({h, textures[i][1], textures[i][2]});
				textures.push_back({textures[i][0], h, textures[i][2]});
				textures.push_back({textures[i][0], textures[i][1], h});
				textures.erase(textures.begin() + i);
			}

			auto& t1_t2 = edges[make_edge(*t1, *t2)];
			auto& t2_t3 = edges[make_edge(*t2, *t3)];
			auto& t1_t3 = edges[make_edge(*t1, *t3)];
			std::replace(t1_t2.begin(), t1_t2.end(), t3, &c);
			std::replace(t2_t3.begin(), t2_t3.end(), t1, &c);
			std::replace(t1_t3.begin(), t1_t3.end(), t2, &c);

			edges[make_edge(*t1, c)].insert(edges[make_edge(*t1, c)].end(), {t2, t3});
			edges[make_edge(*t2, c)].insert(edges[make_edge(*t2, c)].end(), {t1, t3});
			edges[make_edge(*t3, c)].insert(edges[make_edge(*t3, c)].end(), {t2, t1});

			// try to swap edges of the new triangle, swaps only if a criterion is met
			try_swap_edge(s, *t1, *t2, triangles, textures, edges);
			try_swap_edge(s, *t1, *t3, triangles, textures, edges);
			try_swap_edge(s, *t2, *t3, triangles, textures, edges);

			// restart for loop because edges swaps could have changed triangulation
			i = 0;
			nt = triangles.size();
			changed = true;
		}
		if (!changed) break;

		// try to swap every edge of the triangulation
		UniqueQueue<std::pair<size_t, size_t>> edge_queue;
		for (auto it = edges.begin(); it != edges.end(); ++it) edge_queue.enqueue(it->first);
		while (!edge_queue.empty()) {
			auto edge = edge_queue.dequeue();
			try_swap_edge(s, s.get_vertex(edge.first), s.get_vertex(edge.second), triangles, textures, edges);
		}
	}

	return new_vertices;
}

bool HoleFiller::fairing(Scene & s, const std::vector<Vertex*>& new_vertices) {

	//
	// uses eigen's conjugate gradient solver: https://eigen.tuxfamily.org/dox/classEigen_1_1ConjugateGradient.html

	if (new_vertices.empty()) return true;

	size_t n = new_vertices.size() * 3;
	Eigen::SparseMatrix<double> A(n, n);
	Eigen::VectorXd x(n), b(n);

	// prepare matrix of coefficients and the vector of solutions
	for (size_t i = 0; i < new_vertices.size(); ++i) {

		// diagonal
		A.insert(i * 3, i * 3) = 1.0;
		A.insert(i * 3 + 1, i * 3 + 1) = 1.0;
		A.insert(i * 3 + 2, i * 3 + 2) = 1.0;

		// here are sotred contrains given by boundary vertices
		double fixed[3];
		fixed[0] = 0; fixed[1] = 0; fixed[2] = 0;

		//
		// prepare matrix, we look at neighbours of neighbours ...

		auto neighbors = new_vertices[i]->get_one_ring();
		double total = 0.0;
		for (auto&& n : neighbors) total += omega_weighting(*new_vertices[i], *n);

		for (auto&& n : neighbors) {

			double weight = omega_weighting(*new_vertices[i], *n);
			double effect = -2.0 * weight / total;

			auto idx = std::distance(new_vertices.begin(), std::find(new_vertices.begin(), new_vertices.end(), n));
			if (idx == new_vertices.size()) {
				fixed[0] += effect * n->pos().x;
				fixed[1] += effect * n->pos().y;
				fixed[2] += effect * n->pos().z;
			}
			else {
				A.coeffRef(idx * 3, i * 3) += effect;
				A.coeffRef(idx * 3 + 1, i * 3 + 1) += effect;
				A.coeffRef(idx * 3 + 2, i * 3 + 2) += effect;
			}

			auto n_neighbors = n->get_one_ring();
			double n_total = 0.0;
			for (auto&& nn : n_neighbors) n_total += omega_weighting(*n, *nn);
			
			for (auto&& nn : n_neighbors) {
				double n_weight = omega_weighting(*n, *nn);
				double n_effect = weight / total * n_weight / n_total;
				auto n_idx = std::distance(new_vertices.begin(), std::find(new_vertices.begin(), new_vertices.end(), nn));
				if (n_idx == new_vertices.size()) {
					fixed[0] += n_effect * nn->pos().x;
					fixed[1] += n_effect * nn->pos().y;
					fixed[2] += n_effect * nn->pos().z;
				}
				else {
					A.coeffRef(n_idx * 3, i * 3) += n_effect;
					A.coeffRef(n_idx * 3 + 1, i * 3 + 1) += n_effect;
					A.coeffRef(n_idx * 3 + 2, i * 3 + 2) += n_effect;
				}
			}
		}

		b[i * 3]	 = -fixed[0];
		b[i * 3 + 1] = -fixed[1];
		b[i * 3 + 2] = -fixed[2];
		x[i * 3] = new_vertices[i]->pos().x;
		x[i * 3 + 1] = new_vertices[i]->pos().y;
		x[i * 3 + 2] = new_vertices[i]->pos().z;
	}

	// iteratively solve the defined se of linear equations
	Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower|Eigen::Upper> cg;
	cg.setMaxIterations(_max_iters);
	cg.setTolerance(1e-6);
	cg.compute(A);
	auto xz = cg.solveWithGuess(b, x);
	
	// change positions of triangulation vertices according to the solution
	for (size_t i = 0; i < new_vertices.size(); ++i) {
		new_vertices[i]->set_pos(Vector3d(xz[i * 3], xz[i * 3 + 1], xz[i * 3 + 2]));
	}

	return cg.iterations() != _max_iters;
}

HoleFiller::TriangulationWeight HoleFiller::get_weight(const Scene& s, int i, int j, int k, const boundary & hole, 
													   const std::vector<std::vector<int>>& triangulation) {

	auto v1 = hole[i]->pos();
	auto v2 = hole[j]->pos();
	auto v3 = hole[k]->pos();
	Vector3d v4;

	double angle;

	//
	// compute maximal dihedral angle

	if (i + 1 == j) {
		for (auto&& c : hole[j]->get_corners()) {
			if (c->get_next().v == hole[i]) {
				v4 = c->get_previous().v->pos();
				break;
			}
		}	
		angle = MathSupport::dihedral_angle(v1, v2, v3, v4);
	}
	else angle = MathSupport::dihedral_angle(v1, v2, v3, hole[triangulation[i][j-i-1]]->pos());
	if (j + 1 == k) {
		for (auto&& c : hole[k]->get_corners()) {
			if (c->get_next().v == hole[j]) {
				v4 = c->get_previous().v->pos();
				break;
			}
		}
		angle = std::max(angle, MathSupport::dihedral_angle(v2, v3, v1, v4));
	}
	else angle = std::max(angle, MathSupport::dihedral_angle(v2, v3, v1, hole[triangulation[j][k-j-1]]->pos()));
	if (i == 0 && k == hole.size() - 1) {
		for (auto&& c : hole[i]->get_corners()) {
			if (c->get_next().v == hole[k]) {
				v4 = c->get_previous().v->pos();
				break;
			}
		}
		angle = std::max(angle, MathSupport::dihedral_angle(v3, v1, v2, v4));
	}

	// add aspect ratio weighting
	double ratio = 1.0 - MathSupport::triangle_aspect_ratio(v1, v2, v3);
	return TriangulationWeight(angle, ratio);
}

HoleFiller::triangles_t HoleFiller::triangles_from_triangulation(int from, int to, const boundary & hole,
													 const std::vector<std::vector<int>>& triangulation) {
	triangles_t triangles;
	if (from == to || from + 1 == to) return triangles;

	int m = triangulation[from][to - from - 1];
	if (m == -1) return triangles;

	auto triangles1 = triangles_from_triangulation(from, m, hole, triangulation);
	auto triangles2 = triangles_from_triangulation(m, to, hole, triangulation);
	triangles1.insert(triangles1.end(), triangles2.begin(), triangles2.end());
	triangles1.push_back({hole[from], hole[m], hole[to]});

	return triangles1;
}

bool HoleFiller::try_swap_edge(Scene& s, Vertex & a, Vertex & b, triangles_t& triangles, textures_t& textures, edges_t& edges) {

	auto edge = make_edge(a, b);
	if (edges.find(edge) == edges.end()) return false;
	auto& opposites = edges[edge];
	
	// we cannt swap anything if there are not two triangles
	if (opposites.size() != 2) return false;

	// does the edge exists? whould never fail
	Vertex& c = *opposites[0];
	Vertex& d = *opposites[1];
	if (edges.find(make_edge(c, d)) != edges.end()) return false;

	// the only criterion for edge swapping
	if (!MathSupport::is_point_in_triangles_circumsphere(a.pos(), b.pos(), c.pos(), d.pos()) &&
		!MathSupport::is_point_in_triangles_circumsphere(a.pos(), b.pos(), d.pos(), c.pos())) return false;

	//
	// swap the edge and update all needed structures

	edges[make_edge(c, d)] = {&a, &b};
	edges.erase(edge);
	auto& ca = edges[make_edge(c, a)];
	auto& cb = edges[make_edge(c, b)];
	auto& da = edges[make_edge(d, a)];
	auto& db = edges[make_edge(d, b)];
	std::replace(ca.begin(), ca.end(), &b, &d);
	std::replace(cb.begin(), cb.end(), &a, &d);
	std::replace(da.begin(), da.end(), &b, &c);
	std::replace(db.begin(), db.end(), &a, &c);

	int t[2];
	for (size_t i = 0; i < triangles.size(); ++i) {
		if (!(triangles[i][0] == &a || triangles[i][1] == &a || triangles[i][2] == &a) || 
			!(triangles[i][0] == &b || triangles[i][1] == &b || triangles[i][2] == &b)) continue;
		if (triangles[i][0] == &c || triangles[i][1] == &c || triangles[i][2] == &c) t[0] = (int) i;
		else if (triangles[i][0] == &d || triangles[i][1] == &d || triangles[i][2] == &d) t[1] = (int) i;
	}

	size_t ia0 = std::distance(triangles[t[0]].begin(), std::find(triangles[t[0]].begin(), triangles[t[0]].end(), &a));
	size_t ia1 = std::distance(triangles[t[1]].begin(), std::find(triangles[t[1]].begin(), triangles[t[1]].end(), &a));
	size_t ib0 = std::distance(triangles[t[0]].begin(), std::find(triangles[t[0]].begin(), triangles[t[0]].end(), &b));
	size_t ib1 = std::distance(triangles[t[1]].begin(), std::find(triangles[t[1]].begin(), triangles[t[1]].end(), &b));

	auto ra0 = (std::max(ia0, ib0) == 1 ? 1 : (std::min(ia0, ib0) == 1 ? 2 : 0));
	auto ra1 = (std::max(ia1, ib1) == 1 ? 0 : (std::min(ia1, ib1) == 1 ? 1 : 2));
	std::array<Vertex*, 3> newt0 = {triangles[t[0]][ra0], triangles[t[0]][(ra0 + 1) % 3], triangles[t[1]][(ra1 + 2) % 3]};
	std::array<Vertex*, 3> newt1 = {triangles[t[0]][(ra0 + 2) % 3], triangles[t[1]][(ra1 + 2) % 3], triangles[t[0]][(ra0 + 1) % 3]};
	triangles[t[0]] = std::move(newt0);
	triangles[t[1]] = std::move(newt1);

	if (s.has_textures()) {
		std::array<size_t, 3> newtxt0 = {textures[t[0]][ra0], textures[t[0]][(ra0 + 1) % 3], textures[t[1]][(ra1 + 2) % 3]};
		std::array<size_t, 3> newtxt1 = {textures[t[0]][(ra0 + 2) % 3], textures[t[1]][(ra1 + 2) % 3], textures[t[0]][(ra0 + 1) % 3]};
		textures[t[0]] = std::move(newtxt0);
		textures[t[1]] = std::move(newtxt1);
	}

	return true;
}
