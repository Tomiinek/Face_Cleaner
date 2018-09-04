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
* @file    Cleaner.cpp
*/

#include "Cleaner.h"

Cleaner::Cleaner(HoleFiller* hf, PointClassifier* pc, const boost::program_options::variables_map & vm) 
  : _hf(hf), _pc(pc), 
	_unit(vm["scale"].as<double>()),
	_remove_thin_triangles(vm["thin-triangles"].as<bool>()),
	_crop(vm["crop"].as<bool>()), 
	_crop_by_normal(vm["crop-by-normal"].as<bool>()), 
	_remove_defects(vm["remove-defects"].as<bool>()),
    _fill_holes(vm["fill-holes"].as<bool>()), 
	_rotate(vm["rotate"].as<bool>()),
	_landmarks(vm["save-landmarks"].as<bool>()),
	_curvedness_tolerance(vm["curvedness-tolerance"].as<double>()), 
	_far(vm["far"].as<double>()), 
	_near(vm["near"].as<double>()),
	_n2ch_over_n2r_mean(vm["n2ch-over-n2r-mean"].as<double>()),
	_n2ch_over_n2r_dev(vm["n2ch-over-n2r-dev"].as<double>()),
	_neck_normal_threshold(vm["neck-normal-threshold"].as<double>()),
	_ears_normal_threshold(vm["ears-normal-threshold"].as<double>()),
	_neck_angle_threshold(vm["neck-angle-threshold"].as<double>()),
	_ears_angle_threshold(vm["ears-angle-threshold"].as<double>()),
	_wide_protrusion_range(vm["wide-protrusion-range"].as<double>()),
	_wide_protrusion_angle(vm["wide-protrusion-angle"].as<double>()),
	_tiny_protrusion_angle(vm["tiny-protrusion-angle"].as<double>()),
	_maximal_angle(vm["maximal-angle"].as<double>()),
	_minimal_angle(vm["minimal-angle"].as<double>()),
	_max_mean_curv(vm["maximal-mean-curv"].as<double>()),
	_min_mean_curv(vm["minimal-mean-curv"].as<double>()),
	_max_gauss_curv(vm["maximal-gauss-curv"].as<double>()),
	_min_gauss_curv(vm["minimal-gauss-curv"].as<double>()),
	_max_mean_smooth(vm["maximal-mean-curv-smooth"].as<double>()),
	_min_mean_smooth(vm["minimal-mean-curv-smooth"].as<double>()),
	_max_gauss_smooth(vm["maximal-gauss-curv-smooth"].as<double>()),
	_min_gauss_smooth(vm["minimal-gauss-curv-smooth"].as<double>()),
	_min_triangle_aspect_ratio(vm["min-triangle"].as<double>()) {}

void Cleaner::process(Scene & s) {

	size_t removed = 0;
	size_t removed_new = 0;
	removed += remove_multiple_face_edges(s);
	do {
		size_t r1 = remove_non_manifold_vertices(s);
		size_t r2 = remove_minor_components(s);
		removed += r1 + r2;
		if (r1 + r2 == 0) break;
	}
	while (true);
	LOG(INF) << "Removed " << LOG::pad_number_left(removed, 5) << " vertices: Minor components and non-manifold vertices";

	if (!s.has_normals()) {
		LOG(INF) << "Prepearing vertex normals";
		s.prepare_normals();
	}

	if (_crop || _rotate || _landmarks || _remove_defects) {
		LOG(INF) << "Prepearing vertex curvatures";
		Curvature::prepare_curvatures(s, _unit);
		Curvature::smooth_curvatures(_smooth_curvature_sigma * _unit * _unit, s);
	}
	
	Vertex* left_eye, *right_eye, *left_lips, *right_lips, *nose, *root;
	left_eye = right_eye = left_lips = right_lips = nose = root = nullptr;
	auto detection_res = PointClassifier::NTHNG;
	if (_crop || _rotate || _landmarks) {
		LOG(INF) << "Searching for important face points";
		detection_res = _pc->update_face_landmarks(s, left_eye, right_eye, left_lips, right_lips, nose, root);
		switch (detection_res) {
			case PointClassifier::ALL:
				LOG(INF) << "All desired points were found";
				break;
			case PointClassifier::NOSE:
				LOG(WRN) << "Just nose was detected, cropping by normals and face rotation will be skipped";
				break;
			case PointClassifier::NTHNG:
				LOG(ERR) << "No desired face points were found, cropping and face rotation will be skipped";
		}
	}

	Vector3d sight_dir, vertical_dir, face_center;
	if (detection_res == PointClassifier::ALL) {
		auto nose_pos = nose->pos();
		auto eye_middle = (left_eye->pos() + right_eye->pos()) / 2.0;
		auto mouth_middle = (left_lips->pos() + right_lips->pos()) / 2.0;
		auto basic_dir = (mouth_middle - eye_middle).normalized();
		auto normal = Vector3d::cross(eye_middle - right_lips->pos(), left_lips->pos() - right_lips->pos()).normalized();
		face_center = (mouth_middle + eye_middle) / 2.0;
		sight_dir = (nose_pos - eye_middle - basic_dir * Vector3d::dot(nose_pos - eye_middle, basic_dir)).normalized();
		vertical_dir = (basic_dir + (nose_pos - eye_middle - normal * Vector3d::dot(normal, nose_pos - eye_middle)).normalized()) / 2.0;
		if (Vector3d::dot(sight_dir, nose_pos - face_center) < 0) sight_dir *= -1.0;
	}

	if (_crop && detection_res != PointClassifier::NTHNG) {
		LOG(INF) << "Cropping face";
		Vector3d root_pos = (detection_res == PointClassifier::ALL ? root->pos() : Vector3d(0, 0, 0));
		removed_new = crop_face(s, nose->pos(), root_pos, sight_dir, vertical_dir, face_center, detection_res == PointClassifier::ALL);
		LOG(INF) << "Removed " << LOG::pad_number_left(removed_new, 5) << " vertices: Face has been cropped";
		removed += removed_new;
	}

	if (_remove_defects) {
		LOG(INF) << "Removing weird vertices and intersecting triangles";
		removed_new = remove_defects(s);
		LOG(INF) << "Removed " << LOG::pad_number_left(removed_new, 5) << " vertices: Weird vertices and intersections";
		removed += removed_new;
	}

	removed_new = 0;
	do {
		size_t r1 = remove_non_manifold_vertices(s);
		size_t r2 = remove_minor_components(s);
		removed_new += r1 + r2;
		if (r1 + r2 == 0) break;
	}
	while (true);
	LOG(INF) << "Removed " << LOG::pad_number_left(removed_new, 5) << " vertices: Minor components and non-manifold vertices";
	removed += removed_new;

	if (_fill_holes) {
		int max_iter = 10;
		do {
			if (max_iter-- == 0) {
				LOG(ERR) << "Failed to fill some holes without intersecting triangles. You'd better check it!";
				break;
			}

			LOG(INF) << "Filling holes";
			removed_new = 0;
			size_t added = 0;
			std::tie(added, removed_new) = _hf->fill(s);
			removed += removed_new;		
			LOG(INF) << "Added" << LOG::pad_number_left(added, 5) << " vertices: Holes are filled";

			LOG(INF) << "Checking new triangles for intersections";
			std::unordered_set<Vertex*> intersections_to_remove;
			remove_intersecting_triangles(s, intersections_to_remove);
			for (auto&& v : intersections_to_remove) s.remove_vertex(*v);
			LOG(INF) << "Removed " << LOG::pad_number_left(intersections_to_remove.size(), 5) << " vertices: Intersecting triangles";
			if (intersections_to_remove.size() == 0) break;

			removed_new = 0;
			do {
				size_t r1 = remove_non_manifold_vertices(s);
				size_t r2 = remove_minor_components(s);
				removed_new += r1 + r2;
				if (r1 + r2 == 0) break;
			}
			while (true);
			LOG(INF) << "Removed " << LOG::pad_number_left(removed_new, 5) << " vertices: Minor components and non-manifold vertices";
			removed += removed_new;
		}
		while (true);
	}

	if (_remove_thin_triangles) {
		LOG(INF) << "Resolving thin triangles";
		std::deque<Face*> triangle_queue;
		for (auto&& t : s.get_faces()) triangle_queue.push_back(t.get());
		removed_new = remove_thin_triangles(s, triangle_queue);
		removed += removed_new;
		LOG(INF) << "Removed and also added " << LOG::pad_number_left(removed_new, 5) << " vertices: Thin triangles resolved";
	}

	if (_rotate && detection_res == PointClassifier::ALL) {
		LOG(INF) << "Rotating face";
		rotate(s, sight_dir, vertical_dir, face_center);
	}

	LOG(INF) << LOG::pad_right("Prepearing vertex normals");
	s.prepare_normals();
}

size_t Cleaner::remove_multiple_face_edges(Scene& s) {

	std::unordered_set<Vertex*> to_remove;
	size_t nt = s.triangles_num();
	// check number of opposite croners of each triangle corner
	for (size_t i = 0; i < nt; ++i) {
		auto& t = s.get_face(i);
		Vertex* v0,* v1,* v2;
		t.get_vertices(v0, v1, v2);
		if (t.get_opposite(0).size() > 1){
			to_remove.insert(v1);
			to_remove.insert(v2);
		}
		if (t.get_opposite(1).size() > 1){
			to_remove.insert(v0);
			to_remove.insert(v2);
		}
		if (t.get_opposite(2).size() > 1){
			to_remove.insert(v0);
			to_remove.insert(v1);
		}
	}
	// remove defective edges
	for (auto&& v : to_remove) s.remove_vertex(*v);
	return to_remove.size();
}

size_t Cleaner::remove_non_manifold_vertices(Scene& s) {

	std::unordered_set<Vertex*> to_remove;
	size_t nv = s.vertices_num();
	for (size_t i = 0; i < nv; ++i) {
		auto& v = s.get_vertex(i);
		// find beginning of the vertex umbrella
		auto corners = v.get_corners_copy();
		if (corners.empty()) continue;
		auto curr_corner = &(corners[0]->get_next());
		if (corners.size() != v.get_one_ring().size()) {
			for (auto&& c : corners) {
				if (c->get_previous().get_opposite().empty()) {
					curr_corner = &c->get_next();
					break;
				}
			}
		}
		// check correctness of the vertex
		while (!corners.empty()) {
			auto& opp = curr_corner->get_opposite();
			auto it = std::find(corners.begin(), corners.end(), &(curr_corner->get_previous()));
			if (it != corners.end()) corners.erase(it);
			else break;
			if (opp.empty() || opp.size() > 1) break;
			curr_corner = &(opp[0]->get_previous());
		}
		if (!corners.empty()){
			to_remove.insert(&v);
		}
	}
	// remove non-2-manifold vertices
	for (auto&& v : to_remove) s.remove_vertex(*v);
	return to_remove.size();
}

size_t Cleaner::remove_minor_components(Scene& s) {

	size_t removed = 0;
	size_t nv = s.vertices_num();
	if (nv == 0) return removed;

	size_t seen_num = 0;
	std::vector<std::unordered_set<Vertex*>> components;
	std::vector<bool> seen(nv);

	// simple BFS
	while (seen_num < nv) {
		std::unordered_set<Vertex*> new_component;
		std::stack<Vertex*> visited;
		// search for new component seed
		for (size_t i = 0; i < seen.size(); ++i) {
			if (seen[i]) continue;
			visited.push(&s.get_vertex(i));
			new_component.insert(&s.get_vertex(i));
			seen[i] = true;
			++seen_num;
			break;
		}
		// expand component
		while (!visited.empty()) {
			auto v = visited.top();
			visited.pop();
			for (auto&& n : v->get_one_ring()) {
				if (seen[n->idx()]) continue;
				visited.push(n);
				new_component.insert(n);
				seen[n->idx()] = true;
				++seen_num;
			}
		}
		components.push_back(std::move(new_component));
	}

	// find maximal component
	size_t max_idx = 0;
	size_t max_size = 0;
	for (size_t i = 0; i < components.size(); ++i) {
		if (components[i].size() <= max_size) continue;
		max_idx = i;
		max_size = components[i].size();
	}

	// remove everything except the maximal component
	int original_size = (int) s.vertices_num();
	for (size_t i = 0; i < components.size(); ++i) {
		if (i == max_idx) continue;
		size_t component_size = components[i].size();
		if (component_size > 0.3 * original_size) {
			LOG(WRN) << "Large component removed: " << (int)((double) component_size / original_size * 100.0) << " % of vertices. You'd better check it!";
		}
		for (auto&& v : components[i]) s.remove_vertex(*v);
		removed += components[i].size();
	}

	return removed;
}

size_t Cleaner::crop_face(Scene& s, const Vector3d& nose, const Vector3d& root, const Vector3d& sight_dir,
						  const Vector3d& vertical_dir, const Vector3d& face_center, bool points_valid) {
	size_t removed = 0;
	std::unordered_set<Vertex*> to_remove;
	double nose_to_root = (nose - root).length();

	for (auto&& v : s.get_vertices()) {
		double d = (nose - v->pos()).length();
		auto& c = v->curv_smooth();
		// apply decision criteria
		if ((d > _near * _unit && std::sqrt((c.max * c.max + c.min * c.min) / 2) > _curvedness_tolerance) ||
			d > _far * _unit ||
			(points_valid && _crop_by_normal &&
			 d / nose_to_root > _n2ch_over_n2r_mean - 3 * _n2ch_over_n2r_dev &&
			 is_outgoing(*v, sight_dir, vertical_dir, face_center))) {
			to_remove.insert(&(*v));
		}
	}

	// actual removal
	for (auto&& v : to_remove) s.remove_vertex(*v);
	removed += to_remove.size();

	// 'smooth' boundary of the trimmed facial area
	removed += remove_boundary_protrusions(s);

	return removed;
}

bool Cleaner::is_outgoing(const Vertex& v, const Vector3d& dir_forward, const Vector3d& dir_vertical, const Vector3d& center) {

	Vector3d position = v.pos() - center;
	double angle_position = MathSupport::angle_between(dir_forward, position) - PI / 2.0;
	double angle_normal   = MathSupport::angle_between(dir_forward, v.normal());
	Vector3d projection = (position - dir_forward * Vector3d::dot(position, dir_forward)).normalized();	
	double cos = Vector3d::dot(projection, dir_vertical);
	double weight_normal = cos;
	double weight_angle = cos;
	// exponent :D --> used for thresholds blending
	for (int i = 0; i < 3; ++i) weight_angle *= weight_angle;
	weight_angle = std::max(0.0, weight_angle);

	// ugly, but working cropping criteria
	return ((1 + (_ears_normal_threshold / _neck_normal_threshold - 1) * weight_normal) * angle_normal > _ears_normal_threshold) ||
		   ((1 + (_ears_angle_threshold  / _neck_angle_threshold  - 1) * weight_angle) * angle_position > _ears_angle_threshold);
}

size_t Cleaner::remove_boundary_protrusions(Scene& s) {

	size_t removed = 0;
	std::unordered_set<Vertex *> to_remove;

	// number of neighbourging edges we are going to think of
	int length_multiplier_init = (int) (_wide_protrusion_range * _unit / s.mean_edge_length());
	int length_multiplier = length_multiplier_init;

	// remove protrusions
	while (length_multiplier >= 1) {

		boundary b = s.get_outer_boundary();
		int b_size = (int) b.size();

		// get threshold
		double coef = (length_multiplier_init == 1 ? 1 : (length_multiplier - 1) / (length_multiplier_init - 1));
		double max_angle = coef * _wide_protrusion_angle + (1 - coef) * _tiny_protrusion_angle;

#pragma omp parallel for
		for (int i = 0; i < b_size; ++i) {
			int next = (i + 1) % b_size;
			int prev = (int) MathSupport::pos_mod(i - 1, (int) b_size);
			double next_d = 0;
			double prev_d = 0;
			double max_d = length_multiplier * s.mean_edge_length();
			while (next_d < max_d) {
				Vector3d old_pos = b[next]->pos();
				next = (next + 1) % b_size;
				next_d += (old_pos - b[next]->pos()).length();
			}
			while (prev_d < max_d) {
				Vector3d old_pos = b[prev]->pos();
				prev = MathSupport::pos_mod(prev - 1, b_size);
				prev_d += (old_pos - b[prev]->pos()).length();
			}
			double angle = MathSupport::angle_between(b[next]->pos() - b[i]->pos(), b[prev]->pos() - b[i]->pos(), b[i]->normal());
			if (angle > max_angle) {
#pragma omp critical
				{
					to_remove.insert(b[i]);
				}
			}
		}

		// reduce the neighbourhood size
		if (to_remove.empty()) {
			length_multiplier /= 2;
		} else {
			// remove some protrusions
			for (auto&& v : to_remove) s.remove_vertex(*v);
			removed += to_remove.size();
			to_remove.clear();
			// and clear non-manifold vertices
			removed += remove_non_manifold_vertices(s);
			removed += remove_minor_components(s);
		}
	}

	// clear contour
	while (true) {
		to_remove.clear();
		boundary b = s.get_outer_boundary();
		for (auto&& v : b) {
			if (v->get_corners().size() == 1) to_remove.insert(v);
		}
		if (to_remove.empty()) break;
		else {
			for (auto&& v : to_remove) s.remove_vertex(*v);
			removed += to_remove.size();
		}
	}

	return removed;
}

size_t Cleaner::remove_defects(Scene & s) {

	std::unordered_set<Vertex*> vertices_to_remove;
	
	// vertices with weird curvatures 
	for (auto&& v : s.get_vertices()) {
		auto& c = v->curv();
		double cm = (c.max + c.min) / 2;
		double cg = c.max * c.min;
		auto& cs = v->curv_smooth();
		double sm = (cs.max + cs.min) / 2;
		double sg = cs.max * cs.min;
		if (cm > _max_mean_curv    || cm < _min_mean_curv   ||
			sm > _max_mean_smooth  || sm < _min_mean_smooth ||
			cg > _max_gauss_curv   || cg < _min_gauss_curv  ||
			sg > _max_gauss_smooth || sg < _min_gauss_smooth) {
			vertices_to_remove.insert(v.get());
		}
	}

	// edges with weird angles between its faces
	for (auto&& v : s.get_vertices()) {
		auto v_pos = v->pos();
		for (auto&& c : v->get_corners()) {
			if (!c->get_previous().get_opposite().empty()) {
				auto edge = c->get_next().v->pos() - v_pos;
				auto c1 = c->get_previous().v->pos() - v_pos;
				auto c2 = c->get_previous().get_opposite()[0]->v->pos() - v_pos;
				auto t1_normal = Vector3d::cross(c1, edge).normalized();
				auto t2_normal = Vector3d::cross(edge, c2).normalized();
				double angle = std::fmod((MathSupport::angle_between(t2_normal, t1_normal, edge.normalized()) + PI), 2 * PI);
				if (angle > _maximal_angle || angle < _minimal_angle) {
					vertices_to_remove.insert(c->get_next().v);
					vertices_to_remove.insert(v.get());
				}
			}
		}
	}

	// intersecting triangles
	remove_intersecting_triangles(s, vertices_to_remove);

	for (auto&& v : vertices_to_remove) s.remove_vertex(*v);
	return vertices_to_remove.size();
}

void Cleaner::remove_intersecting_triangles(Scene & s, std::unordered_set<Vertex*> & to_remove) {

	std::unordered_set<Face*> faces_to_remove;
	// create triangle buskets in order to avoid quadratic checking
	auto buckets = partition_triangles(s, 100);
	// check buckets for intersections
#pragma omp parallel for
	for (int i = 0; i < (int) buckets.size(); ++i) {
		auto b_intersections = get_intersecting_faces(buckets[i], s);
#pragma omp critical
		{
			for (auto&& f : b_intersections) faces_to_remove.insert(f);
		}
	}
	//for (auto&& f : faces_to_remove) s.remove_face(*f);
	// remove all adjacent vertices, removing just faces can cause problems in some complex situations
	for (auto&& f : faces_to_remove) {
		Vertex* v1,* v2,* v3;
		s.get_triangle_vertices(*f, v1, v2, v3);
		to_remove.insert(v1);
		to_remove.insert(v2);
		to_remove.insert(v3);
	}
}

std::vector<std::vector<Face*>> Cleaner::partition_triangles(const Scene & s, int n) {

	std::vector<std::vector<Face*>> partitions;
	std::vector<Face*> partition_init;

	// initial huge partition
	for (size_t i = 0; i < s.triangles_num(); ++i) partition_init.push_back(&s.get_face(i));
	partitions.push_back(partition_init);

	// while we can divide a partition
	bool complete = false;
	while (!complete) {

		std::vector<std::vector<Face*>> new_partitions;
		complete = true;

		for (auto&& partition : partitions) {
			// if the partition is small enough, skip it
			if (partition.size() <= (size_t) n) {
				new_partitions.push_back(std::move(partition));
				continue;
			}
			std::vector<Face*> gt_bucket; // first new bucket
			std::vector<Face*> lt_bucket; // second new bucket
			std::vector<Face*> extracted; // triangles which will be in both new buckets

			do {
				Vector3d max(-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max());
				Vector3d min( std::numeric_limits<double>::max(),  std::numeric_limits<double>::max(),  std::numeric_limits<double>::max());
				Vertex* v1, *v2, *v3;
				// find partition bounds
				for (auto&& t : partition) {			
					s.get_triangle_vertices(*t, v1, v2, v3);
					max = Vector3d::max(max, v1->pos());
					max = Vector3d::max(max, v2->pos());
					max = Vector3d::max(max, v3->pos());
					min = Vector3d::min(min, v1->pos());
					min = Vector3d::min(min, v2->pos());
					min = Vector3d::min(min, v3->pos());
				}
				// get split plane <-- largest dimension, halved
			    std::array<double, 3> l = { std::abs(max[0] - min[0]), std::abs(max[1] - min[1]), std::abs(max[2] - min[2]) };
				size_t max_idx = std::distance(l.begin(), std::max_element(l.begin(), l.end()));
				double v = (max[(int) max_idx] + min[(int) max_idx]) / 2.0;
				std::vector<Face*> both;
				// sort partition triangles into the new partitions
				for (auto&& t : partition) {
					s.get_triangle_vertices(*t, v1, v2, v3);
					auto v1p = v1->pos()[(int) max_idx]; v1p -= v;
					auto v2p = v2->pos()[(int) max_idx]; v2p -= v;
					auto v3p = v3->pos()[(int) max_idx]; v3p -= v;
					if (std::fabs(v1p) < 1e-8) v1p = 0;
					if (std::fabs(v2p) < 1e-8) v2p = 0;
					if (std::fabs(v3p) < 1e-8) v3p = 0;
					if (v1p * v2p >= 0 && v1p * v3p >= 0) {
						if (v1p < 0 || v2p < 0 || v3p < 0) gt_bucket.push_back(t);
						else lt_bucket.push_back(t);
					}
					else {
						both.push_back(t);
						gt_bucket.push_back(t);
						lt_bucket.push_back(t);
					}
				}
				// if no division was done, extract the largest triangle and repeat
				if ((lt_bucket.size() == partition.size() || gt_bucket.size() == partition.size()) && !partition.empty()) {
					double m_area = std::numeric_limits<double>::min();
					Face* m = nullptr;
					for (auto&& t : both) {
						s.get_triangle_vertices(*t, v1, v2, v3); 
						double area = Vector3d::cross(v2->pos() - v1->pos(), v3->pos() - v1->pos()).length();
						if (area > m_area) {
							m_area = area;
							m = t;
						}
					}
					partition.erase(std::remove(partition.begin(), partition.end(), m), partition.end());
					extracted.push_back(m);
					lt_bucket.clear();
					gt_bucket.clear();
					continue;
				}
				break;

			} while (true);

			// finally, add new partitions
			if (partition.empty()) {
				new_partitions.push_back(std::move(extracted));
				continue;
			}
			gt_bucket.insert(gt_bucket.end(), extracted.begin(), extracted.end());
			new_partitions.push_back(std::move(gt_bucket));
			lt_bucket.insert(lt_bucket.end(), extracted.begin(), extracted.end());
			new_partitions.push_back(std::move(lt_bucket));

			complete = false;
		}
		// repeat and repeat until all partitions are small enough or cannot be divided
		partitions = std::move(new_partitions);
	}

	return partitions;
}

std::unordered_set<Face*> Cleaner::get_intersecting_faces(const std::vector<Face*> faces, const Scene& s) {

	std::unordered_set<Face*> intersecting;
	for (size_t i = 0; i < faces.size(); ++i) {
		for (size_t j = i + 1; j < faces.size(); ++j) {
			Vertex* v1, *v2, *v3, *u1, *u2, *u3;
			s.get_triangle_vertices(*faces[i], v1, v2, v3);
			s.get_triangle_vertices(*faces[j], u1, u2, u3);
			if (!MathSupport::triangles_intersecting(v1->pos(), v2->pos(), v3->pos(), u1->pos(), u2->pos(), u3->pos())) continue;
			intersecting.insert(faces[i]);
			intersecting.insert(faces[j]);
		}
	}
	return intersecting;
}

size_t Cleaner::remove_thin_triangles(Scene & s, std::deque<Face*>& triangle_queue) {

	size_t removed = 0;
	float edge_or_triangle = 0.3f; // threshold for swapping or vertex merging

	while (!triangle_queue.empty()) {
		
		auto i = triangle_queue.front();
		triangle_queue.pop_front();
		Vertex* v1, *v2, *v3;
		s.get_triangle_vertices(*i, v1, v2, v3);

		// compute the aspect ratio of the triangle, used for thin triangles detection
		if (MathSupport::triangle_aspect_ratio(v1->pos(), v2->pos(), v3->pos()) >= _min_triangle_aspect_ratio) continue;

		auto a = (v1->pos() - v2->pos()).length();
		auto b = (v2->pos() - v3->pos()).length();
		auto c = (v1->pos() - v3->pos()).length();
		// min and max edges
		double max = std::max(a, std::max(b, c));
		double min = std::min(a, std::min(b, c));

		// swap or merge?
		if ((min / max) >= edge_or_triangle) {	

			//	
			// edge swap

			Vertex* swap1 = v1;
			Vertex* swap2 = v2;
			if (max == b) { swap1 = v2; swap2 = v3; }
			else if (max == c) { swap1 = v3; swap2 = v1; }
			auto nv = swap1->get_corners();
			auto pv = swap2->get_corners();
			std::vector<Corner*> corners;
			// get adjacent triangles
			for (std::vector<Corner*>::const_iterator it = nv.begin(); it != nv.end(); it++) {
				size_t idx1 = (*it)->idx;
				auto match = std::find_if(pv.begin(), pv.end(), [&it](Corner* item) {
					return item->f == (*it)->f;
				});
				if (match != pv.end()) {
					size_t idx2 = (*match)->idx;
					corners.push_back(&(*it)->f->get_corner(3 - idx1 - idx2));
				}
			}
			if (corners.size() == 2) {
				// the edge has two adjacent triangles, we need to perform edge an edge swap
				auto old_handle1 = corners[0]->f;
				auto old_handle2 = corners[1]->f;
				// add the first new triangle
				if (corners[0]->v != corners[1]->v) {
					auto new_handle1 = s.add_face(corners[0]->v, corners[0]->get_next().v, corners[1]->v);
					if (s.has_textures()) {
						s.set_face_texture_coordinate(new_handle1,
													  old_handle1->get_texture_coordinate(corners[0]->idx),
													  old_handle1->get_texture_coordinate(corners[0]->get_next().idx),
													  old_handle2->get_texture_coordinate(corners[1]->idx));
					}
					// add the second new triangle
					auto new_handle2 = s.add_face(corners[0]->get_previous().v, corners[0]->v, corners[1]->v);
					if (s.has_textures()) {
						s.set_face_texture_coordinate(new_handle2,
													  old_handle1->get_texture_coordinate(corners[0]->get_previous().idx),
													  old_handle1->get_texture_coordinate(corners[0]->idx),
													  old_handle2->get_texture_coordinate(corners[1]->idx));
					}
					for (auto it = triangle_queue.begin(); it != triangle_queue.end(); ) {
						if (*it == old_handle1 || *it == old_handle2) it = triangle_queue.erase(it);
						else ++it;
					}
				}			
				//triangle_queue.push_back(&s.get_face(new_handle1));
				//triangle_queue.push_back(&s.get_face(new_handle2));
				// remove old triangles
				s.remove_face(*old_handle1);
				s.remove_face(*old_handle2);
			}
			else if (corners.size() == 1) {
				// the edge has only one adjacent triangle, we can remove whole thin triangle
				for (auto it = triangle_queue.begin(); it != triangle_queue.end(); ) {
					if (*it == corners[0]->f) it = triangle_queue.erase(it);
					else ++it;
				}
				s.remove_face(*corners[0]->f);
			}
		}
		else {

			//
			// edge collapse

			Vertex* merge1 = v1;
			Vertex* merge2 = v2;
			if (min == b) { merge1 = v2; merge2 = v3; }
			else if (min == c) { merge1 = v3; merge2 = v1; }
			std::unordered_set<Face*> added_triangles;
			std::unordered_set<Face*> removed_triangles;
			s.merge_edge(*merge1, *merge2, added_triangles, removed_triangles);
			for (auto it = triangle_queue.begin(); it != triangle_queue.end(); ) {
				if (removed_triangles.find(*it) != removed_triangles.end()) it = triangle_queue.erase(it);
				else ++it;
			}
			for (auto&& t : added_triangles) triangle_queue.push_back(t);
			++removed;
		}
	}

	return removed;
}

void Cleaner::rotate(Scene & s, const Vector3d& sight_dir, const Vector3d& vertical_dir, const Vector3d& face_center) {

	//
	// compute rotation angles and so on ...

	auto dir_angle = MathSupport::angle_between(sight_dir, Vector3d(0, 0, 1));
	auto dir_axis = Vector3d::cross(sight_dir, Vector3d(0, 0, 1)).normalized();
	auto sin1 = std::sin(dir_angle);
	auto cos1 = std::cos(dir_angle);

	auto rotated_vertical_dir = vertical_dir * cos1 + 
							    Vector3d::dot(vertical_dir, dir_axis) * dir_axis * (1 - cos1) + 
								Vector3d::cross(dir_axis, vertical_dir) * sin1;
	auto vertical_angle = MathSupport::angle_between(rotated_vertical_dir, Vector3d(0, -1, 0), Vector3d(0, 0, 1));
	auto sin2 = std::sin(vertical_angle);
	auto cos2 = std::cos(vertical_angle);

	// rotate mesh vertices
	for (auto&& v : s.get_vertices()) {
		auto old_pos = v->pos();
		auto new_pos = (old_pos - face_center);
		new_pos = new_pos * cos1 + Vector3d::dot(new_pos, dir_axis) * dir_axis * (1 - cos1) + Vector3d::cross(dir_axis, new_pos) * sin1;
		v->set_pos(Vector3d(cos2 * new_pos.x - sin2 * new_pos.y, sin2 * new_pos.x + cos2 * new_pos.y, new_pos.z));
	}

	// rotate mesh landmarks
	if (_landmarks) {
		Vector3d lnd[6];
		// get cached landmarks
		_pc->get_landmark_positions(lnd[0], lnd[1], lnd[2], lnd[3], lnd[4], lnd[5]);
		// rotate them
		for (size_t i = 0; i < 6; i++) {
			auto new_pos = (lnd[i] - face_center);
			new_pos = new_pos * cos1 + Vector3d::dot(new_pos, dir_axis) * dir_axis * (1 - cos1) + Vector3d::cross(dir_axis, new_pos) * sin1;
			lnd[i] = Vector3d(cos2 * new_pos.x - sin2 * new_pos.y, sin2 * new_pos.x + cos2 * new_pos.y, new_pos.z);
		}
		// update cache
		_pc->set_landmark_positions(lnd[0], lnd[1], lnd[2], lnd[3], lnd[4], lnd[5]);
	}
}

