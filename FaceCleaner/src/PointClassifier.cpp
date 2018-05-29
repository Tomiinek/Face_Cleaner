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
* @file    PointClassifier.cpp
*/

#include "PointClassifier.h"

PointClassifier::PointClassifier(const boost::program_options::variables_map& vm, const std::string& nose_tip_path, const std::string& nose_root_path, 
								 const std::string& eye_path, const std::string& mouth_path, int threads)
  : _unit(vm["scale"].as<double>()), 
	_e2e({vm["e2e-mean"].as<double>(), vm["e2e-dev"].as<double>()}),
	_l2l({vm["m2m-mean"].as<double>(), vm["m2m-dev"].as<double>()}),
	_n2r({vm["n2r-mean"].as<double>(), vm["n2r-dev"].as<double>()}),
	_n2e_o_n2r({vm["n2e-over-n2r-mean"].as<double>(), vm["n2e-over-n2r-dev"].as<double>()}),
	_r2e_o_e2e({vm["r2e-over-e2e-mean"].as<double>(), vm["r2e-over-e2e-dev"].as<double>()}),
	_r2l_o_n2r({vm["r2m-over-n2r-mean"].as<double>(), vm["r2m-over-n2r-dev"].as<double>()}),
	_n2l_o_l2l({vm["n2m-over-m2m-mean"].as<double>(), vm["n2m-over-m2m-dev"].as<double>()}) {

	// set paths to models, they are not loaded yet!
	_models.clear();
	_models.insert({"nose-tip", Model(nose_tip_path)});
	_models.insert({"nose-root", Model(nose_root_path)});
	_models.insert({"eye-corner", Model(eye_path)});
	_models.insert({"lips-corner", Model(mouth_path)});

	// max number of threads
	_max_threads = threads;

	// create batch for inference, just this instance is used for all predictions
	_batch_size = 1024;
	_batch = tensorflow::Tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({_batch_size, 6 * 16 + 2}));
}

bool PointClassifier::load_models() {
	// try to open and load all models (paths specified in constructor)
	for (auto&& m : _models) {
		if (!m.second.open(_max_threads)) {
			_models.clear();
			return false;
		}
	}
	return true;
}

PointClassifier::return_value PointClassifier::update_face_landmarks(Scene & s, Vertex *& left_eye, Vertex *& right_eye, Vertex *& left_lips,
																     Vertex *& right_lips, Vertex *& nose, Vertex *& root) {	
	// clear probabilities cached by models
	for (auto&& m : _models) m.second.clear();

	//
	// prepare buffers for mean and Gauss curvatures of all vertices

	std::vector<double> gauss(s.vertices_num());
	std::vector<double> means(s.vertices_num());
	for (auto&& u : s.get_vertices()) {
		auto& c = u->curv_smooth();
		double m = (c.max + c.min) / 2.0;
		double g = c.max * c.min;
		if (g < min_gauss_curv || std::isnan(g)) gauss[u->idx()] = 0.0f;
		else if (g > max_gauss_curv) gauss[u->idx()] = 1.0f;
		else gauss[u->idx()] = (g - min_gauss_curv) / (max_gauss_curv - min_gauss_curv);
		if (m < min_means_curv || std::isnan(m)) means[u->idx()] = 0.0f;
		else if (m > max_means_curv) means[u->idx()] = 1.0f;
		else means[u->idx()] = (m - min_means_curv) / (max_means_curv - min_means_curv);
	}
	
	//
	// prepare vertex subset which should be sufficient to find landmarks
	// this reduces number of vertices needed to be evaluated --> time savings

	std::vector<Vertex*> vertices_selection; 
	{
		size_t n = s.vertices_num();
		std::vector<bool> vertices_selection_helper(n);
		while (true) {
			size_t unvisited = n;
			for (size_t i = 0; i < n; ++i) {
				if (!vertices_selection_helper[i]) {
					unvisited = i;
					break;
				}
			}
			if (unvisited == n) break;
			auto& v = s.get_vertex(unvisited);
			// we skip neighbours
			for (auto&& neighbor : v.get_one_ring()) {
				vertices_selection_helper[neighbor->idx()] = true;
			}
			// and add just some vertices
			vertices_selection_helper[unvisited] = true;
			vertices_selection.push_back(&v);
		}
	}

	// computation of all feature vectors and inference
	evaluate(s, vertices_selection, gauss, means);
	// compute areas where were detected some interesting points in detail
	for (auto&& m : _models) expand_positives(s, m.second, gauss, means);

	// discard components which are < 6
	int min_component_size = 6;
	// get representatives for all components (geometric median)
	auto nose_tips = extract_central_points(_models["nose-tip"].get_positives(), min_component_size);
	auto nose_root = extract_central_points(_models["nose-root"].get_positives(), min_component_size);
	auto eye_corner = extract_central_points(_models["eye-corner"].get_positives(), min_component_size);
	auto mouth_corner = extract_central_points(_models["lips-corner"].get_positives(), min_component_size);

	// if there is no nose tip representative, we can stop the detection
	if (nose_tips.empty()) {
		_result = PointClassifier::NTHNG;
		return _result;
	}

	// list of good sets of landmarks
	std::vector<std::array<std::pair<Vertex*, std::pair<double, double>>, 6>> good_points;

	//
	// ultra-ugly filtering, based on the three-sigma rule

	for (auto eye_it1 = eye_corner.begin(); eye_it1 != eye_corner.end(); ++eye_it1){
		for (auto eye_it2 = eye_it1; ++eye_it2 != eye_corner.end();) {
			double eye_dist = (eye_it1->first->pos() - eye_it2->first->pos()).length();
			if (!_e2e.pass(eye_dist, _unit)) 
				continue;
			for (auto root_it = nose_root.begin(); root_it != nose_root.end(); ++root_it) {
				if (!_r2e_o_e2e.pass((eye_it1->first->pos() - root_it->first->pos()).length() / eye_dist) || 
					!_r2e_o_e2e.pass((eye_it2->first->pos() - root_it->first->pos()).length() / eye_dist)) 
					continue;
				for (auto nose_it = nose_tips.begin(); nose_it != nose_tips.end(); ++nose_it) {
					double n2r_dist = (root_it->first->pos() - nose_it->first->pos()).length();
					if (!_n2r.pass(n2r_dist, _unit) || 
						!_n2e_o_n2r.pass((eye_it1->first->pos() - nose_it->first->pos()).length() / n2r_dist) ||
						!_n2e_o_n2r.pass((eye_it2->first->pos() - nose_it->first->pos()).length() / n2r_dist))
						continue;
					for (auto mouth_it1 = mouth_corner.begin(); mouth_it1 != mouth_corner.end(); ++mouth_it1){
						for (auto mouth_it2 = mouth_it1; ++mouth_it2 != mouth_corner.end();) {	
							auto l2l_dist = (mouth_it1->first->pos() - mouth_it2->first->pos()).length();
							if (!_l2l.pass(l2l_dist, _unit) ||
								!_n2l_o_l2l.pass((mouth_it1->first->pos() - nose_it->first->pos()).length() / l2l_dist) ||
								!_n2l_o_l2l.pass((mouth_it2->first->pos() - nose_it->first->pos()).length() / l2l_dist) ||
								!_r2l_o_n2r.pass((mouth_it1->first->pos() - root_it->first->pos()).length() / n2r_dist) ||
								!_r2l_o_n2r.pass((mouth_it2->first->pos() - root_it->first->pos()).length() / n2r_dist))
								continue;
							auto nearer1 = ((mouth_it1->first->pos() - eye_it1->first->pos()).length() <
									        (mouth_it1->first->pos() - eye_it2->first->pos()).length() ? 
											eye_it1->first : eye_it2->first);
							auto nearer2 = ((mouth_it2->first->pos() - eye_it1->first->pos()).length() <
								            (mouth_it2->first->pos() - eye_it2->first->pos()).length() ? 
											eye_it1->first : eye_it2->first);
							if (nearer1 == nearer2) continue;
							good_points.push_back({*nose_it, *eye_it1, *eye_it2, *root_it, *mouth_it1, *mouth_it2});	
						}
					}
				}
			}
		}
	}

	// no suitable sets of landmarks --> :(
	if (good_points.empty()) {		
		nose = nose_tips[0].first;
		_nose = nose->pos();
		_result = PointClassifier::NOSE;
		return _result;
	}

	//
	// if there are more sets of landmarks, we choose only one with respect to component weights

	struct
	{
		bool operator()(std::array<std::pair<Vertex*, std::pair<double, double>>, 6>& a, 
						std::array<std::pair<Vertex*, std::pair<double, double>>, 6>& b) const {
			if (a[0].second.second != b[0].second.second) {
				return a[0].second.second > b[0].second.second; 
			}
			else if (a[1].second.first * a[2].second.first != b[1].second.first * b[2].second.first) {
				return a[1].second.first * a[2].second.first > b[1].second.first * b[2].second.first;
			} 
			else if (a[4].second.first * a[5].second.first != b[4].second.first * b[5].second.first) {
				return a[4].second.first * a[5].second.first > b[4].second.first * b[5].second.first;
			}
			else if (a[3].second.first != b[3].second.first) { return a[3].second.first > b[3].second.first; }
			return true;
		}
	} face_triangles_comparer;
	std::sort(good_points.begin(), good_points.end(), face_triangles_comparer);

	//
	// save found landmarks and return value

	left_eye = good_points[0][1].first;
	_left_eye = left_eye->pos();
	right_eye = good_points[0][2].first;
	_right_eye = right_eye->pos();
	left_lips = good_points[0][4].first;
	_left_lips = left_lips->pos();
	right_lips = good_points[0][5].first;
	_right_lips = right_lips->pos();
	nose = good_points[0][0].first;
	_nose = nose->pos();
	root = good_points[0][3].first;
	_root = root->pos();
	_result = PointClassifier::ALL;
	return _result;
}

PointClassifier::return_value PointClassifier::get_face_landmarks(Vertex *& left_eye, Vertex *& right_eye, Vertex *& left_lips, Vertex *& right_lips, Vertex *& nose, Vertex *& root, Scene & scene) {
	
	double min_left_eye, min_right_eye, min_left_lips, min_right_lips, min_nose, min_root;
	min_left_eye = min_right_eye = min_left_lips = min_right_lips = min_nose = min_root = std::numeric_limits<double>::max();

	double d;
	for (auto&& v : scene.get_vertices()) {
		// check nearest eyes
		d = (v->pos() - _left_eye).length_squared();
		if (d < min_left_eye) { min_left_eye = d; left_eye = v.get(); }
		d = (v->pos() - _right_eye).length_squared();
		if (d < min_right_eye) { min_right_eye = d; right_eye = v.get(); }
		// check nearest lips
		d = (v->pos() - _left_lips).length_squared();
		if (d < min_left_lips) { min_left_lips = d; left_lips = v.get(); }
		d = (v->pos() - _right_lips).length_squared();
		if (d < min_right_lips) { min_right_lips = d; right_lips = v.get(); }
		// check nearest nose tip
		d = (v->pos() - _nose).length_squared();
		if (d < min_nose) { min_nose = d; nose = v.get(); }
		// check nearest nose root
		d = (v->pos() - _root).length_squared();
		if (d < min_root) { min_root = d; root = v.get(); }
	}	
	return _result;
}

PointClassifier::return_value PointClassifier::get_landmark_positions(Vector3d& left_eye, Vector3d & right_eye, Vector3d & left_lips, Vector3d & right_lips, Vector3d & nose, Vector3d & root) {
	
	left_eye = _left_eye;
	right_eye = _right_eye;
	left_lips = _left_lips;
	right_lips = _right_lips;
	nose = _nose;
	root = _root;
	return _result;
}

void PointClassifier::set_landmark_positions(Vector3d left_eye, Vector3d right_eye, Vector3d left_lips, Vector3d right_lips, Vector3d nose, Vector3d root) {

	_left_eye = left_eye;
	_right_eye = right_eye;
	_left_lips = left_lips;
	_right_lips = right_lips;
	_nose = nose;
	_root = root;
}

std::tuple<std::array<float, 6>, std::vector<float>> PointClassifier::get_feature_vector(Scene& s, Vertex & v, int n, const std::vector<double>& gauss, const std::vector<double>& means) {
	
	Histogram histogramg1(n);
	Histogram histogramg2(n);
	Histogram histogramg3(n);
	Histogram histogramm1(n);
	Histogram histogramm2(n);
	Histogram histogramm3(n);

	std::unordered_set<Vertex*> visited{&v};
	std::queue<Vertex*> frontier;
	frontier.push(&v);
	Vector3d position = v.pos();

	// weights of vertices in histograms	
	double c1 = 0;
	double c2 = 0;
	double c3 = 0;

	// stats, used for decision whether it is needed to do inference
	double min_g = std::numeric_limits<double>::max();
	double max_g = std::numeric_limits<double>::min();
	double min_m = std::numeric_limits<double>::max();
	double max_m = std::numeric_limits<double>::min();
	double mean_g = 0.0;
	double mean_m = 0.0;

	// simple BFS starting in the vertex
	while (!frontier.empty()) {
		std::queue<Vertex*> new_frontier;
		while (!frontier.empty()) {
			auto curr = frontier.front();
			frontier.pop();
			auto& ring = curr->get_one_ring();
			if (ring.empty()) continue;
			for (auto neighbor : ring) {
				if (visited.find(neighbor) != visited.end()) continue;
				auto d = (neighbor->pos() - position).length_squared();
				// largest histogram
				if (d <= max_range * max_range * _unit * _unit) {	
					new_frontier.push(neighbor);
					visited.insert(neighbor);
					auto& curv = neighbor->curv_smooth();
					if (std::isnan(curv.max) || std::isnan(curv.min) || std::isnan(curv.min_max)) continue;
					auto area = neighbor->get_voroni_area();
					c1 += area;
					histogramm1.insert(means[neighbor->idx()], area);
					histogramg1.insert(gauss[neighbor->idx()], area);
					mean_m += means[neighbor->idx()] * area;
					mean_g += gauss[neighbor->idx()] * area;
					min_g = std::min(min_g, gauss[neighbor->idx()]);
					max_g = std::max(max_g, gauss[neighbor->idx()]);
					min_m = std::min(min_m, means[neighbor->idx()]);
					max_m = std::max(max_m, means[neighbor->idx()]);
					// medium histogram
					if (d <= mid_range * mid_range * _unit * _unit) {
						c2 += area;
						histogramm2.insert(means[neighbor->idx()], area);
						histogramg2.insert(gauss[neighbor->idx()], area);
						// tiny histogram
						if (d <= min_range * min_range * _unit * _unit) {
							c3 += area;
							histogramm3.insert(means[neighbor->idx()], area);
							histogramg3.insert(gauss[neighbor->idx()], area);
						}
					}
				}
			}
		}
		frontier = std::move(new_frontier);
	}
	// normalize stats
	mean_g /= c1;
	mean_m /= c1;

	std::vector<float> features(6 * n + 2);

	// Gauss part of the feature vector
	for (int j = 0; j < n; j++) features[j] = (float)( histogramg1[j] / c1 );
	for (int j = 0; j < n; j++) features[j + 1 * n] = (float) (histogramg2[j] / c2);
	for (int j = 0; j < n; j++) features[j + 2 * n] = (float) (histogramg3[j] / c3);
	features[3 * n] = (float) gauss[v.idx()];

	// Mean part of the feature vector
	for (int j = 0; j < n; j++) features[3 * n + j + 1] = (float) (histogramm1[j] / c1);
	for (int j = 0; j < n; j++) features[4 * n + j + 1] = (float) (histogramm2[j] / c2);
	for (int j = 0; j < n; j++) features[5 * n + j + 1] = (float) (histogramm3[j] / c3);
	features[6 * n + 1] = (float) means[v.idx()];

	std::array<float, 6> stats = {(float) min_g, (float) mean_g, (float) max_g, (float) min_m, (float) mean_m, (float) max_m};
	return {stats, std::move(features)};
}

void PointClassifier::evaluate(Scene& s, const std::vector<Vertex*>& vertices, const std::vector<double>& gauss, const std::vector<double>& means) {

#pragma omp parallel for
	for (int i = 0; i < vertices.size(); ++i) {
		auto v = vertices[i];
		std::array<float, 6> stats;
		std::vector<float> features;
		std::tie(stats, features) = get_feature_vector(s, *v, 16, gauss, means);
		// skip if the neighborhood is too flat (we do not expect any interesting points there)
		if (std::abs(stats[1] - 0.3333) >= mean_curv_center_tolerance ||
			std::abs(stats[2] - stats[0]) >= mean_curv_range_tolerance ||
			std::abs(stats[4] - 0.5555) >= gauss_curv_center_tolerance ||
			std::abs(stats[5] - stats[3]) >= gauss_curv_range_tolerance) {
#pragma omp critical(batch)
				{
					// add the feature vector to the evaluation batch
					size_t batch_load_size = _batch_load.size();
					size_t features_size = features.size();
					for (size_t j = 0; j < features_size; j++) {
						_batch.flat<float>()(batch_load_size * features_size + j) = features[j];
					}
					_batch_load.push_back(v);
					if (_batch_load.size() == _batch_size) {
						for (auto&& m : _models) m.second.feed(_batch_load, _batch);
						_batch_load.clear();
					}
				}
		}
	}
	// evaluate the remaining batch (do not have to be completely filled)
	if (_batch_load.size() != 0) {
		auto sliced_batch = _batch.Slice(0, _batch_load.size());
		for (auto&& m : _models) m.second.feed(_batch_load, sliced_batch);
		_batch_load.clear();
	}
}

void PointClassifier::expand_positives(Scene& s, Model & model, const std::vector<double>& gauss, const std::vector<double>& means) {

	// get all neighbors of model positives
	auto& positives = model.get_positives();
	std::unordered_set<Vertex*> neighbors_to_check;
	for (auto&& p : positives) {
		auto& neighbors = p.first->get_one_ring();
		for (auto&& n : neighbors) neighbors_to_check.insert(n);
	}

	std::vector<Vertex*> batch_load;
	std::vector<Vertex*> vertices_selection;
	// create vector of unordered map ...
	std::copy(neighbors_to_check.begin(), neighbors_to_check.end(), std::back_inserter(vertices_selection));

	// actual evaluation of neighbours of positives
	evaluate(s, vertices_selection, gauss, means);
}

std::vector<std::pair<Vertex*, std::pair<double, double>>> PointClassifier::extract_central_points(
	const std::unordered_map<Vertex*, float>& positives, int min_component_size) {

	//
	// get all components

	std::vector<std::unordered_set<Vertex*>> components;
	std::unordered_set<Vertex*> seen;
	while (seen.size() < positives.size()) {
		std::unordered_set<Vertex*> new_component;
		std::stack<Vertex*> visited;
		for (auto&& p : positives) {
			if (seen.find(p.first) != seen.end()) continue;
			new_component.insert(p.first);
			visited.push(p.first);
			seen.insert(p.first);
			break;
		}
		while (!visited.empty()) {
			auto v = visited.top();
			visited.pop();
			for (auto&& n : v->get_one_ring()) {
				if (positives.find(n) == positives.end() ||
					seen.find(n) != seen.end()) continue;
				visited.push(n);
				new_component.insert(n);
				seen.insert(n);
			}
		}
		components.push_back(std::move(new_component));
	}

	//
	// compute geometric median, geometric mean and total sum of the components
	// note that the computation of geometric mean is naive and that the 
	// computation of geometric is not (becuase of numeric stability of large components)

	std::vector<std::pair<Vertex*, std::pair<double, double>>> centers;
	for (auto&& component : components) {
		if (component.size() < min_component_size) continue;
		float w_sum = 0.0;
		double w_mult_m = 1.0;
		long long w_mult_ex = 0;
		std::vector<std::pair<double, Vertex*>> costs(component.size());
		size_t j = 0;
		for (auto&& p : component) {
			auto w = positives.find(p)->second;
			w_sum += w;
			int i;
			double f1 = std::frexp(w, &i);
			w_mult_m *= f1;
			w_mult_ex += i;
			double cost = 0.0;
			for (auto&& n : component) {
				if (p == n) continue;
				auto n_w = positives.find(n)->second;
				cost += (p->pos() - n->pos()).length() * n_w;
			}
			costs[j] = std::make_pair(cost, p);
			++j;
		}
		double inv_n = 1.0 / component.size();
		w_mult_m = std::pow(std::numeric_limits<double>::radix, w_mult_ex * inv_n) * std::pow(w_mult_m, inv_n);
		Vertex* nearest;
		double minimum = std::numeric_limits<double>::max();
		for (size_t j = 0; j < component.size(); j++) {
			if (costs[j].first < minimum) {
				minimum = costs[j].first;
				nearest = costs[j].second;
			}
		}
		centers.push_back(std::make_pair(nearest, std::make_pair(w_mult_m, w_sum)));
	}

	return centers;
}
