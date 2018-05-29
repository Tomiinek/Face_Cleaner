#ifndef FACECLEANER_POINTCLASSIFIER_H
#define FACECLEANER_POINTCLASSIFIER_H

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
 * @brief   Object which manages the detection of facial landmarks.
 *
 * @file    PointClassifier.h
 * @author  Tomas Nekvinda <tom(at)neqindi.cz>
 */

#pragma once
#define COMPILER_MSVC
#define NOMINMAX

#include <queue>
#include <cmath>
#include <fstream>
#include <iostream>
#include <streambuf>
#include <unordered_map>

#include "Scene.h"
#include "Vertex.h"

#include "boost/program_options.hpp"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/platform/env.h"
#include "tensorflow/cc/framework/ops.h"
#undef LOG

class PointClassifier
{
public:

	/**
	 * @brief Possible return values, ALL point, NOSE tip or NTHNG detected.
	 * 
	 */
	enum return_value : uint8_t { ALL, NOSE, NTHNG };

	/**
	 * @brief Method for an easy check of the three-sigma rule.
	 * 
	 */
	struct condition
	{
		double mean;
		double deviation;
		bool pass(double x, double u = 1.0) const { return std::abs(x / u - mean) < 3 * deviation; }
	};

	/**
	 * @brief Construct a new PointClassifier object.
	 * 
	 * @param vm				Boost Program Options with needed configuration parameters.
	 * @param nose_tip_path 	Path (absolute or relative) to the nose tip classifier.
	 * @param nose_root_path 	Path (absolute or relative) to the nose root classifier.
	 * @param eye_path 			Path (absolute or relative) to the eye corners classifier.
	 * @param mouth_path 		Path (absolute or relative) to the mouth corners classifier.
	 * @param threads 			Maximal number of threads to be used.
	 */
	PointClassifier(const boost::program_options::variables_map& vm, const std::string& nose_tip_path, const std::string& nose_root_path,
					const std::string& eye_path, const std::string& mouth_path, int threads);
	
	/**
	 * @brief Load models for classification.
	 * 
	 * @return @c true if all models were loaded successfully, @c false otherwise.
	 */
	bool load_models();

	/**
	 * @brief Evaluate all vertices of a mesh with all models and save found landmarks.
	 * 
	 * @param s 				Scene with mesh.
	 * @param left_eye 			Resulting left eye corner, nullptr if not found.
	 * @param right_eye 		Resulting right eye corner, nullptr if not found.
	 * @param left_lips 		Resulting left mouth corner, nullptr if not found.
	 * @param right_lips 		Resulting right mouth corner, nullptr if not found.
	 * @param nose 				Resulting nose tip, nullptr if not found.
	 * @param root 				Resulting nose root, nullptr if not found.
	 * @return return_value 	Result of the detection, on of {@c ALL , @c NOSE , @c NTHNG }
	 */
	return_value update_face_landmarks(Scene& s, Vertex *& left_eye, Vertex *& right_eye, Vertex *& left_lips, 
										   Vertex *& right_lips, Vertex *& nose, Vertex *& root);

	/**
	 * @brief Get landmarks found by update_face_landmarks method.
	 * 
	 * @param left_eye			Left eye corner.
	 * @param right_eye			Right eye corner.
	 * @param left_lips			Left mouth corner.
	 * @param right_lips		Right mouth corner.
	 * @param nose				Nose tip.
	 * @param root				Nose root.
	 * @param scene				Scene with mesh.
	 * @return return_value 	Result of the detection, on of {@c ALL , @c NOSE , @c NTHNG }
	 */
	return_value get_face_landmarks(Vertex *& left_eye, Vertex *& right_eye, Vertex *& left_lips,
									Vertex *& right_lips, Vertex *& nose, Vertex *& root, Scene& scene);
	
	/**
	 * @brief Get positions of landmarks, do not have to return positions of existing vertices (after some removals).
	 *  
	 * @param left_eye 			Position of left eye corner.
	 * @param right_eye 		Position of right eye corner.
	 * @param left_lips 		Position of left mouth corner.
	 * @param right_lips 		Position of right mouth corner.
	 * @param nose 				Position of nose tip.
	 * @param root 				Position of nose root.
	 * @return return_value 	Result of the detection, on of {@c ALL , @c NOSE , @c NTHNG }
	 */
	return_value get_landmark_positions(Vector3d& left_eye, Vector3d& right_eye, Vector3d& left_lips, 
										Vector3d& right_lips, Vector3d& nose, Vector3d& root);
	
	/**
	 * @brief Set new landmark positions. Used while rotating mesh, because we need to update the positions.
	 * 
	 * @param left_eye 		Set left eye corner position.
	 * @param right_eye 	Set right eye corner position.
	 * @param left_lips 	Set left mouth corner position.
	 * @param right_lips 	Set right mouth corner position.
	 * @param nose 			Set nose tip position.
	 * @param root 			Set nose root position.
	 */
	void set_landmark_positions(Vector3d left_eye, Vector3d right_eye, Vector3d left_lips, 
								Vector3d right_lips, Vector3d nose, Vector3d root);

private:

	/**
	 * @brief Object representing histogram. Used while bulding feature vectors.
	 * 
	 */
	class Histogram
	{
	public:
		/**
		 * @brief Create a new Histogram object.
		 * 
		 * @param bin_num 	Number of histogram bins.
		 */
		Histogram(int bin_num) : _bin_num(bin_num) { _data.assign(bin_num, 0); }

		/**
		 * @brief Get data from a partivular histogram bin.
		 * 
		 * @param idx 		Bin index, must be in the range [0,_bin_num).
		 * @return double 	Bin value.
		 */
		double operator[](std::size_t idx) { return _data[idx]; }
		const double operator[](std::size_t idx) const { return _data[idx]; }

		/**
		 * @brief Insert an item into the histogram. The item can be weighted.
		 * 
		 * @param value 	Value of the item.
		 * @param weight 	Weight of the item (should be just 1 in a standard frequency histogram).
		 */
		void insert(double value, double weight) {
			if (std::isnan(value)) value = 0.0;
			if (std::isnan(weight)) weight = 0.0;
			int key = (int) (value * _bin_num);
			if (key == _bin_num) --key;
			_data[key] += weight;
		}

	private:
		int _bin_num;
		std::vector<double> _data;
	};

	/**
	 * @brief Object handling a classifier, loading, disposing, feeding.
	 * 
	 */
	class Model
	{
	public:
		Model() : _session(nullptr) {}
		Model(std::string path) : _path(std::move(path)), _session(nullptr) {}

		/**
		 * @brief Load the model and prepare it for usage.
		 * 
		 * @param threads 	The maximal number of threads which can be used durng the inference.
		 * @return 			@c true if the model was loaded successfully, @c false otherwise. 
		 */
		bool open(int threads) {
			tensorflow::SessionOptions options;
			tensorflow::ConfigProto& config = options.config;
			if (threads > 0) {
				config.set_inter_op_parallelism_threads(threads);
				config.set_intra_op_parallelism_threads(threads);
				config.set_use_per_session_threads(false);
			}
			tensorflow::GraphDef graph_def;
			tensorflow::Status load_graph_status = ReadBinaryProto(tensorflow::Env::Default(), _path, &graph_def);
			if (!load_graph_status.ok()) return false;
			_session = tensorflow::NewSession(tensorflow::SessionOptions(options));
			tensorflow::Status session_create_status = _session->Create(graph_def);
			if (!session_create_status.ok()) return false;
			return true;
		}
		
		/**
		 * @brief 
		 * 
		 * @param batch_load 
		 * @param input 
		 */
		void feed(const std::vector<Vertex*>& batch_load, const tensorflow::Tensor& input) {
			std::vector<tensorflow::Tensor> outputs;
			std::vector<std::pair<std::string, tensorflow::Tensor>> inputs = {{"input_node", input}};
			tensorflow::Status run_status = _session->Run(inputs, {"output_node"}, {}, &outputs);
			if (!run_status.ok()) {
				std::cout << "Running of a model failed: " << run_status;
				return;
			}
			for (size_t i = 0; i < batch_load.size(); i++) {
				float probability = outputs[0].tensor<float, (2)>()(2*i+1);
				if (probability > 0.5f) {
					_positives.insert({batch_load[i], (probability - 0.5) * 2.0});
				}
			}
			
		}

		/**
		 * @brief Get probabilities of positives which were saved during the inference.
		 * 
		 * @return 	Vertex - probability pairs (the probability is scaled to [0,1] range from [0.5,1]).
		 */
		const std::unordered_map<Vertex*, float> get_positives() { return _positives; }
		/**
		 * @brief Clear saved probabilities of positives.
		 * 
		 */
		void clear() { _positives.clear(); }

		/**
		 * @brief Release all resources of the classificator.
		 * 
		 */
		~Model() { if (_session != nullptr) _session->Close(); }

	private:
		// path to the model file
		std::string _path;
		// storage of probabilities of positives
		std::unordered_map<Vertex*, float> _positives;
		// tf.Session of the loaded model
		tensorflow::Session* _session;
	};

	/**
	 * @brief Get the feature vector of a vertex.
	 * 
	 * @param s 		Scene with the mesh.
	 * @param v 		Vertex in @a s .
	 * @param n 		Size of histogram bins (16).
	 * @param gauss 	List of Gauss curvatures of mesh vertices.
	 * @param means 	List of mean curvatures of mesh vertices.
	 * @return 			Stats -> (min Gauss, mean Gauss, max Gauss, min Mean, mean Mean, max Mean) and the feature vector of size @a n * 16 + 2.
	 */
	std::tuple<std::array<float, 6>, std::vector<float>>
		get_feature_vector(Scene& s, Vertex & v, int n, const std::vector<double>& gauss, const std::vector<double>& means);
	
	/**
	 * @brief Evaluate vertices with all models.
	 * 
	 * @param s 		Scene with the mesh.
	 * @param vertices  Some vertices in @a s .
	 * @param gauss 	List of Gauss curvatures of mesh vertices.
	 * @param means 	List of mean curvatures of mesh vertices.
	 */
	void evaluate(Scene& s, const std::vector<Vertex*>& vertices, const std::vector<double>& gauss, const std::vector<double>& means);
	
	/**
	 * @brief Evaluate neighbours of positives, just positives!
	 * 
	 * @param s 		Scene with the mesh.
	 * @param model 	Model whose positives are going to be explored.
	 * @param gauss 	List of Gauss curvatures of mesh vertices.
	 * @param means 	List of mean curvatures of mesh vertices.
	 */
	void expand_positives(Scene& s, Model& model, const std::vector<double>& gauss, const std::vector<double>& means);
	
	/**
	 * @brief Extract representatives of positives components.
	 * 
	 * @param positives 			Positives of a model.
	 * @param min_component_size 	Minimal components size, smaller are discarted.
	 * @return 						Representatives with pair of (geometric mean of component vertices probabilites, sum of component vertices probabilities).
	 */
	std::vector<std::pair<Vertex*, std::pair<double, double>>> 
		extract_central_points(const std::unordered_map<Vertex*, float>& positives, int min_component_size);

	//
	// these constants should not be changed, bacuse were used in classification learning process
	// definition of upper and lower bound of curvatures being inserted into histograms

	const double min_gauss_curv = -0.025;
	const double max_gauss_curv = 0.05;
	const double min_means_curv = -0.25;
	const double max_means_curv = 0.2;

	//
	// definition of the ranges of histograms, these are later multiplied by *unit* variable

	const double max_range = 10.0;
	const double mid_range = 6.5;
	const double min_range = 3.0;

	//
	// definition of face points distances and ratios

	const condition _e2e;
	const condition _l2l;
	const condition _n2r;
	const condition _n2e_o_n2r;
	const condition _r2e_o_e2e;
	const condition _r2l_o_n2r;
	const condition _n2l_o_l2l;

	// millimetre to unit correspondence (the number equal to 1 mm in reality)
	const double _unit;
	// classifier models
	std::unordered_map<std::string, Model> _models;

	// detected landmarks memory, is updated by update_face_landmarks calls 
	return_value _result;
	Vector3d _left_eye, _right_eye, _left_lips, _right_lips, _nose, _root;

	//
	// these constants define histograms which are ignored in order to skip its inference, feel free to remove

	const double mean_curv_center_tolerance = 0.05;
	const double gauss_curv_center_tolerance = 0.05;
	const double mean_curv_range_tolerance = 0.1;
	const double gauss_curv_range_tolerance = 0.3;

	//
	// we evaluate vertices in batches, tf supports parallel instructions and
	// smart inference, so it is much faster then evaluating each vertex separately
	int _max_threads;
	long long _batch_size;
	tensorflow::Tensor _batch;
	std::vector<Vertex*> _batch_load;
};


#endif //FACECLEANER_POINTCLASSIFIER_H