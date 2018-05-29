#ifndef FACECLEANER_CLEANER_H
#define FACECLEANER_CLEANER_H

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
* @brief   The most important class. Defines whole algorithm workflow and groups some basic cleaning methods.
*
* @file    Cleaner.h
* @author  Tomas Nekvinda <tom(at)neqindi.cz>
*/

#include <unordered_set>
#include <vector>
#include <stack>
#include <list>
#include <deque>

#include "Scene.h"
#include "Vertex.h"
#include "HoleFiller.h"
#include "MathSupport.h"
#include "Logger.h"
#include "PointClassifier.h"

#include <boost/program_options.hpp>

class Cleaner
{
public:

	/**
	 * @brief Construct a new Cleaner object
	 * 
	 * @param hf  HoleFiller instance, must be properly initialized.
	 * @param pc  PointClassifier instance, must be properly initialized.
	 * @param vm  Boost Program Options with needed configuration parameters.
	 */
	Cleaner(HoleFiller* hf, PointClassifier* pc, const boost::program_options::variables_map& vm);
	
	/**
	 * @brief Clean a mesh. The entry point of our cleaning algorithm.
	 * 
	 * @param s  Scene with the mesh to be cleaned.
	 */
	void process(Scene& s);

private:

	/**
	 * @brief Remove edges with more than two adjacent triangles.
	 * 
	 * @param s 		Scene with the mesh to be fixed.
	 * @return 		   Number of vertices removed.
	 */
	size_t remove_multiple_face_edges(Scene& s);

	/**
	 * @brief Remove non-2-manifold vertices.
	 * 
	 * @param s 		Scene with the mesh to be fixed.
	 * @return 		 	Number of vertices removed.
	 */
	size_t remove_non_manifold_vertices(Scene& s);

	/**
	 * @brief Remove all minor connectivity components. Preserve the largest one.
	 * 
	 * @param s 		Scene with the mesh to be fixed.
	 * @return 		 	Number of vertices removed.
	 */
	size_t remove_minor_components(Scene& s);

	/**
	* @brief Identify vertices of intersecting triangles.
	*
	* @param s 			Scene with the mesh to be fixed.
	* @param to_remove 	Container of vertices of intersecting triangles.
	*/
	void remove_intersecting_triangles(Scene & s, std::unordered_set<Vertex*> & to_remove);

	/**
	 * @brief Trim the facial are of a mesh.
	 * 
	 * @param s 			Scene with the mesh to be fixed.
	 * @param nose 			Position of the nose tip.
	 * @param root 			Position of the nose root (can be dummy if _crop_by_noraml == false).
	 * @param sight_dir 	Sight direction w. r. to the face (can be dummy if _crop_by_noraml == false).
	 * @param vertical_dir  Vertical direction w. r. to the face (can be dummy if _crop_by_noraml == false).
	 * @param face_center   Position of the nose root (can be dummy if _crop_by_noraml == false).
	 * @param points_valid  @c true if @a root , @a sight_dir , ... are valid
	 * @return 		 		Number of vertices removed.
	 */
	size_t crop_face(Scene& s, const Vector3d& nose, const Vector3d& root, const Vector3d& sight_dir,
					 const Vector3d& vertical_dir, const Vector3d& face_center, bool points_valid);
	
	/**
	 * @brief Perform the check by criteria based on face directions.
	 * 
	 * @param v 			Vertex to be checked.
	 * @param dir_forward   Sight direction w. r. to the face.
	 * @param dir_vertical  Vertical direction w. r. to the face.
	 * @param center 		Center of the face.
	 * @return				@c true if the vertex @a v should be removed, @c false otherwise.
	 */
	bool is_outgoing(const Vertex& v, const Vector3d& dir_forward, const Vector3d& dir_vertical, const Vector3d& center);
	
	/**
	 * @brief Remove protrusions of the largest mesh boundary.
	 * 
	 * @param s 		Scene with the mesh to be fixed.
	 * @return 		 	Number of removed vertices.
	 */
	size_t remove_boundary_protrusions(Scene& s);

	/**
	 * @brief Remove spikes and blobs of a mesh.
	 * 
	 * @param s 		Scene with the mesh to be fixed.
	 * @return 		 	Number of removed vertices.
	 */
	size_t remove_defects(Scene& s);

	/**
	 * @brief Split mesh triangles into partitions in order to find triangle intersections faster.
	 * 
	 * @param s 	Scene with the mesh to be fixed.
	 * @param n 	Desired size of partitions.
	 * @return 		Triangle partitions, one triangle can be present in multiple partitions!
	 */
	std::vector<std::vector<Face*>> partition_triangles(const Scene& s, int n);

	/**
	 * @brief Get all intersecting triangles in a mesh.
	 * 
	 * @param faces 	Mesh triangles (usually partition returned by partition_triangles method).
	 * @param s 		Scene with the mesh.
	 * @return 			Intersecting triangles.
	 */
	std::unordered_set<Face*> get_intersecting_faces(const std::vector<Face*> faces, const Scene& s);

	/**
	 * @brief Resolve thin-shaped triangles of a mesh.
	 * 
	 * @param s 				Scene with the mesh to be fixed.
	 * @param triangle_queue 	Queue of triangles to be check and possibly fixed.
	 * @return 		 			Number of removed vertices.
	 **/
	size_t remove_thin_triangles(Scene& s, std::deque<Face*>& triangle_queue);

	/**
	 * @brief Rotate a mesh to a uniform orientation based on face directions.
	 * 
	 * @param s 			Scene with the mesh to be rotated.
	 * @param sight_dir 	Sight direction w. r. to the face.
	 * @param vertical_dir  Vertical direction w. r. to the face.
	 * @param face_center 	Center of the face.
	 */
	void rotate(Scene &s, const Vector3d& sight_dir, const Vector3d& vertical_dir, const Vector3d& face_center);

	//
	// PROCESS SETTINGS:
	//

	// millimetre to unit correspondence (the number equal to 1 mm in reality)
	const double _unit;

	// this is adaptive due to division by mean edge length
	const double _smooth_curvature_sigma = 1.0;

	//
	// workflow definition

	const bool _crop;
	const bool _crop_by_normal;
	const bool _fill_holes;
	const bool _remove_defects;
	const bool _remove_thin_triangles;
	const bool _rotate;
	const bool _landmarks;

	//
	// paremeters for cropping

	const double _curvedness_tolerance;
	const double _near;
	const double _far;

	const double _n2ch_over_n2r_mean;
	const double _n2ch_over_n2r_dev;
	
	const double _neck_normal_threshold;   // pi * 5 / 12  =  75 deg
	const double _ears_normal_threshold;   // pi * 25 / 36 = 125 deg;
	const double _neck_angle_threshold;    // pi / 18      =  10 deg
	const double _ears_angle_threshold;    // pi * 5 / 18  =  50 deg;

	const double _wide_protrusion_range;   // 6 mm neighborhood
	const double _wide_protrusion_angle;   // pi * 3 / 2   = 270 deg
	const double _tiny_protrusion_angle;   // pi * 2 / 5   =  72 deg

	//
	// parameters for cleaning

	const double _maximal_angle;	// 250 deg
	const double _minimal_angle;	// 110 deg

	const double _max_mean_curv;
	const double _min_mean_curv;
	const double _max_gauss_curv;
	const double _min_gauss_curv;
	const double _max_mean_smooth;
	const double _min_mean_smooth;
	const double _max_gauss_smooth;
	const double _min_gauss_smooth;

	// other parameters	
	const double _min_triangle_aspect_ratio;

	// instances of used helper classes
	HoleFiller* _hf;
	PointClassifier* _pc;
};


#endif //FACECLEANER_CLEANER_H
