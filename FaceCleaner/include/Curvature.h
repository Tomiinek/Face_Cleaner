#ifndef FACECLEANER_CURVATURE_H
#define FACECLEANER_CURVATURE_H

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
 * @brief   { Object which can hold curvature information of a single vertex. 
 * 			There are also present some methods for curvature computation. } 
 *
 * @file    Curvature.h
 * @author  Tomas Nekvinda <tom(at)neqindi.cz>
 */

#include <cmath>
#include "Vector.h"

class Scene;
class Vertex;

class Curvature
{
// implementation of http://gfx.cs.princeton.edu/pubs/_2004_ECA/curvpaper.pdf
// see also http://gfx.cs.princeton.edu/proj/trimesh2/ Szymon Rusinkiewicz with 
// the reference implementation (under GNU General Public License, Version 2, 
// Copyright (c) Szymon Rusinkiewicz)

public:

	/**
	 * @brief Compute curvatures for each vertex of a mesh.
	 * 
	 * @param scene  Input mesh.
	 * @param scale  The mesh unit-to-millimeter correspondence.
	 */
	static void prepare_curvatures(Scene& scene, double scale);

	/**
	 * @brief Smooth curvatures for each vertex of a mesh. Curvatures must be already prepared!
	 * 
	 * @param sigma	 Smooth radius, should depend on mesh unit-to-millimeter correspondence.
	 * @param scene  Input mesh.	
	 */
	static void smooth_curvatures(double sigma, Scene& scene);

	// maximal principal curvature, can be something else during computation
	double max;
	// the right-to-left diagonal of the second fundamental form
	double min_max;
	// minimal principal curvature, can be something else during computation
	double min;
	// direction of the principal direction of maximal curvature
	Vector3d dir_max;
	// dricetion of the proncipal direction of minimal curvature
	Vector3d dir_min;

private:
	 /**
	 * @brief Calculate the rotation of an orthonormal coordinates into a new coordinate system.
	 * 
	 * @param u 	First coordinate (unit vector).
	 * @param v 	Second coordinate (unit vector).
	 * @param norm  Axis of the new coordinate system which should be perpendicular to @a u and @a v .
	 */
	static void rotate_coordinates(Vector3d& u, Vector3d& v, const Vector3d& norm);

	/**
	* @brief Diagonalize the Second Fundamental Form, should be used before obtaining principal curvatures.
	*
	* @param old	 An arbitrary curvature.
	* @param normal	 Normal of the coordinate system of @a old .
	* @param result	 The resulting diagonalized curvature.
	*/
	static void diagonalize(Curvature old, const Vector3d& normal, Curvature& result);

	/**
	* @brief Project the Second fundamental form into a new coordinate system.
	*
	* @param old_u		First axis of the old coordinate system.
	* @param old_v		Second axis of the old coordinate system.
	* @param old_ku		Old tensor upper left.
	* @param old_kuv	Old tensor minor diagonal.
	* @param old_kv		Old tensor lower right.
	* @param new_u		First axis of the new coordinate system.
	* @param new_v		Second axis of the new coordinate system.
	* @param r1			New tensor upper left.
	* @param r2			New tensor minor diagonal.
	* @param r3			New tensor lower right.
	*/
	static void project(Vector3d old_u, Vector3d old_v,
						double old_ku, double old_kuv, double old_kv,
						Vector3d new_u, Vector3d new_v,
						double& r1, double& r2, double& r3);

	/**
	 * @brief Smooths curvatures in a vertex local neighbourhood. Supposes diagonalized curvatures in all mesh vertices.
	 * 
	 * @param v 		Vertex whose smooth curvatures should be calculated.
	 * @param sigma 	Smoothing radius, parameter of Gaussian function.
	 * @return 			Second fundamental form of the smoothed curvature (it is symmetric, so vector3 is sufficient).
	 */
	static Vector3d diffuse_vertex_field(Vertex& v, double sigma);
};


#endif //FACECLEANER_CURVATURE_H
