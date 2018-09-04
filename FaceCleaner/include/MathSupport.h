#ifndef FACECLEANER_MATH_H
#define FACECLEANER_MATH_H

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
 * @brief  A namespace which groups mathematical and geometrical routines.
 * 
 * @file   MathSupport.h
 * @author Tomas Nekvinda <tom(at)neqindi.cz>
 */

#include <array>
#include <cmath>
#include <vector>
#include "Vector.h"

/**
 * @brief The definition of the number Pi.
 */
#define PI 3.14159265358979323846

namespace MathSupport
{
	/**
	 * @brief A positive modulo operation.
	 * 
	 * @param i Divident.
	 * @param n Divisor.
	 * @return int A positive integer r so that @a i = q * @a n + r, r < n.
	 */
	inline int pos_mod(int i, int n) { return (i % n + n) % n; }

	/**
	 * @brief LDLT decomposition A=LDL^T of a symmetric matrix which is not positive definite.
	 * 
	 * @tparam T 		   Type of matrix elements.
	 * @tparam N 		   Number of matrix rows.
	 * @param A[in,out] rw Input matrix A, it is replaced with the triangular matrix L.
	 * @param rdiag 	   Diagonal matrix D.
	 * @return 			   Return true if the decomposition was successful, false otherwise.
	 */
	template <class T, int N>
	bool ldlt_decompose(T A[N][N], T rdiag[N]){
		// comes from trimesh2 GNU library, see http://gfx.cs.princeton.edu/proj/trimesh2/
		T v[N-1];
		for (int i = 0; i < N; i++) {
			for (int k = 0; k < i; k++)
				v[k] = A[i][k] * rdiag[k];
			for (int j = i; j < N; j++) {
				T sum = A[i][j];
				for (int k = 0; k < i; k++)
					sum -= v[k] * A[j][k];
				if (i == j) {
					if (sum <= T(0)) return false;
					rdiag[i] = T(1) / sum;
				} else {
					A[j][i] = sum;
				}
			}
		}
		return true;
	}

	/**
	 * @brief Solution of (LDL^T)x=B equation.
	 * 
	 * @tparam T 	Type of matrix elements.
	 * @tparam N 	Number of matrix rows.
	 * @param A 	Triangular matrix L.
	 * @param rdiag Diagonal matrix D.
	 * @param B 	Target vector.
	 * @param x 	Solution.
	 */
	template <class T, int N>
	void ldlt_solve(T A[N][N], T rdiag[N], T B[N], T x[N]) {
		// comes from trimesh2 GNU library, see http://gfx.cs.princeton.edu/proj/trimesh2/
		for (int i = 0; i < N; i++) {
			T sum = B[i];
			for (int k = 0; k < i; k++)
				sum -= A[i][k] * x[k];
			x[i] = sum * rdiag[i];
		}
		for (int i = N - 1; i >= 0; i--) {
			T sum = 0;
			for (int k = i + 1; k < N; k++)
				sum += A[k][i] * x[k];
			x[i] -= sum * rdiag[i];
		}
	}

	/**
	 * @brief Computes Gauss function.
	 * 
	 * @param d 	   Function argument, usualy distance.
	 * @param sigma    Gauss function control parameter.
	 * @return double  Gives the solution of e^(- @a d ^2 * @a sigma / 2)
	 */
	inline double gaussian_weight(double d, double sigma) {
		double d2 = sigma * d* d;
		return (float)((d2 >= 9.0f) ? 0.0f : std::exp(-0.5 * d2));
	}

	/**
	 * @brief Computes the angle between two vectors.
	 * 
	 * @tparam T 		Numeric type of the vectors.
	 * @param u 		First vector.
	 * @param v 		Second vector.
	 * @return double 	Magnitude of the angle between @a u and @a v in range [0 rad, pi rad].
	 */
	template <typename T>
	double angle_between(const Vector<T>& u, const Vector<T>& v){
		double d = Vector<T>::dot(u.normalized(), v.normalized());
		if (d > 1) d = 1.0;
		else if (d < -1) d = -1.0;
		return std::acos(d);
	}

	/**
	 * @brief Computes the angle between two vectors with respect to a normal vector.
	 * 
	 * @tparam T 		Numeric type of the vectors.
	 * @param u 		First vector.
	 * @param v 		Second vector.
	 * @param n 		Normal vector, does not have to be perpendicular to u and v.
	 * @return double 	Magnitude of the angle between @a u and @a v in range [0 rad, 2*pi rad].
	 */
	template <typename T>
	double angle_between(const Vector<T>& u, const Vector<T>& v, const Vector<T>& n){
		double a = angle_between(u, v);
		auto cross = Vector<T>::cross(u, v);
		if (Vector<T>::dot(n, cross) < 0) a = 2.0 * PI - a;
		return a;
	}

	/**
	 * @brief Computes the dihedral angle between two triangles.
	 * 
	 * @tparam T 		Numeric type of vectors of triangle points.
	 * @param v1 		First vertex which is not shared by the two triangles.
	 * @param v2 		Second vertex which is not shared by the two triangles.
	 * @param v3 		First vertex shared by the two triangles.
	 * @param v4 		Second vertex shared by the two triangles.
	 * @return double   The absolute value of the difference of the dihedral angle magnitude and pi.
	 */
	template <typename T>
	double dihedral_angle(const Vector<T>& v1, const Vector<T>& v2, const Vector<T>& v3, const Vector<T>& v4) {
		auto n1 = Vector3d::cross(v2 - v1, v3 - v1);
		if (n1.length_squared() == 0) return PI;
		auto n2 = Vector3d::cross(v4 - v1, v2 - v1);
		return angle_between(n1, n2);
	}

	/**
	 * @brief Determines if two three-dimensional triangles intersect.
	 * 
	 * @tparam T  Numeric type of vectors of triangle points. 
	 * @param v0  First triangle vertex.	 
	 * @param v1  First triangle vertex.	
	 * @param v2  First triangle vertex.	
	 * @param u0  Second triangle vertex.
	 * @param u1  Second triangle vertex.
	 * @param u2  Second triangle vertex.
	 * @return 	  @c true if the two triangles are intersecting, @c false otherwise.
	 */
	template <typename T>
	bool triangles_intersecting(const Vector<T>& v0, const Vector<T>& v1, const Vector<T>& v2,
									   const Vector<T>& u0, const Vector<T>& u1, const Vector<T>& u2) {
		// implementation of https://www.geometrictools.com/Documentation/MethodOfSeparatingAxes.pdf.
		// inspired by source code from by David Eberly:
		//     https://www.geometrictools.com/GTEngine/Samples/Geometrics/AllPairsTriangles/TriangleIntersection.cpp
		//	   http://www.boost.org/LICENSE_1_0.txt

		Vector<T> e0[3] = { v1 - v0, v2 - v1, v0 - v2 };
		Vector<T> n0 = Vector<T>::cross(e0[0], e0[1]).normalized();
		auto n0dv0 = Vector<T>::dot(n0, v0);

		double min1, max1;
		std::tie(min1, max1) = project_onto_axis(u0, u1, u2, n0);
		// are triangles separated by perpendicular planes?
		if (n0dv0 < min1 || n0dv0 > max1) return false;

		Vector<T> e1[3] = {u1 - u0, u2 - u1, u0 - u2};
		Vector<T> n1 = Vector<T>::cross(e1[0], e1[1]).normalized();

		Vector3d dir {};
		double min0, max0;
		int i0, i1;

		Vector<T> n0xn1 = Vector<T>::cross(n0, n1);
		if (2 * std::numeric_limits<T>::epsilon() <= n0xn1.length()) n0xn1.normalize();

		// triangles are not parallel.
		if (Vector<T>::dot(n0xn1, n0xn1) >= 1e-08) {

			Vector<T> triangle0[3] = {v0, v1, v2};
			Vector<T> triangle1[3] = {u0, u1, u2};

			int positive = 0, negative = 0, zero = 0;
			int sign[3];
			double dist[3];

			// check separating planes given by triangles
			Vector<T> normal = Vector<T>::cross(v1 - v0, v2 - v0);
			for (int i = 0; i < 3; ++i) {
				dist[i] = Vector<T>::dot(normal, triangle1[i] - v0);
				if (dist[i] > 1e-08) { sign[i] = 1; ++positive; }
				else if (dist[i] < -1e-08) { sign[i] = -1; ++negative; }
				else { sign[i] = 0; ++zero; }
			}

			// triangle lays on one side of the plane, so they cannot intersect
			if (positive == 0 || negative == 0) return false;

			double t;
			int i_m, i_p;
			Vector<T> intr0, intr1;

			// no triangle vertex lays on the plane
			if (zero == 0) {
				int i_sign = (positive == 1 ? +1 : -1);
				for (int i = 0; i < 3; ++i) {
					if (sign[i] != i_sign) continue;
					i_m = (i + 2) % 3;
					i_p = (i + 1) % 3;
					t = dist[i] / (dist[i] - dist[i_m]);
					intr0 = triangle1[i] + t * (triangle1[i_m] - triangle1[i]);
					t = dist[i] / (dist[i] - dist[i_p]);
					intr1 = triangle1[i] + t * (triangle1[i_p] - triangle1[i]);
					return triangle_intersect_segment(n0, triangle0, intr0, intr1);
				}
			}

			for (int i = 0; i < 3; ++i) {
				if (sign[i] != 0) continue;
				i_m = (i + 2) % 3;
				i_p = (i + 1) % 3;
				t = dist[i_m] / (dist[i_m] - dist[i_p]);
				intr0 = triangle1[i_m] + t * (triangle1[i_p] - triangle1[i_m]);
				return triangle_intersect_segment(n0, triangle0, triangle1[i], intr0);
			}
		}
		// triangles are parallel (and, in fact, coplanar)
		else {
			// check separating axes given by edges of the first triangle
			for (i0 = 0; i0 < 3; ++i0) {
				dir = Vector<T>::cross(n0, e0[i0]);
				if (2 * std::numeric_limits<T>::epsilon() > dir.length()) dir = Vector<T>{};
				else dir.normalize();
				std::tie(min0, max0) = project_onto_axis(v0, v1, v2, dir);
				std::tie(min1, max1) = project_onto_axis(u0, u1, u2, dir);
				if (max0 - min1 < 1e-08 || max1 - min0 < 1e-08) return false;
			}
			// check separating axes given by edges of the second triangle
			for (i1 = 0; i1 < 3; ++i1) {
				dir = Vector<T>::cross(n1, e1[i1]);
				if (2 * std::numeric_limits<T>::epsilon() > dir.length()) dir = Vector<T>{};
				else dir.normalize();
				std::tie(min0, max0) = project_onto_axis(v0, v1, v2, dir);
				std::tie(min1, max1) = project_onto_axis(u0, u1, u2, dir);
				if (max0 - min1 < 1e-08 || max1 - min0 < 1e-08) return false;
			}
		}
		return true;
	}

	/**
	 * @brief Helper function for trinagle-to-triangle intersection test. Projects three points on an axis.
	 * 
	 * @tparam T 	Numeric type of vectors of points.  
	 * @param t0 	First point.
	 * @param t1 	Second point.
	 * @param t2 	Third point.
	 * @param axis  Axis to project the points on.
	 * @return 		The left most and the right most of projected points on the axis.
	 */
	template <typename T>
	std::pair<double, double> project_onto_axis(const Vector<T>& t0, const Vector<T>& t1, const Vector<T>& t2, const Vector<T>& axis) {		
		double dot0 = Vector3d::dot(axis, t0);
		double dot1 = Vector3d::dot(axis, t1);
		double dot2 = Vector3d::dot(axis, t2);
		double fmin = dot0;
		double fmax = fmin;
		if (dot1 < fmin) fmin = dot1;
		else if (dot1 > fmax) fmax = dot1;
		if (dot2 < fmin) fmin = dot2;
		else if (dot2 > fmax) fmax = dot2;
		return std::make_pair(fmin, fmax);
	}

	/**
	 * @brief Determines if a three-dimensional triangle intersects a segment.
	 * 
	 * @tparam T 		Numeric type of vectors of points.  
	 * @param normal 	Normal of the plane defined by triangle.
	 * @param triangle  Triangle points.
	 * @param end0 		Start of the segment.
	 * @param end1 		End of the segment.
	 * @return 			@c true if the traingle intersects the segment, @c false otherwise. 
	 */
	template <typename T>
	bool triangle_intersect_segment(const Vector<T>& normal, Vector<T> triangle[3], const Vector<T>& end0, const Vector<T>& end1) {
		// find the coordinate (max_normal) which we are going to remove by projection 
		int max_normal = 0;
		double fmax = std::abs(normal.x);
		double abs_max = std::abs(normal.y);
		if (abs_max > fmax) { max_normal = 1; fmax = abs_max; }
		abs_max = std::abs(normal.z);
		if (abs_max > fmax) max_normal = 2;
		// prepare variables for point projections
		Vector2<T> proj[3];
		Vector2<T> seg0 {};
		Vector2<T> seg1 {};
		// project points onto a plane
		if (max_normal == 0) {
			for (int i = 0; i < 3; ++i) { proj[i].x = triangle[i].y; proj[i].y = triangle[i].z; }
			seg0.x = end0.y; seg0.y = end0.z;
			seg1.x = end1.y; seg1.y = end1.z;
		}
		else if (max_normal == 1) {
			for (int i = 0; i < 3; ++i) { proj[i].x = triangle[i].x; proj[i].y = triangle[i].z; }
			seg0.x = end0.x; seg0.y = end0.z;
			seg1.x = end1.x; seg1.y = end1.z;
		}
		else {
			for (int i = 0; i < 3; ++i) { proj[i].x = triangle[i].x; proj[i].y = triangle[i].y; }
			seg0.x = end0.x; seg0.y = end0.y;
			seg1.x = end1.x; seg1.y = end1.y;
		}
		return segment_triangle_intersection_2d(seg0, seg1, proj[0], proj[1], proj[2]);
	}

	/**
	 * @brief Determines if a two-dimensional triangle intersects a segment.
	 * 
	 * @tparam T 	Numeric type of vectors of points.  
	 * @param p0 	Start of the segment.
	 * @param p1 	End of the segment.
	 * @param t0 	First triangle point.
	 * @param t1 	Second triangle point.
	 * @param t2  	Third triangle point.
	 * @return 		@c true if the traingle intersects the segment, @c false otherwise. 
	 */
	template <typename T>
	bool segment_triangle_intersection_2d(const Vector2<T>& p0, const Vector2<T>& p1, 
											    const Vector2<T>& t0, const Vector2<T>& t1, const Vector2<T>& t2) {
		// see https://gamedev.stackexchange.com/questions/21096/what-is-an-efficient-2d-line-segment-versus-triangle-intersection-test
		auto side = [](const Vector2<T>& p, const Vector2<T>& q, const Vector2<T>& a, const Vector2<T>& b) { 
			auto z1 = (b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y);
			auto z2 = (b.x - a.x) * (q.y - a.y) - (q.x - a.x) * (b.y - a.y);
			return z1 * z2;
		};
		double f1 = side(p0, t2, t0, t1), f2 = side(p1, t2, t0, t1);
		double f3 = side(p0, t0, t1, t2), f4 = side(p1, t0, t1, t2);
		double f5 = side(p0, t1, t2, t0), f6 = side(p1, t1, t2, t0);
		double f7 = side(t0, t1, p0, p1);
		double f8 = side(t1, t2, p0, p1);
		return !((f1 <= 0 && f2 <= 0) || (f3 <= 0 && f4 <= 0) || (f5 <= 0 && f6 <= 0) || (f7 >= 0 && f8 >= 0));
	}

	/**
	 * @brief Calculates triangle aspect ratio. Definition by P.G. Ciarlet, 1978, The finite element method for elliptic problem.
	 * 
	 * @tparam T 	Numeric type of vectors of triangle points.  
	 * @param v1 	First triangle point.
	 * @param v2 	Second triangle point.
	 * @param v3 	Third triangle point.
	 * @return 		Triangle aspect ratio. 
	 */
	template <typename T>
	double triangle_aspect_ratio(Vector<T> v1, Vector<T> v2, Vector<T> v3) {
		// for definition see also https://hal.archives-ouvertes.fr/hal-00761482/document
		constexpr double norm = 3.46410161514; // sqrt(3) * 2.0
		auto a = (v1 - v2).length();
		auto b = (v2 - v3).length();
		auto c = (v3 - v1).length();
		double max = std::max(a, std::max(b, c));
		double area = Vector<T>::cross(v2 - v1, v3 - v1).length() / 2.0;
		return norm * area / (max * (a + b + c) / 2.0);
	}

	/**
	 * @brief Check if a point is inside the circumsphere of a triangle.
	 * 
	 * @tparam T 	Numeric type of vectors of points.  
	 * @param t1 	First triangle point.
	 * @param t2 	Second triangle point.
	 * @param t3	Third triangle point.
	 * @param p 	Point to be checked.
	 * @return 		@c true if the point is inside the circumsphere, @c false otherwise. 
	 */
	template <typename T>
	bool is_point_in_triangles_circumsphere(Vector<T> t1, Vector<T> t2, Vector<T> t3, Vector<T> p) {
		// see https://gamedev.stackexchange.com/questions/60630/how-do-i-find-the-circumcenter-of-a-triangle-in-3d
		auto e1 = t3 - t1;
		auto e2 = t2 - t1;
		auto n = Vector<T>::cross(e2, e1);
		auto center = (Vector<T>::cross(n, e2) * e1.length_squared() + Vector<T>::cross(e1, n) * e2.length_squared()) / (2.0 * n.length_squared());
		double radius = center.length();
		return (p - t1 - center).length() <= radius;
	}
}

#endif //FACECLEANER_MATH_H
