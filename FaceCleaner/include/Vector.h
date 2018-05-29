#ifndef FACECLEANER_VECTOR_H
#define FACECLEANER_VECTOR_H

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
 * @brief   A file which groups classes handling vector types and vector operations.
 *
 * @file    Vector.h
 * @author  Tomas Nekvinda <tom(at)neqindi.cz>
 */

#include <cmath>
#include <ostream>

template<typename T>
struct comparison_traits
{
	static bool equal(const T& a, const T& b) { return a == b; }
};

template<>
struct comparison_traits<double>
{
	static bool equal(const double& a, const double& b) { return std::fabs(a - b) < 1e-15; }
};

template<>
struct comparison_traits<float>
{
	static bool equal(const float& a, const float& b) { return std::fabs(a - b) < 1e-15; }
};

template<typename T>
class Vector;
template<typename T>
class Vector2;

// assign names for the most used types of vectors
using Vector3f = Vector<float>;
using Vector3d = Vector<double>;
using Vector2d = Vector2<double>;

/**
 * @brief Two dimensional vector representation. Just few operations are implemented. 
 * 
 * @tparam T Numeric type of the vector elements.  
 */
template<typename T>
class Vector2
{
public:
	Vector2() : x(T()), y(T()) {}
	Vector2(T x, T y) : x(x), y(y) {}

	friend const Vector2<T> operator*(const Vector2<T> &v, T scalar) {
		return Vector2<T>(v.x * scalar, v.y * scalar);
	}

	const Vector2<T> operator+(const Vector2<T> &v) const {
		return Vector2<T>(x + v.x, y + v.y);
	}

	const Vector2<T> operator/(T scalar) const {
		return (*this) * (1 / scalar);
	}

	Vector2<T> &operator+=(const Vector2<T> &v) {
		x += v.x;
		y += v.y;
		return *this;
	}

	T x;
	T y;
};

/**
 * @brief Three dimensional vector representation.
 * 
 * @tparam T 
 */
template<typename T>
class Vector
{
public:
	Vector() : x(T()), y(T()), z(T()) {}
	Vector(T x, T y, T z) : x(x), y(y), z(z) {}

	friend std::ostream& operator<<(std::ostream &os, const Vector<T> &v) {
		return os << '(' << v.x << ',' << v.y << ',' << v.z << ')';
	}

	bool operator==(const Vector<T> &v) const {
		return comparison_traits<T>::equal(x, v.x) &&
		       comparison_traits<T>::equal(y, v.y) &&
		       comparison_traits<T>::equal(z, v.z);
	}

	bool operator!=(const Vector<T> &v) const {
		return !((*this) == v);
	}

	friend const Vector<T> operator*(const Vector<T> &v, T scalar) {
		return Vector<T>(v.x * scalar, v.y * scalar, v.z * scalar);
	}

	friend const Vector<T> operator*(T scalar, const Vector<T> &v) {
		return Vector<T>(v.x * scalar, v.y * scalar, v.z * scalar);
	}

	const Vector<T> operator+(const Vector<T> &v) const {
		return Vector<T>(x + v.x, y + v.y, z + v.z);
	}

	const Vector<T> operator-(const Vector<T> &v) const {
		return Vector<T>(x - v.x, y - v.y, z - v.z);
	}

	const Vector<T> operator/(T scalar) const {
		return (*this) * (1 / scalar);
	}

	/**
	 * @brief The dot product of two vectors
	 * 
	 * @param v1 First vector. 
	 * @param v2 Second vector. 
	 * @return 	 Dot product: v1 . v2 
	 */
	static T dot(const Vector<T>& v1, const Vector<T>& v2) {
		return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	}

	/**
	 * @brief The cross product of two vectors.
	 * 
	 * @param v1 First vector. 
	 * @param v2 Second vector. 
	 * @return   Vector which is the cross product of two vectors: v1 x v2.
	 */
	static Vector<T> cross(const Vector<T> &v1, const Vector<T>& v2) {
		return Vector<T>(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
	}

	/**
	 * @brief Element-wise min function. 
	 * 
	 * @param v1 First vector.
	 * @param v2 Second vector.
	 * @return   New vector whose elements are minima of corresponding elements of the two input vectors. 
	 */
	static Vector<T> min(const Vector<T>& v1, const Vector<T>& v2) {
		return Vector<T>(std::fmin(v1.x, v2.x), std::fmin(v1.y, v2.y), std::fmin(v1.z, v2.z));
	}

	/**
	 * @brief Element-wise max function. 
	 * 
	 * @param v1 First vector.
	 * @param v2 Second vector.
	 * @return 	 New vector whose elements are maxima of corresponding elements of the two input vectors. 
	 */
	static Vector<T> max(const Vector<T>& v1, const Vector<T>& v2) {
		return Vector<T>(std::fmax(v1.x, v2.x), std::fmax(v1.y, v2.y), std::fmax(v1.z, v2.z));
	}

	/**
	 * @brief Distance between two points defined by vectors.
	 * 
	 * @param v1 First point.
	 * @param v2 Second point. 
	 * @return   Distance the between first and the second vector. 
	 */
	static T distance(const Vector<T>& v1, const Vector<T>& v2) {
		Vector<T> distance = v1 - v2;
		return distance.length();
	}

	/**
	 * @brief Squared distance between two points defined by vectors.
	 * 
	 * @param v1 First point.
	 * @param v2 Second point. 
	 * @return   Squared distance the between first and the second vector. 
	 */
	static T distance_squared(const Vector<T>& v1, const Vector<T>& v2) {
		Vector<T> distance = v1 - v2;
		return distance.length_squared();
	}

	Vector<T> &operator=(const Vector<T> &v) {
		x = v.x;
		y = v.y;
		z = v.z;
		return *this;
	}

	Vector<T> &operator+=(const Vector<T> &v) {
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}

	Vector<T> &operator-=(const Vector<T> &v) {
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}

	Vector<T> &operator*=(T scalar) {
		x *= scalar;
		y *= scalar;
		z *= scalar;
		return *this;
	}

	Vector<T> &operator/=(T scalar) {
		scalar = 1 / scalar;
		x *= scalar;
		y *= scalar;
		z *= scalar;
		return *this;
	}

	Vector<T> &operator-() {
		x = -x;
		y = -y;
		z = -z;
		return *this;
	}

	T& operator[](int i) {
		if (i == 0) return x;
		else if (i == 1) return y;
		else if (i == 2) return z;
		else throw std::out_of_range("vector: index out of range!");
	}

	double length_squared() const {
		return x * x + y * y + z * z;
	}

	double length() const {
		return std::sqrt(length_squared());
	}

	void normalize() {
		auto magnitude = 1.0 / length();
		x *= magnitude;
		y *= magnitude;
		z *= magnitude;
	}

	Vector<T> normalized() const {
		Vector<T> tmp = *this;
		tmp.normalize();
		return tmp;
	}

	T x;
	T y;
	T z;
};

#endif //FACECLEANER_VECTOR_H