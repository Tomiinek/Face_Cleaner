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
 * @file    Corner.cpp
 */

#include "Corner.h"
#include "Vertex.h"
#include "Face.h"

Corner::Corner(size_t id, Vertex *vertex, Face *face, std::vector<Corner*> opposites)
		: v(vertex), f(face), idx(id), o(std::move(opposites)) {
	v->add_corner(this);
	for (auto&& c : o) c->add_opposite(this);
}

const Corner & Corner::get_previous() const {
	return f->get_previous(this);
}

const Corner & Corner::get_next() const {
	return f->get_next(this);
}

double Corner::get_area() const {

	// by Szymon Rusinkiewicz for more details on Voronoi (or mixed) area computation, see:
	// http://www.geometry.caltech.edu/pubs/DMSB_III.pdf

	Vertex* v0,* v1,* v2;
	f->get_vertices(v0, v1, v2);
	Vector3d e[3] = {
		v2->pos() - v1->pos(),
		v0->pos() - v2->pos(),
		v1->pos() - v0->pos()
	};
	double area = 0.5 * Vector3d::cross(e[0], e[1]).length();
	double l_sq[3] = {e[0].length_squared(), e[1].length_squared(), e[2].length_squared()};

	// l_sq(a) + l_sq(b) >< l_sq(c)
	double e_w[3] = { l_sq[0] * (l_sq[1] + l_sq[2] - l_sq[0]),
					  l_sq[1] * (l_sq[2] + l_sq[0] - l_sq[1]),
					  l_sq[2] * (l_sq[0] + l_sq[1] - l_sq[2])};

	// cot Q = dot(PQ, RQ) / (l(PQ) * l(RQ))  / l(cross(PQ, RQ)) / (l(PQ) * l(RQ))
	//	     = dot(PQ, RQ) * l(cross(PQ, RQ)) / (l_sq(PQ) * l_sq(RQ))
	// area  = 1/8 * ( l_sq(PR) * dot(PQ, RQ) * l(cross(PQ, RQ)) / (l_sq(PQ) * l_sq(RQ)) +
	//		           l_sq(PQ) * dot(PR, RQ) * l(cross(PR, RQ)) / (l_sq(PR) * l_sq(RQ)) )

	if (e_w[0] <= 0.0) {
		if (idx != 0) return -0.25 * l_sq[3 - idx] * area / Vector3d::dot(e[0], e[3 - idx]);
		else return area * (1 + 0.25 * (l_sq[2] / Vector3d::dot(e[0], e[2]) + l_sq[1] / Vector3d::dot(e[0], e[1])));
	}
	else if (e_w[1] <= 0.0) {
		if (idx != 1) return -0.25 * l_sq[2 - idx] * area / Vector3d::dot(e[1], e[2 - idx]);
		else return area * (1 + 0.25 * (l_sq[0] / Vector3d::dot(e[1], e[0]) + l_sq[2] / Vector3d::dot(e[1], e[2])));
	}
	else if (e_w[2] <= 0.0) {
		if (idx != 2) return -0.25 * l_sq[1 - idx] * area / Vector3d::dot(e[2], e[1 - idx]);
		else return area * (1 + 0.25 * (l_sq[1] / Vector3d::dot(e[2], e[1]) + l_sq[0] / Vector3d::dot(e[2], e[0])));
	}
	else {
		return 0.5 * area / (e_w[0] + e_w[1] + e_w[2]) * (e_w[(idx + 1) % 3] + e_w[(idx + 2) % 3]);
	}
}
