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

#include "Vertex.h"
#include "Face.h"

void Vertex::compute_normal() {
	// algorithm by Nelson Max, see https://pdfs.semanticscholar.org/9146/5dff3c4beecc70bad31442925aa3abe35061.pdf
	Vector3d new_normal;
	for (auto&& corner : _corners) {
		Face* t = corner->f;
		auto v1 = t->get_corner(0).v->pos();
		auto v2 = t->get_corner(1).v->pos();
		auto v3 = t->get_corner(2).v->pos();
		auto a = v1 - v2;
		auto b = v2 - v3;
		auto c = v3 - v1;
		double la = a.length_squared();
		double lb = b.length_squared();
		double lc = c.length_squared();
		if (la == 0.0 || lb == 0.0 || lc == 0.0) return;
		auto n = Vector3d::cross(a, b);
		if (corner->idx == 0) new_normal += n / (la * lc);
		if (corner->idx == 1) new_normal += n / (lb * la);
		if (corner->idx == 2) new_normal += n / (lc * lb);
	}
	_normal = new_normal.normalized();
}

bool Vertex::get_boundary_neighbors(Vertex *& next, Vertex *& previous) const {
	next = previous = nullptr;
	if (_corners.empty()) return false;
	for (auto&& c : _corners) {
		if (c->get_previous().get_opposite().empty()) previous = c->get_next().v;
		if (c->get_next().get_opposite().empty()) next = c->get_previous().v;
	}
	return (next != nullptr) && (previous != nullptr);
}
