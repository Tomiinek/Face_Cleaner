//
// Created by Tomiinek on 7. 2. 2018.
//

#include "Curvature.h"
#include "Scene.h"
#include "Vertex.h"

void Curvature::prepare_curvatures(Scene& scene, double scale) {

	// implementation of http://gfx.cs.princeton.edu/pubs/_2004_ECA/curvpaper.pdf
	// see also trimesh2 library by Szymon Rusinkiewicz: http://gfx.cs.princeton.edu/proj/trimesh2/

	if (scene.vertices_num() == 0 || scene.triangles_num() == 0) return;

	// setup initial coordinate system for every vertex
	Vertex* v1, *v2, *v3;	
	for (size_t i = 0; i < scene.triangles_num(); i++) {
		scene.get_triangle_vertices(scene.get_face(i), v1, v2, v3);
		v1->curv().dir_max = v2->pos() - v1->pos();
		v2->curv().dir_max = v3->pos() - v2->pos();
		v3->curv().dir_max = v1->pos() - v3->pos();
	}
	for (size_t i = 0; i < scene.vertices_num(); i++) {
		auto& v = scene.get_vertex(i);
		auto& c = v.curv();
		c.dir_max = Vector3d::cross(c.dir_max, v.normal());
		c.dir_max.normalize();
		c.dir_min = Vector3d::cross(v.normal(), c.dir_max);
	}
	// compute the Second fundamental form (SFF) for each vertex
	for (size_t i = 0; i < scene.triangles_num(); i++) {

		// first, prepare the SFF for a triangle
		auto& t = scene.get_face(i);
		scene.get_triangle_vertices(t, v1, v2, v3);
		Vector3d e[3] = {
			(v3->pos() - v2->pos()) / scale,
			(v1->pos() - v3->pos()) / scale,
			(v2->pos() - v1->pos()) / scale
		};
		Vertex* tv[3] = {v1, v2, v3};
		auto en = e[0].normalized();
		auto n = Vector3d::cross(e[0], e[1]);
		auto b = Vector3d::cross(n, en).normalized();
		double m[3] = {};
		double w[3][3] = {0};

		for (int j = 0; j < 3; j++) {
			auto u = Vector3d::dot(e[j], en);
			auto v = Vector3d::dot(e[j], b);
			w[0][0] += u*u;
			w[0][1] += u*v;
			w[2][2] += v*v;
			Vector3d dn = tv[(j + 2) % 3]->normal() - tv[(j + 1) % 3]->normal();
			double dnu = Vector3d::dot(dn, en);
			double dnv = Vector3d::dot(dn, b);
			m[0] += dnu* u;
			m[1] += dnu* v + dnv* u;
			m[2] += dnv* v;
		}
		w[1][1] = w[0][0] + w[2][2];
		w[1][2] = w[0][1];

		double diag[3];
		// decomposition and solution from trimesh2, can be used for example
		// the eigen's implementation: https://eigen.tuxfamily.org/dox/classEigen_1_1LDLT.html
		if (!MathSupport::ldlt_decompose<double, 3>(w, diag)) return;
		MathSupport::ldlt_solve<double, 3>(w, diag, m, m);

		// second, update SFF of adjacent vertices, there must be done a projection into 
		// the coordinate system of a particular vertex
		for (int i = 0; i < 3; i++) {
			auto& curv = tv[i]->curv();
			double c1, c12, c2;
			Curvature::project(en, b, m[0], m[1], m[2], curv.dir_max, curv.dir_min, c1, c12, c2);
			double wt = t.get_corner((size_t) i).get_area() / tv[i]->get_voroni_area();
			curv.max += wt * c1;
			curv.min_max += wt * c12;
			curv.min += wt * c2;
		}
	}
	// get principal curvatures for each vertex by diagonalization of vertex SFF
	for (size_t i = 0; i < scene.vertices_num(); i++) {
		auto& v = scene.get_vertex(i);
		auto& c = v.curv();
		Curvature::diagonalize(c, v.normal(), c);
	}
}

void Curvature::smooth_curvatures(double sigma, Scene& scene) {

	// update sigma in order to avoid dependency on mesh density 
	sigma /= scene.mean_edge_length();
#pragma omp parallel for
	for (int i = 0; i < (int) scene.vertices_num(); ++i) {
		auto& v = scene.get_vertex(i);
		// smooth SFFs by Gaussian function
		auto c_flt = diffuse_vertex_field(v, 1.0 / (sigma * sigma));
		auto& c = v.curv();
		auto& sc = v.curv_smooth();
		// get principal curvatures by diagonalization of SFF
		Curvature::diagonalize(Curvature{c_flt.x, c_flt.y, c_flt.z, c.dir_max, c.dir_min}, v.normal(), sc);
	}
}

void Curvature::rotate_coordinates(Vector3d & u, Vector3d & v, const Vector3d & norm) {

	auto old_norm = Vector3d::cross(u, v);
	double n_dot = Vector3d::dot(old_norm, norm);
	if (n_dot <= -1.0f) {
		u *= -1;
		v *= -1;
		return;
	}
	auto perp_old_norm = norm - n_dot * old_norm;
	auto d_perp = 1.0 / (1 + n_dot) * (old_norm + norm);
	u -= d_perp * Vector3d::dot(u, perp_old_norm);
	v -= d_perp * Vector3d::dot(v, perp_old_norm);
}

void Curvature::diagonalize(Curvature old, const Vector3d & normal, Curvature & result) {

	// make both coordinate systems coplanar
	rotate_coordinates(old.dir_max, old.dir_min, normal);

	//
	// rotate the coordinate system in order to diagonalize the Second fundamental form

	double c = 1, s = 0, t = 0;
	if (std::abs(old.min_max) > 0.00001) {
		double h = 0.5 * (old.min - old.max) / old.min_max;
		t = h < 0.0 ? 1.0 / (h - std::sqrt(1.0 + h * h)) : 1.0 / (h + std::sqrt(1.0f + h * h));
		c = 1.0 / std::sqrt(1.0f + t * t);
		s = t * c;
	}

	result.max = old.max - t * old.min_max;
	result.min = old.min + t * old.min_max;

	if (std::abs(result.max) >= std::abs(result.min)) {
		result.dir_max = c * old.dir_max - s * old.dir_min;
	}
	else {
		double tmp = result.max;
		result.max = result.min;
		result.min = tmp;
		result.dir_max = s * old.dir_max + c * old.dir_min;
	}

	result.dir_min = Vector3d::cross(normal, result.dir_max);
}

void Curvature::project(Vector3d old_u, Vector3d old_v, double old_ku, double old_kuv, double old_kv, 
						Vector3d new_u, Vector3d new_v, double & r1, double & r2, double & r3) {

	rotate_coordinates(new_u, new_v, Vector3d::cross(old_u, old_v));
	double u1 = Vector3d::dot(new_u, old_u);
	double v1 = Vector3d::dot(new_u, old_v);
	double u2 = Vector3d::dot(new_v, old_u);
	double v2 = Vector3d::dot(new_v, old_v);
	r1 = old_ku * u1 * u1 + old_kuv * (2.0 * u1 * v1) + old_kv * v1 * v1;
	r2 = old_ku * u1 * u2 + old_kuv * (u1 * v2 + u2 * v1) + old_kv * v1 * v2;
	r3 = old_ku * u2 * u2 + old_kuv * (2.0 * u2 * v2) + old_kv * v2 * v2;
}

Vector3d Curvature::diffuse_vertex_field(Vertex& v, double sigma) {

	Vector3d flt;

	// no neigbhours, this shoudld probably never happen as we remove minor components
	auto& neighbors = v.get_one_ring();
	if (neighbors.empty()) {
		double c1, c12, c2;
		auto& c = v.curv();
		Curvature::project(c.dir_max, c.dir_min, c.max, 0, c.min, c.dir_max, c.dir_min, c1, c12, c2);
		return Vector3d(c1, c12, c2);
	}

	double area = v.get_voroni_area();
	double sum_w = area;
	auto nv = v.normal();

	// initial setup for the vertex
	double c1, c12, c2;
	auto& c = v.curv();
	Curvature::project(c.dir_max, c.dir_min, c.max, 0, c.min, c.dir_max, c.dir_min, c1, c12, c2);
	flt += area * Vector3d(c1, c12, c2);

	// actual smoothing, BFS
	std::unordered_set<Vertex*> flags = {&v};
	std::stack<Vertex*, std::vector<Vertex*>> boundary(neighbors);
	while (!boundary.empty()) {

		auto& n = boundary.top();
		boundary.pop();
		if (flags.find(n) != flags.end()) continue;
		flags.insert(n);

		if (Vector3d::dot(nv, n->normal()) <= 0) continue;
		double w = MathSupport::gaussian_weight((n->pos() - v.pos()).length(), sigma);
		if (w <= 0.000001) continue;
		w *= Vector3d::dot(nv, n->normal());
		w *= n->get_voroni_area();

		double nc1, nc12, nc2;
		auto& nc = n->curv();
		if (!std::isnan(nc.max) && !std::isnan(nc.min_max) && !std::isnan(nc.min)) {
			Curvature::project(nc.dir_max, nc.dir_min, nc.max, 0, nc.min, c.dir_max, c.dir_min, nc1, nc12, nc2);
			flt += w * Vector3d(nc1, nc12, nc2);
			sum_w += w;
		}

		auto& n_neighbors = n->get_one_ring();
		if (n_neighbors.empty()) continue;
		for (auto&& nn : n_neighbors) {
			if (flags.find(nn) != flags.end()) continue;
			boundary.push(nn);
		}
	}

	// normalize the Curvature tensor by weights
	flt /= sum_w;
	return flt;
}