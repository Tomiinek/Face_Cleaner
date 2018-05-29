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
 * @file    WavefrontObjectFiles.cpp
 */

#include "WavefrontObjectFiles.h"

bool WavefrontObjectFiles::load(std::ifstream& input, Scene& scene) {

	scene.clear();
	std::vector<std::tuple<double, double, double>> normals;
	normals.reserve(256);

	size_t num_vertices = 0;
	size_t num_triangles = 0;
	size_t num_txt_coords = 0;
	size_t num_normals = 0;

	std::vector<std::string> tokens;
	std::string header;
	double t[3];

	try {

		// get baisc info about the file and check if it is empty
		Info info;
		if (!load_info(input, info)) {
			LOG(ERR) << "Error while loading a file, it is empty";
			return false;
		}

		while (!input.eof()) {
			// split the data line into tokens
			tokens.clear();
			tokenize_next_line(input, tokens);
			size_t num_tokens = tokens.size();
			if (num_tokens == 0) continue;

			// handle the input line according to the first character of the line
			header.clear();
			header = tokens[0];

			// vertices
			if (header == "v") {

				if (num_tokens < 4) {
					LOG(ERR) << "Error reading vertex coordinates, there should be present x, y, z";
					return false;
				}

				Vector3d v;
				try {
					v.x = std::stod(tokens[1]);
					v.y = std::stod(tokens[2]);
					v.z = std::stod(tokens[3]);
				}
				catch (std::invalid_argument const&) {
					LOG(ERR) << "Error reading vertex coordinates, an invalid number format";
					return false;
				}
				catch (std::out_of_range const&) {
					LOG(ERR) << "Error reading vertex coordinates, a coordiante is out of the range of double";
					return false;
				}

				scene.add_vertex(v);
				++num_vertices;

			// vertex coordinates
			} else if (header == "vt") {

				if (num_tokens < 3) {
					LOG(ERR) << "Error reading vertex texture coordinates";
					return false;
				}

				try {
					t[0] = std::stod(tokens[1]);
					t[1] = std::stod(tokens[2]);
				}
				catch (std::invalid_argument const&) {
					LOG(ERR) << "Error reading vertex texture coordinates, an invalid number format";
					return false;
				}
				catch (std::out_of_range const&) {
					LOG(ERR) << "Error reading vertex texture coordinates, a coordiante is out of the range of double";
					return false;
				}

				scene.add_texture_coordinate(t[0], t[1]);
				++num_txt_coords;

			// vertex normals
			} else if (header == "vn") {

				if (num_tokens < 4) {
					LOG(ERR) << "Error reading vertex normal coordinates";
					return false;
				}

				try {
					t[0] = std::stod(tokens[1]);
					t[1] = std::stod(tokens[2]);
					t[2] = std::stod(tokens[3]);
				}
				catch (std::invalid_argument const&) {
					LOG(ERR) << "Error reading vertex normal coordinates, an invalid number format";
					return false;
				}
				catch (std::out_of_range const&) {
					LOG(ERR) << "Error reading vertex normal coordinates, a coordiante is out of the range of double";
					return false;
				}

				normals.push_back(std::make_tuple(t[0], t[1], t[2]));
				++num_normals;

			// faces
			} else if (header == "f") {

				if (num_tokens < 4) {
					LOG(ERR) << "Error reading face definition. Only tri-meshes are supported.\n";
					return false;
				}

				std::vector<int> v_idx(3);
				std::vector<int> n_idx(3);
				std::vector<int> t_idx(3);

				for (int j = 0; j < 3; ++j) {
					try {
						split_token(tokens[j + 1], v_idx[j], n_idx[j], t_idx[j], info);
					}
					catch (std::invalid_argument const&) {
						LOG(ERR) << "Error reading face definition, an invalid number format";
						return false;
					}
					catch (std::out_of_range const&) {
						LOG(ERR) << "Error reading face definition, a number is out of the range";
						return false;
					}

					obj_index_ok(v_idx[j], (int) num_vertices);
					obj_index_ok(t_idx[j], info.num_txt_coords);
					obj_index_ok(n_idx[j], info.num_normals);
				}

				size_t fh = scene.add_face((size_t) v_idx[0], (size_t) v_idx[1], (size_t) v_idx[2]);
				if (info.num_txt_coords > 0) scene.set_face_texture_coordinate(fh, t_idx[0], t_idx[1], t_idx[2]);
				++num_triangles;

				for (int i = 0; i < 3; ++i) {
					if (info.num_normals > 0) {
						scene.get_vertex((size_t) v_idx[i]).set_normal(
								std::get<0>(normals[n_idx[i]]),
								std::get<1>(normals[n_idx[i]]),
								std::get<2>(normals[n_idx[i]]));
					}
				}
			}
		}

		// note the presence of normals and texture coordinates somewhere
		if (info.num_normals > 0) scene.set_has_normals();
		if (info.num_txt_coords > 0) scene.set_has_textures();
		
	}
	catch (const std::exception & e) {
		LOG(ERR) << "An error occurred while reading a file, please check the file format or report this error:";
		LOG(ERR) << e.what();
		return false;
	}

	return true;
}

bool WavefrontObjectFiles::save(std::ofstream& output, const Scene& scene) {

	bool n = scene.has_normals();
	bool t = scene.has_textures();
	std::vector<int> txt_d;
	if (t) txt_d.resize(scene.textures_num());

	// dump vertices
	for (size_t i = 0; i < scene.vertices_num(); ++i) {
		auto v = scene.get_vertex(i);
		output << 'v' << ' ' << v.x() << ' ' << v.y() << ' ' << v.z() << '\n';
	}
	// should be normals present?
	if (n) {
		// dump normals
		for (size_t i = 0; i < scene.vertices_num(); ++i) {
			auto v = scene.get_vertex(i).normal();
			output << "vn" << ' ' << v.x << ' ' << v.y << ' ' << v.z << '\n';
		}
	}
	// should be textures present?
	if (t) {
		int d = 0;
		// dump texture coordinates
		for (size_t i = 0; i < scene.textures_num(); ++i) {
			auto& t = scene.get_texture_coordinate(i);
			txt_d[i] = d;
			if (t.first == 0) {
				++d;
				continue;
			}
			output << "vt" << ' ' << t.second.x << ' ' << t.second.y << '\n';
		}
	}
	// dump faces
	for (size_t i = 0; i < scene.triangles_num(); ++i) {
		size_t a, b, c;
		scene.get_triangle_vertices(i, a, b, c);
		auto& txt = scene.get_face(i).get_texture_coordinates();
		++a; ++b; ++c;
		if (n && t) output << 'f' << ' ' << a << "/" << (txt[0] + 1 - txt_d[txt[0]]) << "/" << a << ' '
			                             << b << "/" << (txt[1] + 1 - txt_d[txt[1]]) << "/" << b << ' ' 
			                             << c << "/" << (txt[2] + 1 - txt_d[txt[2]]) << "/" << c << '\n';
		else if (t) output << 'f' << ' ' << a << "/" << (txt[0] + 1 - txt_d[txt[0]]) << ' ' 
			                             << b << "/" << (txt[1] + 1 - txt_d[txt[1]]) << ' ' 
			                             << c << "/" << (txt[2] + 1 - txt_d[txt[2]]) << '\n';
		else if (n) output << 'f' << ' ' << a << "//" << a << ' ' << b << "//" << b << ' ' << c << "//" << c << '\n';
		else output << 'f' << ' ' << a << ' ' <<  b << ' ' << c << '\n';
	}
	return true;
}

bool WavefrontObjectFiles::load_info(std::ifstream& stream, Info& info) {

	// get length of the file
	stream.seekg (0, std::ios::end);
	long length = (long) stream.tellg();
	if (length == 0) return false;

	// reset stream cursor position
	stream.seekg (0, std::ios::beg);

	bool has_normals = false;
	info.num_vertices = 0;
	info.num_faces = 0;
	info.num_txt_coords = 0;
	info.num_normals = 0;

	std::string line;
	while (!stream.eof()){
		std::getline(stream, line);
		// we are kind and do not throw any erros
		if (line.size() > 2){
			if (line[0]=='v'){
				if(line[1]==' ' || line[1]=='\t') ++info.num_vertices;
				else if(line[1]=='t') ++info.num_txt_coords;
				else if(line[1]=='n'){ ++info.num_normals; has_normals = true; }
			} else {
				if(line[0]=='f') info.num_faces++;
			}
		}
	}

	// reset stram cursor position again
	stream.clear();
	stream.seekg(0, std::ios::beg);
	return true;
}

void WavefrontObjectFiles::tokenize_next_line(std::ifstream& stream, std::vector<std::string>& tokens) {
	
	if(stream.eof()) return;

	std::string line;
	while (std::getline(stream, line)) {
		// skip an empty line or a comment
		if (line.length()==0 || line[0] == '#') continue;

		size_t from	= 0;
		size_t to = 0;
		size_t length = line.size();

		do {
			while (from != length && (line[from] == ' ' || line[from] == '\t' || line[from] == '\r') ) from++;
			if (from != length) {
				to = from+1;
				while (to!=length && line[to] != ' ' && line[to] != '\t' && line[to] != '\r') to++;
				tokens.push_back(line.substr(from, to-from));
				from = to;
			}
		}
		while (from<length);
		// we want to tokenize just the next line, so we can leave now
		break;
	}

}

bool WavefrontObjectFiles::obj_index_ok(int &index, int max) {
	if (index > max) return false;
	if (index < 0) {
		// fit the index into the positive range
		index += max + 1;
		if (index < 0 || index > max)	return false;
	}
	return true;
}

void WavefrontObjectFiles::split_token(const std::string& token, int& v_id, int& n_id, int& t_id, Info& info) {

	std::string vertex;
	std::string texcoord;
	std::string normal;

	if( (info.num_txt_coords > 0) && (info.num_normals > 0) ) split_v_t_n_token(token, vertex, texcoord, normal);
	if(!(info.num_txt_coords > 0) && (info.num_normals > 0) ) split_v_n_token(token, vertex, normal);
	if( (info.num_txt_coords > 0) &&!(info.num_normals > 0) ) split_v_t_token(token, vertex, texcoord);
	if(!(info.num_txt_coords > 0) &&!(info.num_normals > 0) ) split_v_token(token, vertex);

	v_id = std::atoi(vertex.c_str()) - 1;
	if (info.num_txt_coords > 0) t_id = std::atoi(texcoord.c_str()) - 1;
	if (info.num_normals > 0) n_id = std::atoi(normal.c_str()) - 1;
}

void WavefrontObjectFiles::split_v_token(const std::string& token, std::string& vertex) {
	vertex = token;
}

void WavefrontObjectFiles::split_v_t_token(const std::string& token, std::string& vertex, std::string& texcoord) {

	vertex.clear();
	texcoord.clear();

	size_t from = 0;
	size_t to = 0;
	size_t length = token.size();

	if (from != length) {
		char c = token[from];
		vertex.push_back(c);

		to = from + 1;
		while (to < length && ((c = token[to]) != '/')) {
			vertex.push_back(c);
			++to;
		}
		++to;
		while (to < length && ((c = token[to]) != ' ')) {
			texcoord.push_back(c);
			++to;
		}
	}
}

void WavefrontObjectFiles::split_v_n_token(const std::string& token, std::string& vertex, std::string& normal) {

	vertex.clear();
	normal.clear();

	size_t from = 0;
	size_t to = 0;
	size_t length = token.size();

	if (from != length) {
		char c = token[from];
		vertex.push_back(c);

		to = from + 1;
		while (to != length && ((c = token[to]) != '/')) {
			vertex.push_back(c);
			++to;
		}
		++to;
		++to;  // second '/'
		while (to != length && ((c = token[to]) != ' ')) {
			normal.push_back(c);
			++to;
		}
	}
}

void WavefrontObjectFiles::split_v_t_n_token(const std::string& token, std::string& vertex, std::string& texcoord, std::string& normal) {

	vertex.clear();
	texcoord.clear();
	normal.clear();

	size_t from = 0;
	size_t to = 0;
	size_t length = token.size();

	if (from != length) {
		char c = token[from];
		vertex.push_back(c);

		to = from + 1;
		while (to != length && ((c = token[to]) != '/')) {
			vertex.push_back(c);
			++to;
		}
		++to;
		while (to != length && ((c = token[to]) != '/')) {
			texcoord.push_back(c);
			++to;
		}
		++to;
		while (to != length && ((c = token[to]) != ' ')) {
			normal.push_back(c);
			++to;
		}
	}
}