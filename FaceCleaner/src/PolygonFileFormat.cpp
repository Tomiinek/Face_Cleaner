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
 * @file    PolygonFileFormat.cpp
 */

#include "PolygonFileFormat.h"

// supported .ply types ---> cpp types mapping 
std::map<PolygonFileFormat::Type, PolygonFileFormat::PropertyInfo> 
PolygonFileFormat::property_types = {
	{Type::INT8, {1, "char"}},
	{Type::UINT8,{1, "uchar"}},
	{Type::INT16,{2, "short"}},
	{Type::UINT16,{2, "ushort"}},
	{Type::INT32,{4, "int"}},
	{Type::UINT32,{4, "uint"}},
	{Type::FLOAT32,{4, "float"}},
	{Type::FLOAT64,{8, "double"}},
	{Type::INVALID,{0, "INVALID"}}
};

bool PolygonFileFormat::save(std::ofstream & output, const Scene & scene) {
	
	// clear registered elements, because we are goint to register new ones
	_elements.clear();

	// prepare buffer for vertex coordinates
	std::map<std::string, ParsingHelper> query;
	std::vector<float> vertices;
	vertices.reserve(3 * scene.vertices_num());
	for (auto&& v : scene.get_vertices()) {
		vertices.push_back((float) v->x());
		vertices.push_back((float) v->y());
		vertices.push_back((float) v->z());
	}

	// prepare buffer for vertex indices of faces
	std::vector<uint32_t> indices;
	indices.reserve(3 * scene.triangles_num());
	for (auto&& f : scene.get_faces()) {
		Vertex* a, *b, *c;
		f->get_vertices(a, b, c);
		indices.push_back((uint32_t) a->idx());
		indices.push_back((uint32_t) b->idx());
		indices.push_back((uint32_t) c->idx());
	}

	// register vertex (with coords property) and face (with indices property) elements
	add_properties_to_element(query, "vertex", {"x", "y", "z"}, Type::FLOAT32, vertices.size(), reinterpret_cast<uint8_t*>(vertices.data()), Type::INVALID, 0);
	add_properties_to_element(query, "face", {"vertex_indices"}, Type::UINT32, indices.size(), reinterpret_cast<uint8_t*>(indices.data()), Type::UINT16, 3);
	
	// we save normals just if they were present in the original file
	std::vector<float> normals;
	if (scene.has_normals()) {
		// prepare buffer for normals
		normals.reserve(3 * scene.vertices_num());
		for (auto&& v : scene.get_vertices()) {
			auto normal = v->normal();
			normals.push_back((float) normal.x);
			normals.push_back((float) normal.y);
			normals.push_back((float) normal.z);
		}
		// register new property of vertex element
		add_properties_to_element(query, "vertex", {"nx", "ny", "nz"}, Type::FLOAT32, normals.size(), reinterpret_cast<uint8_t*>(normals.data()), Type::INVALID, 0);
	}

	// we save texture coordinates just if they were present in the original file
	std::vector<float> textures;
	if (scene.has_textures()) {
		// prepare buffer for texture coordinates
		textures.reserve(6 * scene.triangles_num());
		for (auto&& f : scene.get_faces()) {
			auto txt = f->get_texture_coordinates();
			textures.push_back((float) scene.get_texture_coordinate(txt[0]).second.x);
			textures.push_back((float) scene.get_texture_coordinate(txt[0]).second.y);
			textures.push_back((float) scene.get_texture_coordinate(txt[1]).second.x);
			textures.push_back((float) scene.get_texture_coordinate(txt[1]).second.y);
			textures.push_back((float) scene.get_texture_coordinate(txt[2]).second.x);
			textures.push_back((float) scene.get_texture_coordinate(txt[2]).second.y);
		}
		// register new property of face element
		add_properties_to_element(query, "face", {"texcoord"}, Type::FLOAT32, textures.size(), reinterpret_cast<uint8_t*>(textures.data()), Type::UINT16, 6);
	}

	// because we support binary and ASCII .ply
	if (_is_binary) write_binary(output, query);
	else write_ascii(output, query);

	return true;
}

bool PolygonFileFormat::load(std::ifstream & input, Scene & scene) {
	
	// clear Scene as we are going to import a new mesh
	scene.clear();
	// clear registered elements, because we are goint to register new ones
	_elements.clear();

	try {
		// read .ply header and set _emelent and its properties
		parse_header(input);

		bool normals_set, textures_set;
		std::shared_ptr<Data> vertices, normals, colors, faces, texcoords;
		std::map<std::string, ParsingHelper> query;
		
		// the file must have defined vertex element with its coordinates
		if (!request_properties_from_element("vertex", vertices, {"x", "y", "z"}, query)) {
			LOG(ERR) << "File does not contain vertex definition with x, y, z properties";
			return false;
		}

		// the file must have defined face element with THREE indices
		if (!request_properties_from_element("face", faces, {"vertex_indices"}, query)) {
			LOG(ERR) << "File does not contain face definition with a list of vertex indices";
			return false;
		}

		// try to get also normals and texture coordinates
		normals_set = request_properties_from_element("vertex", normals, {"nx", "ny", "nz"}, query);
		textures_set = request_properties_from_element("face", texcoords, {"texcoord"}, query);

		//
		// import the data part of .ply file:

		parse_data(input, query, true); // skip header

		// create buffers
		std::vector<std::shared_ptr<Data>> buffers;
		for (auto&& i : query) buffers.push_back(i.second.data);
		std::sort(buffers.begin(), buffers.end());
		buffers.erase(std::unique(buffers.begin(), buffers.end()), buffers.end());

		for (auto & b : buffers) {
			for (auto & entry : query) {
				if (entry.second.data != b || b->buffer.get() != nullptr) continue;
				b->buffer = Buffer(entry.second.cursor->size);
			}
		}

		parse_data(input, query, false); // read data

		//
		// ultra ugly switches because of different number types, i do not know better solution:

		// add vertices to Scene
		auto vertices_buffer = vertices->buffer.get();
		auto vertices_bytes = vertices->buffer.size_bytes();
		auto vertices_size = vertices->size;
		switch (vertices->t) {
			case Type::INT8:       add_vertex<int8_t>(vertices_buffer, vertices_bytes, vertices_size, scene);    break;
			case Type::UINT8:      add_vertex<uint8_t>(vertices_buffer, vertices_bytes, vertices_size, scene);   break;
			case Type::INT16:      add_vertex<int16_t>(vertices_buffer, vertices_bytes, vertices_size, scene);   break;
			case Type::UINT16:     add_vertex<uint16_t>(vertices_buffer, vertices_bytes, vertices_size, scene);  break;
			case Type::INT32:      add_vertex<int32_t>(vertices_buffer, vertices_bytes, vertices_size, scene);   break;
			case Type::UINT32:     add_vertex<uint32_t>(vertices_buffer, vertices_bytes, vertices_size, scene);  break;
			case Type::FLOAT32:    add_vertex<float>(vertices_buffer, vertices_bytes, vertices_size, scene);     break;
			case Type::FLOAT64:    add_vertex<double>(vertices_buffer, vertices_bytes, vertices_size, scene);    break;
			case Type::INVALID:    return false;
		}

		// add faces to Scene
		auto faces_buffer = faces->buffer.get();
		auto faces_bytes = faces->buffer.size_bytes();
		auto faces_size = faces->size;
		switch (faces->t) {
			case Type::INT8:       add_face<int8_t>(faces_buffer, faces_bytes, faces_size, scene);    break;
			case Type::UINT8:      add_face<uint8_t>(faces_buffer, faces_bytes, faces_size, scene);   break;
			case Type::INT16:      add_face<int16_t>(faces_buffer, faces_bytes, faces_size, scene);   break;
			case Type::UINT16:     add_face<uint16_t>(faces_buffer, faces_bytes, faces_size, scene);  break;
			case Type::INT32:      add_face<int32_t>(faces_buffer, faces_bytes, faces_size, scene);   break;
			case Type::UINT32:     add_face<uint32_t>(faces_buffer, faces_bytes, faces_size, scene);  break;
			case Type::FLOAT32:    add_face<float>(faces_buffer, faces_bytes, faces_size, scene);     break;
			case Type::FLOAT64:    add_face<double>(faces_buffer, faces_bytes, faces_size, scene);    break;
			case Type::INVALID:    return false;
		}

		// add normals to Scene
		if (normals_set) {
			scene.set_has_normals();
			auto normals_buffer = normals->buffer.get();
			auto normals_bytes = normals->buffer.size_bytes();
			auto normals_size = normals->size;
			switch (normals->t) {
				case Type::INT8:       add_normal<int8_t>(normals_buffer, normals_bytes, normals_size, scene);    break;
				case Type::UINT8:      add_normal<uint8_t>(normals_buffer, normals_bytes, normals_size, scene);   break;
				case Type::INT16:      add_normal<int16_t>(normals_buffer, normals_bytes, normals_size, scene);   break;
				case Type::UINT16:     add_normal<uint16_t>(normals_buffer, normals_bytes, normals_size, scene);  break;
				case Type::INT32:      add_normal<int32_t>(normals_buffer, normals_bytes, normals_size, scene);   break;
				case Type::UINT32:     add_normal<uint32_t>(normals_buffer, normals_bytes, normals_size, scene);  break;
				case Type::FLOAT32:    add_normal<float>(normals_buffer, normals_bytes, normals_size, scene);     break;
				case Type::FLOAT64:    add_normal<double>(normals_buffer, normals_bytes, normals_size, scene);    break;
				case Type::INVALID:    return false;
			}
		}

		// add textures to Scene
		if (textures_set) {
			scene.set_has_textures();
			auto textures_buffer = texcoords->buffer.get();
			auto textures_bytes = texcoords->buffer.size_bytes();
			auto textures_size = texcoords->size;
			switch (texcoords->t) {
				case Type::INT8:       add_texture<int8_t>(textures_buffer, textures_bytes, textures_size, scene);   break;
				case Type::UINT8:      add_texture<uint8_t>(textures_buffer, textures_bytes, textures_size, scene);  break;
				case Type::INT16:      add_texture<int16_t>(textures_buffer, textures_bytes, textures_size, scene);  break;
				case Type::UINT16:     add_texture<uint16_t>(textures_buffer, textures_bytes, textures_size, scene); break;
				case Type::INT32:      add_texture<int32_t>(textures_buffer, textures_bytes, textures_size, scene);  break;
				case Type::UINT32:     add_texture<uint32_t>(textures_buffer, textures_bytes, textures_size, scene); break;
				case Type::FLOAT32:    add_texture<float>(textures_buffer, textures_bytes, textures_size, scene);    break;
				case Type::FLOAT64:    add_texture<double>(textures_buffer, textures_bytes, textures_size, scene);   break;
				case Type::INVALID:    return false;
			}
		}

	} 
	catch (const std::exception & e) { 
		LOG(ERR) << "An error occurred while reading a file, please check the file format or report this error:";
		LOG(ERR) << e.what();
		return false;
	}

	return true;
}

bool PolygonFileFormat::parse_header(std::istream & is) {

	std::string line;
	while (std::getline(is, line)) {
		std::istringstream ls(line);
		std::string token;
		ls >> token;
		if (token == "ply" || token == "PLY" || token == "") continue;
		else if (token == "comment") continue;
		else if (token == "format") {
			ls >> token;
			if (token == "binary_little_endian") _is_binary = true;
			else if (token == "binary_big_endian") _is_binary = _is_big_endian = true;
		}
		else if (token == "element") {
			_elements.emplace_back(ls);
		}
		else if (token == "property") {
			_elements.back().properties.emplace_back(ls);
		}
		else if (token == "obj_info") continue;
		else if (token == "end_header") break;
		else {
			LOG(WRN) << "File contains an unexpected token: " << token;
			return false;
		}
	}
	return true;
}

void PolygonFileFormat::parse_data(std::istream & is, const std::map<std::string, ParsingHelper>& query, bool first_pass) {

	std::function<size_t(const Type t, void* dest, size_t& offset, std::istream& is)> read;
	std::function<size_t(const Property& p, std::istream& is)> skip;

	const auto start = is.tellg();

	if (_is_binary) {
		read = [&](const Type t, void * dest, size_t& offset, std::istream& _is) { return read_property_binary(t, dest, offset, _is); };
		skip = [&](const Property& p, std::istream & _is) { return skip_property_binary(p, _is); };
	}
	else {
		read = [&](const Type t, void * dest, size_t& offset, std::istream& _is) { return read_property_ascii(t, dest, offset, _is); };
		skip = [&](const Property& p, std::istream& _is) { return skip_property_ascii(p, _is); };
	}

	for (auto&& element : _elements) {
		for (size_t count = 0; count < element.size; ++count) {
			if (count == 102394) {
				count = 102394;
			}
			for (auto&& property : element.properties) {
				auto it = query.find(make_key(element.name, property.name));
				if (it == query.end()) {
					skip(property, is);
					continue;
				}
				auto& helper = it->second;
				if (first_pass) {
					helper.cursor->size += skip(property, is);
					continue;
				}
				if (property.is_list) {
					size_t size = 0;
					size_t dummyCount = 0;
					read(property.list_type, &size, dummyCount, is);
					for (size_t i = 0; i < size; ++i) {
						read(property.type, (helper.data->buffer.get() + helper.cursor->offset), helper.cursor->offset, is);
					}
				}
				else read(property.type, (helper.data->buffer.get() + helper.cursor->offset), helper.cursor->offset, is);
			}
		}
	}
	if (first_pass) {
		is.clear();
		is.seekg(start, is.beg);
	}
}

size_t PolygonFileFormat::read_property_binary(const Type t, void * dest, size_t & offset, std::istream & is) {

	int stride = property_types[t].stride;
	offset += stride;
	std::vector<char> src(stride);
	is.read(src.data(), stride);
	switch (t) {
		case Type::INT8:       ply_cast<int8_t>(dest, src.data(), _is_big_endian);        break;
		case Type::UINT8:      ply_cast<uint8_t>(dest, src.data(), _is_big_endian);       break;
		case Type::INT16:      ply_cast<int16_t>(dest, src.data(), _is_big_endian);       break;
		case Type::UINT16:     ply_cast<uint16_t>(dest, src.data(), _is_big_endian);      break;
		case Type::INT32:      ply_cast<int32_t>(dest, src.data(), _is_big_endian);       break;
		case Type::UINT32:     ply_cast<uint32_t>(dest, src.data(), _is_big_endian);      break;
		case Type::FLOAT32:    ply_cast_float<float>(dest, src.data(), _is_big_endian);   break;
		case Type::FLOAT64:    ply_cast_double<double>(dest, src.data(), _is_big_endian); break;
		case Type::INVALID:    throw std::invalid_argument("invalid ply property");
	}
	return stride;
}

size_t PolygonFileFormat::read_property_ascii(const Type t, void * dest, size_t & offset, std::istream & is) {
	
	int stride = property_types[t].stride;
	offset += stride;
	switch (t) {
		case Type::INT8:       *((int8_t *) dest) = ply_read_ascii<int32_t>(is);       break;
		case Type::UINT8:      *((uint8_t *) dest) = ply_read_ascii<uint32_t>(is);     break;
		case Type::INT16:      ply_cast_ascii<int16_t>(dest, is);                      break;
		case Type::UINT16:     ply_cast_ascii<uint16_t>(dest, is);                     break;
		case Type::INT32:      ply_cast_ascii<int32_t>(dest, is);                      break;
		case Type::UINT32:     ply_cast_ascii<uint32_t>(dest, is);                     break;
		case Type::FLOAT32:    ply_cast_ascii<float>(dest, is);                        break;
		case Type::FLOAT64:    ply_cast_ascii<double>(dest, is);                       break;
		case Type::INVALID:    throw std::invalid_argument("invalid ply property");
	}
	return stride;
}

size_t PolygonFileFormat::skip_property_binary(const Property & p, std::istream & is) {
	
	static std::vector<char> skip(property_types[p.type].stride);
	if (p.is_list) {
		size_t size = 0;
		size_t dummy = 0;
		read_property_binary(p.list_type, &size, dummy, is);
		for (size_t i = 0; i < size; ++i) is.read(skip.data(), property_types[p.type].stride);
		return size * property_types[p.type].stride;
	}
	else {
		is.read(skip.data(), property_types[p.type].stride);
		return property_types[p.type].stride;
	}
}

size_t PolygonFileFormat::skip_property_ascii(const Property & p, std::istream & is) {
	
	std::string skip;
	if (p.is_list) {
		size_t size = 0;
		size_t dummyCount = 0;
		read_property_ascii(p.list_type, &size, dummyCount, is);
		for (size_t i = 0; i < size; ++i) is >> skip;
		return size * property_types[p.type].stride;
	}
	else {
		is >> skip;
		return property_types[p.type].stride;
	}
}

bool PolygonFileFormat::request_properties_from_element(const std::string & element, std::shared_ptr<Data>& result, const std::initializer_list<std::string> properties, std::map<std::string, ParsingHelper>& queries) {
	
	ParsingHelper helper;
	helper.data = std::make_shared<Data>();
	helper.data->size = 0;
	helper.data->t = Type::INVALID;
	helper.cursor = std::make_shared<Cursor>();
	helper.cursor->offset = 0;
	helper.cursor->size = 0;

	int element_idx = (int) find_element(element);
	if (element_idx == -1) return false;
	auto& e = _elements[element_idx];
	helper.data->size = e.size;

	for (auto p : properties) {
		int property_idx = (int) find_property(p, e.properties);
		if (property_idx == -1) return false;
		auto& p = e.properties[property_idx];
		helper.data->t = p.type;
		auto result = queries.insert(std::pair<std::string, ParsingHelper>(make_key(e.name, p.name), helper));
		if (!result.second) return false;
	}
	result = helper.data;
	return true;
}

void PolygonFileFormat::write_header(std::ostream & os) const {

	const std::locale& fix_loc = std::locale("C");
	os.imbue(fix_loc);
	os << "ply\n";
	if (_is_binary) os << ((_is_big_endian) ? "format binary_big_endian 1.0" : "format binary_little_endian 1.0") << "\n";
	else os << "format ascii 1.0\n";

	for (auto&& e : _elements) {
		os << "element " << e.name << " " << e.size << "\n";
		for (auto&& p : e.properties) {
			if (p.is_list) {
				os << "property list " << property_types[p.list_type].name << " "
					<< property_types[p.type].name << " " << p.name << "\n";
			}
			else os << "property " << property_types[p.type].name << " " << p.name << "\n";
		}
	}
	os << "end_header\n";
}

void PolygonFileFormat::write_binary(std::ostream & os, const std::map<std::string, ParsingHelper>& queries) const {

	write_header(os);
	for (auto&& e : _elements) {
		for (size_t i = 0; i < e.size; ++i) {
			for (auto&& p : e.properties) {
				auto & helper = queries.find(make_key(e.name, p.name))->second;
				if (!p.is_list) {
					write_property_binary(p.type, os, (helper.data->buffer.get() + helper.cursor->offset), helper.cursor->offset);
					continue;
				}
				uint8_t list_size[4] = {0, 0, 0, 0};
				std::memcpy(list_size, &p.list_size, sizeof(uint32_t));
				size_t dummy = 0;
				write_property_binary(p.list_type, os, list_size, dummy);
				for (int j = 0; j < p.list_size; ++j) {
					write_property_binary(p.type, os, (helper.data->buffer.get() + helper.cursor->offset), helper.cursor->offset);
				}
			}
		}
	}
}

void PolygonFileFormat::write_ascii(std::ostream & os, const std::map<std::string, ParsingHelper>& queries) const {

	write_header(os);
	for (auto&& e : _elements) {
		for (size_t i = 0; i < e.size; ++i) {
			for (auto&& p : e.properties) {
				auto& helper = queries.find(make_key(e.name, p.name))->second;
				if (!p.is_list) {
					write_property_ascii(p.type, os, (helper.data->buffer.get() + helper.cursor->offset), helper.cursor->offset);
					continue;
				}
				os << p.list_size << " ";
				for (int j = 0; j < p.list_size; ++j) {
					write_property_ascii(p.type, os, (helper.data->buffer.get() + helper.cursor->offset), helper.cursor->offset);
				}
			}
			os << "\n";
		}
	}
}

void PolygonFileFormat::write_property_ascii(Type t, std::ostream & os, uint8_t * src, size_t & offset) const {
	switch (t) {
		case Type::INT8:       os << static_cast<int32_t>(*reinterpret_cast<int8_t*>(src));    break;
		case Type::UINT8:      os << static_cast<uint32_t>(*reinterpret_cast<uint8_t*>(src));  break;
		case Type::INT16:      os << *reinterpret_cast<int16_t*>(src);     break;
		case Type::UINT16:     os << *reinterpret_cast<uint16_t*>(src);    break;
		case Type::INT32:      os << *reinterpret_cast<int32_t*>(src);     break;
		case Type::UINT32:     os << *reinterpret_cast<uint32_t*>(src);    break;
		case Type::FLOAT32:    os << *reinterpret_cast<float*>(src);       break;
		case Type::FLOAT64:    os << *reinterpret_cast<double*>(src);      break;
		case Type::INVALID:    throw std::invalid_argument("invalid ply property");
	}
	os << " ";
	offset += property_types[t].stride;
}

void PolygonFileFormat::write_property_binary(Type t, std::ostream & os, uint8_t * src, size_t & offset) const {
	os.write((char *) src, property_types[t].stride);
	offset += property_types[t].stride;
}

void PolygonFileFormat::add_properties_to_element(std::map<std::string, ParsingHelper>& queries, const std::string & element, 
												  const std::initializer_list<std::string> properties, const Type type, 
												  const size_t count, uint8_t * data, const Type list_type, const size_t list_count) {
	ParsingHelper helper;
	helper.data = std::make_shared<Data>();
	helper.data->size = count;
	helper.data->t = type;
	helper.data->buffer = Buffer(data);
	helper.cursor = std::make_shared<Cursor>();
	helper.cursor->offset = 0;
	helper.cursor->size = 0;

	auto create_property_on_element = [&](Element& e) {
		for (auto p : properties) {
			Property new_prop = (list_type == Type::INVALID) ? Property(type, p) : Property(list_type, type, p, list_count);
			auto result = queries.insert(std::pair<std::string, ParsingHelper>(make_key(element, p), helper));
			if (result.second) e.properties.push_back(new_prop);
		}
	};

	int idx = (int) find_element(element);
	if (idx == -1) {
		auto new_element = (list_type == Type::INVALID) ? Element(element, count / properties.size()) : Element(element, count / list_count);
		create_property_on_element(new_element);
		_elements.push_back(new_element);
	}
	else {
		Element& e = _elements[idx];
		create_property_on_element(e);
	}
}
