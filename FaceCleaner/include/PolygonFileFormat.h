#ifndef FACECLEANER_POLYGONFILEFORMAT_H
#define FACECLEANER_POLYGONFILEFORMAT_H

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
 * @brief   Polygon File Format importer and exporter. 
 *
 * @file    PolygonFileFormat.h
 * @author  Tomas Nekvinda <tom(at)neqindi.cz>
 */

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <map>
#include <functional>
#include "IModelHandler.h"
#include "Logger.h"

class PolygonFileFormat : public IModelHandler
{
public:
	/**
	 * @brief Export the given Scene into the file stream.
	 * 
	 * @param output Output file stream.
	 * @param scene  Scene to be exported.
	 * @return 		 @c true if the export was successful, @c false otherwise. 
	 * 
	 * Modified code by Dimitri Diakopoulos (http://www.dimitridiakopoulos.com)
	 * Also see: https://github.com/ddiakopoulos/tinyply
	 *
	 * Dimitri Diakopoulos's license:
	 * This software is in the public domain. Where that dedication is not recognized, 
	 * you are granted a perpetual, irrevocable license to copy, distribute, and modify 
	 * this file as you see fit. If these terms are not suitable to your organization, 
	 * you may choose to license it under the terms of the 2-clause simplified BSD.
	 *
	 */
	virtual bool save(std::ofstream& output, const Scene& scene);

	/**
	 * @brief Add vertices to the Scene object.
	 * 
	 * @tparam T 			Numeric type of coordiantes of vertices.
	 * @param src 			Byte buffer with vertex coordinates.
	 * @param length_bytes 	Size of the buffer @a src .
	 * @param length 		Number of coordinates stored in @ src buffer.
	 * @param scene			Scene in which vertices should be added.
	 */
	template<typename T>
	void add_vertex(uint8_t* src, size_t length_bytes, size_t length, Scene& scene) {
		std::vector<T> v(length * 3);
		std::memcpy(v.data(), src, length_bytes);
		for (size_t i = 0; i < v.size(); i += 3) {
			scene.add_vertex((double) v[i], (double) v[i + 1], (double) v[i + 2]);
		}
	}

	/**
	 * @brief Add triangular faces to the Scene object.
	 * 
	 * @tparam T 			Numeric type of vertex indices of faces.
	 * @param src 			Byte buffer with vertex indices of faces.
	 * @param length_bytes 	Size of the buffer @a src .
	 * @param length 		Number of indices stored in @ src buffer.
	 * @param scene 		Scene in which faces should be added.
	 */
	template<typename T>
	void add_face(uint8_t* src, size_t length_bytes, size_t length, Scene& scene) {
		std::vector<T> v(length * 3);
		std::memcpy(v.data(), src, length_bytes);
		for (size_t i = 0; i < v.size(); i += 3) {
			scene.add_face((size_t) v[i], (size_t) v[i + 1], (size_t) v[i + 2]);
		}
	}

	/**
	 * @brief Set normals to the Scene object.
	 * 
	 * @tparam T 			Numeric type of normals coordiantes.
	 * @param src 			Byte buffer with coordinates of normals.
	 * @param length_bytes  Size of the buffer @a src .
	 * @param length 		Number of coordinates stored in @ src buffer.
	 * @param scene 		Scene whose normal should be set.
	 */
	template<typename T>
	void add_normal(uint8_t* src, size_t length_bytes, size_t length, Scene& scene) {
		std::vector<T> v(length * 3);
		std::memcpy(v.data(), src, length_bytes);
		for (size_t i = 0; i < v.size(); i += 3) {
			scene.get_vertex(i / 3).set_normal((double) v[i], (double) v[i + 1], (double) v[i + 2]);
		}
	}

	/**
	 * @brief Set texture coordinates to faces of the Scene object.
	 * 
	 * @tparam T 			Numeric type of texture coordiantes.
	 * @param src 			Byte buffer with texture coordinates.
	 * @param length_bytes  Size of the buffer @a src .
	 * @param length 		Number of coordinates stored in @ src buffer.
	 * @param scene 		Scene whose textures should be set.
	 */
	template<typename T>
	void add_texture(uint8_t* src, size_t length_bytes, size_t length, Scene& scene) {
		std::vector<T> v(length * 6);
		std::memcpy(v.data(), src, length_bytes);
		for (size_t i = 0; i < v.size(); i += 6) {
			size_t h1 = scene.add_texture_coordinate((double) v[i], (double) v[i + 1]);
			size_t h2 = scene.add_texture_coordinate((double) v[i + 2], (double) v[i + 3]);
			size_t h3 = scene.add_texture_coordinate((double) v[i + 4], (double) v[i + 5]);
			scene.get_face(i / 6).set_texture_coordinates(h1, h2, h3);
		}
	}

	/**
	 * @brief Import a file into the Scene object.
	 * 
	 * @param input Input file stream.
	 * @param scene Scene object to be initialized.
	 * @return 		@c true if the import was successful, @c false otherwise. 
	 */
	virtual bool load(std::ifstream& input, Scene& scene);

private:

	// supported .ply types
	enum class Type : uint8_t { INVALID, INT8, UINT8, INT16, UINT16, INT32, UINT32, FLOAT32, FLOAT64 };

	//
	// follows the code from Tinyply by Dimitri Diakopoulos, see: https://github.com/ddiakopoulos/tinyply
	// comments for the code is not provided, please see the original implementation for further details
	//

	class Buffer
	{
	public:
		Buffer() {};
		Buffer(const size_t size) : _data(new uint8_t[size], delete_array()), _size(size) { _alias = _data.get(); }
		Buffer(uint8_t * ptr) { _alias = ptr; }
		uint8_t * get() { return _alias; }
		size_t size_bytes() const { return _size; }
	private:
		uint8_t * _alias{nullptr};
		struct delete_array { void operator()(uint8_t * p) { delete[] p; } };
		std::unique_ptr<uint8_t, decltype(Buffer::delete_array())> _data;
		size_t _size;
	};
	struct Data
	{
		Type t;
		size_t size;
		Buffer buffer;
	};
	struct Cursor
	{
		size_t offset;
		size_t size;
	};
	struct ParsingHelper
	{
		std::shared_ptr<Data> data;
		std::shared_ptr<Cursor> cursor;
	};

	struct PropertyInfo { int stride; std::string name; };
	static std::map<Type, PropertyInfo> property_types;

	struct Property
	{
		Property(std::istream& is) : is_list(false) {
			std::string raw_type;
			is >> raw_type;
			if (raw_type == "list") {
				std::string size_type;
				is >> size_type >> raw_type;
				list_type = property_type_from_string(size_type);
				is_list = true;
			}
			type = property_type_from_string(raw_type);
			is >> name;
		}

		Property(Type t, std::string& n) : name(n), type(t) {}
		Property(Type list_t, Type t, std::string& n, size_t s) :
			name(n), type(t), is_list(true), list_type(list_t), list_size(s) {}

		std::string name;
		Type type;
		bool is_list = false;
		Type list_type = Type::INVALID;
		size_t list_size = 0;
	};
	struct Element
	{
		Element(std::istream& istream) { istream >> name >> size; }
		Element(const std::string & _name, size_t count) : name(_name), size(count) {}
		std::string name;
		size_t size;
		std::vector<Property> properties;
	};

	static Type property_type_from_string(const std::string & t) {
		if (t == "int8" || t == "char")             return Type::INT8;
		else if (t == "uint8" || t == "uchar")      return Type::UINT8;
		else if (t == "int16" || t == "short")      return Type::INT16;
		else if (t == "uint16" || t == "ushort")    return Type::UINT16;
		else if (t == "int32" || t == "int")        return Type::INT32;
		else if (t == "uint32" || t == "uint")      return Type::UINT32;
		else if (t == "float32" || t == "float")    return Type::FLOAT32;
		else if (t == "float64" || t == "double")   return Type::FLOAT64;
		return Type::INVALID;
	}

	template<typename T> T endian_swap(const T & v) { return v; }
	template<> uint16_t endian_swap(const uint16_t & v) { return (v << 8) | (v >> 8); }
	template<> uint32_t endian_swap(const uint32_t & v) { return (v << 24) | ((v << 8) & 0x00ff0000) | ((v >> 8) & 0x0000ff00) | (v >> 24); }
	template<> uint64_t endian_swap(const uint64_t & v) {
		return (((v & 0x00000000000000ffLL) << 56) |
			((v & 0x000000000000ff00LL) << 40) |
				((v & 0x0000000000ff0000LL) << 24) |
				((v & 0x00000000ff000000LL) << 8) |
				((v & 0x000000ff00000000LL) >> 8) |
				((v & 0x0000ff0000000000LL) >> 24) |
				((v & 0x00ff000000000000LL) >> 40) |
				((v & 0xff00000000000000LL) >> 56));
	}
	template<> int16_t endian_swap(const int16_t & v) { uint16_t r = endian_swap(*(uint16_t*) &v); return *(int16_t*) &r; }
	template<> int32_t endian_swap(const int32_t & v) { uint32_t r = endian_swap(*(uint32_t*) &v); return *(int32_t*) &r; }
	template<> int64_t endian_swap(const int64_t & v) { uint64_t r = endian_swap(*(uint64_t*) &v); return *(int64_t*) &r; }
	float endian_swap_float(const uint32_t & v) { union { float f; uint32_t i; }; i = endian_swap(v); return f; }
	double endian_swap_double(const uint64_t & v) { union { double d; uint64_t i; }; i = endian_swap(v); return d; }

	template<typename T> void ply_cast(void * dest, const char * src, bool be) {
		*(static_cast<T *>(dest)) = (be) ? endian_swap(*(reinterpret_cast<const T *>(src))) : *(reinterpret_cast<const T *>(src));
	}
	template<typename T> void ply_cast_float(void * dest, const char * src, bool be) {
		*(static_cast<T *>(dest)) = (be) ? endian_swap_float(*(reinterpret_cast<const uint32_t *>(src))) : *(reinterpret_cast<const T *>(src));
	}
	template<typename T> void ply_cast_double(void * dest, const char * src, bool be) {
		*(static_cast<T *>(dest)) = (be) ? endian_swap_double(*(reinterpret_cast<const uint64_t *>(src))) : *(reinterpret_cast<const T *>(src));
	}
	template<typename T> T ply_read_ascii(std::istream & is) {
		T data;
		is >> data;
		return data;
	}
	template<typename T> void ply_cast_ascii(void * dest, std::istream & is) {
		*(static_cast<T *>(dest)) = ply_read_ascii<T>(is);
	}

	std::string make_key(const std::string& a, const std::string& b) const { return (a + "-" + b); }
	size_t find_element(const std::string& key) const {
		for (size_t i = 0; i < _elements.size(); ++i) if (_elements[i].name == key) return i;
		return -1;
	}
	size_t find_property(const std::string & key, const std::vector<Property>& list) {
		for (size_t i = 0; i < list.size(); ++i) if (list[i].name == key) return i;
		return -1;
	}

	bool parse_header(std::istream & is);
	void parse_data(std::istream& is, const std::map<std::string, ParsingHelper>& query, bool first_pass);

	size_t read_property_binary(const Type t, void* dest, size_t& offset, std::istream& is);
	size_t read_property_ascii(const Type t, void * dest, size_t & offset, std::istream & is);
	size_t skip_property_binary(const Property& p, std::istream& is);
	size_t skip_property_ascii(const Property& p, std::istream& is);

	bool request_properties_from_element(const std::string & element, std::shared_ptr<Data>& result,
										 const std::initializer_list<std::string> properties,
										 std::map<std::string, ParsingHelper>& queries);

	void write_header(std::ostream& os) const;
	void write_binary(std::ostream & os, const std::map<std::string, ParsingHelper>& queries) const;
	void write_ascii(std::ostream & os, const std::map<std::string, ParsingHelper>& queries) const;
	void write_property_ascii(Type t, std::ostream & os, uint8_t * src, size_t & offset) const;
	void write_property_binary(Type t, std::ostream& os, uint8_t* src, size_t& offset) const;

	void add_properties_to_element(std::map<std::string, ParsingHelper>& queries, const std::string& element, const std::initializer_list<std::string> properties,
								   const Type type, const size_t count, uint8_t* data, const Type list_type, const size_t list_count);

	bool _is_binary = false;
	bool _is_big_endian = false;
	std::vector<Element> _elements;

};


#endif //FACECLEANER_POLYGONFILEFORMAT_H