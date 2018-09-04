#ifndef FACECLEANER_WAVEFRONTOBJECTFILES_H
#define FACECLEANER_WAVEFRONTOBJECTFILES_H

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
 * @brief   Wavefront .OBJ importer and exporter. 
 *
 * @file    WavefrontObjectFiles.h
 * @author  Tomas Nekvinda <tom(at)neqindi.cz>
 */

#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>
#include "IModelHandler.h"
#include "Logger.h"

class WavefrontObjectFiles : public IModelHandler
{
public:

	/**
	 * @brief Object which holds basic info about the file.
	 * 
	 */
	struct Info
	{
		int num_vertices = 0;
		int num_faces = 0;
		int num_txt_coords = 0;
		int num_normals = 0;
	};

	/**
	 * @brief Export the given Scene into the file stream.
	 * 
	 * @param output Output file stream.
	 * @param scene  Scene to be exported.
	 * @return 		 @c true if the export was successful, @c false otherwise. 
	 */
	bool save(std::ofstream& output, const Scene& scene) override;

	/**
	 * @brief Import file into the Scene object.
	 * 
	 * @param input Input file stream.
	 * @param scene Scene object to be initialized.
	 * @return 		@c true if the import was successful, @c false otherwise. 
	 */
	bool load(std::ifstream& input, Scene& scene) override;

private:
	/**
	 * @brief Briefly read the file and export basic information.
	 * 
	 * @param input The input file stream.
	 * @param mask 	Info storage.
	 * @return 		 @c false if the file is empty, @true otherwise
	 */
	bool load_info(std::ifstream& input, Info& mask);

	/**
	 * @brief Check if the index is in the valid range.
	 * 
	 * @param index  Index to be checked, negative indices are handled correctly.
	 * @param max 	 Maximal valid index.
	 * @return 		 @c true if the @a index is valid, @c false otherwise. 
	 */
	bool obj_index_ok(int &index, int max);

	/**
	 * @brief Read the next line and split it into tokens by whitespaces.
	 * 
	 * @param input 	File stream which is source of the line.
	 * @param tokens 	Vector in which are the resulting tokens saved.
	 */
	void tokenize_next_line(std::ifstream& input, std::vector<std::string>& tokens);

	/**
	 * @brief Split the face token, there are possible four variants.
	 * 
	 * @param token The token to be splitted, something like 1/2/3 or 1//2, ...
	 * @param v_id  Extracted index of the vertex after token splitting.
	 * @param n_id  Extracted index of the normal after token splitting.
	 * @param t_id  Extracted index of the texture coordinate after token splitting.
	 * @param info  Info object which is used to determine what we should look after.
	 */
	void split_token(const std::string& token, int& v_id, int& n_id, int& t_id, Info& info);
	void split_v_token(const std::string& token, std::string& vertex);
	void split_v_t_token(const std::string& token, std::string& vertex, std::string& texcoord);
	void split_v_n_token(const std::string& token, std::string& vertex, std::string& normal);
	void split_v_t_n_token(const std::string& token, std::string& vertex, std::string& texcoord, std::string& normal);
};


#endif //FACECLEANER_WAVEFRONTOBJECTFILES_H
