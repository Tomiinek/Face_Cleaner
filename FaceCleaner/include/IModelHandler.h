#ifndef FACECLEANER_IMODELHANDLER_H
#define FACECLEANER_IMODELHANDLER_H

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
 * @brief   Interface defining methods which should be present in a mesh importer/exporter.
 *
 * @file    IModelHandler.h
 * @author  Tomas Nekvinda <tom(at)neqindi.cz>
 */

#include <memory>
#include "Scene.h"

class IModelHandler
{
public:
	virtual ~IModelHandler() = default;

	/**
	 * @brief Import a trimesh file into the Scene object.
	 * 
	 * @param input Input file stream.
	 * @param scene Scene object to be filled.
	 * @return 		@c true if the file was imported successfuly, @c false otherwise 
	 */
	virtual bool load(std::ifstream& input, Scene& scene) = 0;

	/**
	 * @brief Export the Scene object into a file. 
	 * 
	 * @param output Output file stream.
	 * @param scene  Scene object to be dumped.
	 * @return		 @c true if the scene was exported successfuly, @c false otherwise
	 */
	virtual bool save(std::ofstream& output, const Scene& scene) = 0;
};


#endif //FACECLEANER_IMODELHANDLER_H
