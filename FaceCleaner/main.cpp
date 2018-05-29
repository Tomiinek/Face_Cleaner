/*****************************************************************************\
**                                                       _ __   ___ __ _     **
**    Face Cleaner                                      | '_ \ / __/ _` |    **
**    Copyright (c) 2018                                | | | | (_| (_| |    **
**    Tomas Nekvinda, tom(at)neqindi.cz                 |_| |_|\___\__, |    **
**                                                                    |_|    **
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

#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#define NOMINMAX
#include <windows.h>

#include "boost/program_options.hpp"
#include <boost/filesystem.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

#include <Eigen/Eigen>

#include "Logger.h"
#include "Scene.h"
#include "Vertex.h"
#include "Cleaner.h"
#include "PointClassifier.h"
#include "WavefrontObjectFiles.h"
#include "PolygonFileFormat.h"

namespace po = boost::program_options;
namespace bf = boost::filesystem;

// configuration of the logger
structlog LOGCFG = {};

// return valus and version ...
namespace {
	const std::string version = "1.0.0";
	const size_t ERROR_IN_COMMAND_LINE = 1;
	const size_t SUCCESS = 0;
	const size_t UNHANDLED_EXCEPTION = 2;
}

int parse_settings(int argc, const char* argv[], bf::path exec_base, po::variables_map& vm, int& verbosity, int& threads, int& max_refinement,
					std::string& log_file, std::string& file_suffix, std::string& destination_path, std::string& landmark_suffix) {

	std::string config_file;

	std::string cfg_default = ".\\face-cleaner.cfg";
	std::string log_default = ".\\face-cleaner.log";
#ifdef BOOST_POSIX_API
	std::replace(cfg_default.begin(), cfg_default.end(), '\\', '/');
	std::replace(log_default.begin(), log_default.end(), '\\', '/');
#endif

	// definition of all generic options
	po::options_description generic("Generic options");
	generic.add_options()
		("version", "print information about the current program version")
		("help", "produce help message")
		("manual", "print description of the required configuration file parameters");

	// definition of other command line options
	po::options_description config("Configuration options");
	config.add_options()
		("verbose,v", po::value<int>(&verbosity)->default_value(3), "set verbosity level, 0 quiet, 1 error, 2 warning (default), 3 full info")
		("threads,t", po::value<int>(&threads)->default_value(0), "maximal number of threads available, 0 to set automatically")
		("config,c", po::value<std::string>(&config_file)->default_value(cfg_default), "path (absolute or relative) the configuration file")
		("log-file,l", po::value<std::string>(&log_file)->default_value(log_default), "name of the log file")
		("suffix,s", po::value<std::string>(&file_suffix)->default_value("_cleaned"), "suffix added to processed files")
		("destination,d", po::value<std::string>(&destination_path)->default_value("."), "path (absolute or relative) to the directory where will be stored output files, set . to save beside input files");

	// definition of parameters of the cinfiguration file, all are required
	po::options_description params("Required parameters");
	params.add_options()
		("scale", po::value<double>()->required(), "The number of mesh units which corresponds to 1 millimetre in reality.")
		("save-landmarks", po::bool_switch()->required(), "Set ON if you want to detect and store landmarks into the file specified by landmarks-suffix parameter. OFF if you do not want to save the landmarks.")
		("landmarks-suffix", po::value<std::string>(&landmark_suffix)->required(), "Suffix will be appended to all output files containing indices of detected landmarks.")
		("crop", po::bool_switch()->required(), "Set ON if you want to trim the facial area, OFF otherwise. Usually preserves neck.See also crop - by - normal and the group of face cropping parameters.")
		("crop-by-normal", po::bool_switch()->required(), "Set ON if you want to trim the facial area and you want to remove also neck, OFF otherwise. To have an effect, the crop parameter must also be set to ON.See also the group of face cropping parameters.")
		("remove-defects", po::bool_switch()->required(), "Set ON if you want to remove geometrical errors, OFF otherwise. See also the group of defects removal parameters.")
		("fill-holes", po::bool_switch()->required(), "Set ON if you want to fill mesh holes, OFF otherwise. See also the max - refinement parameter.")
		("thin-triangles", po::bool_switch()->required(), "Set ON if you want to resolve thin or zero - volume triangles, OFF otherwise. See also the min - triangle parameter.")
		("rotate", po::bool_switch()->required(), "Set ON if you want to rotate the face mesh to a uniform orientation based od the face landmarks, OFF otherwise.")
		("nose-tip-model", po::value<std::string>()->required(), "A path (absolute or relative) to the nose tip classifier.")
		("nose-root-model", po::value<std::string>()->required(), "A path (absolute or relative) to the nose root classifier.")
		("eye-corner-model", po::value<std::string>()->required(), "A path (absolute or relative) to the eye corners classifier.")
		("mouth-corner-model", po::value<std::string>()->required(), "A path (absolute or relative) to the mouth corners classifier.")
		("e2e-mean", po::value<double>()->required(), "The expected distance between inner eye corners in millimetres.")
		("e2e-dev", po::value<double>()->required(), "The expected standard deviation of the distance between inner eye corners in millimetres.")
		("m2m-mean", po::value<double>()->required(), "The expected distance of mouth corners in millimetres.")
		("m2m-dev", po::value<double>()->required(), "The expected standard deviation of the distance of mouth corners in millimeters.")
		("n2r-mean", po::value<double>()->required(), "The expected distance between the nose tip and the nose root in millimeters.")
		("n2r-dev", po::value<double>()->required(), "The expected standard deviation of the distance between the nose tip and the nose root in millimeters.")
		("n2e-over-n2r-mean", po::value<double>()->required(), "The expected ratio of the nose tip-to-inner eye corner distance to nose tip-to-nose root distance.")
		("n2e-over-n2r-dev", po::value<double>()->required(), "The expected standard deviation of the ratio of the nose tip-to-inner eye corner distance to nose tip-to-nose root distance.")
		("r2e-over-e2e-mean", po::value<double>()->required(), "The expected ratio of the nose root-to-inner eye corner distance to eye corners distance.")
		("r2e-over-e2e-dev", po::value<double>()->required(), "The expected standard deviation of the ratio of the nose root-to-inner eye corner distance to eye corners distance.")
		("r2m-over-n2r-mean", po::value<double>()->required(), "The expected ratio of the nose root-to-mouth corner distance to nose tip-to-nose root distance.")
		("r2m-over-n2r-dev", po::value<double>()->required(), "The expected standard deviation of the ratio of the nose root-to-mouth corner distance to nose tip-to-nose root distance.")
		("n2m-over-m2m-mean", po::value<double>()->required(), "The expected ratio of the nose tip-to-mouth corner distance to mouth corners distance.")
		("n2m-over-m2m-dev", po::value<double>()->required(), "The expected standard deviation of the ratio of the nose tip-to-mouth corner distance to mouth corners distance.")
		("far", po::value<double>()->required(), "The maximal allowed distance between a mesh vertex and the nose tip in millimetres.")
		("near", po::value<double>()->required(), "A distance from the nose tip in millimetres.Farther vertices are checked, nearer are not.")
		("n2ch-over-n2r-mean", po::value<double>()->required(), "The expected ratio of the nose tip-to-chin distance to nose tip-to-nose root distance.")
		("n2ch-over-n2r-dev", po::value<double>()->required(), "The expected standard deviation of the ratio of the nose tip-to-chin distance to nose tip-to-nose root distance.")
		("neck-normal-threshold", po::value<double>()->required(), "The maximal allowed angle (radians) between the direction of sight and the normal of any vertex near chin.See also the near parameter.")
		("ears-normal-threshold", po::value<double>()->required(), "The maximal allowed angle (radians) between the direction of sight and the normal of any vertex near ears.See also the near parameter.")
		("neck-angle-threshold", po::value<double>()->required(), "The maximal angle (radians) between the plane defined by inner eye corners and mouth corners and the vector defined by a vertex near chin and the center of face.")
		("ears-angle-threshold", po::value<double>()->required(), "The maximal angle (radians) between the plane defined by inner eye corners and mouth corners and the vector defined by a vertex near ears and the center of face.")
		("curvedness-tolerance", po::value<double>()->required(), "The maximal curvedness accepted in the area between near and far")
		("wide-protrusion-range", po::value<double>()->required(), "The size of boundary neighbourhood (millimetres) which is used to find out boundary protrusions. See also  wide-protrusion-angle and tiny-protrusion-angle parametres.")
		("wide-protrusion-angle", po::value<double>()->required(), "The maximal tolerated angle (radians) between a boundary vertex and the two boundary vertices which are distant twice the wide-protrusion-range from each other.")
		("tiny-protrusion-angle", po::value<double>()->required(), "The maximal tolerated angle (radians) between a boundary vertex and the two boundary vertices which are distant roughly twice the mean edge length from each other.")
		("maximal-angle", po::value<double>()->required(), "Maximal tolerated dihedral angles in radians.")
		("minimal-angle", po::value<double>()->required(), "Minimal tolerated dihedral angles in radians.")
		("maximal-mean-curv", po::value<double>()->required(), "Maximal tolerated magnitude of the vertex mean curvature mm^-1.")
		("minimal-mean-curv", po::value<double>()->required(), "Minimal tolerated magnitude of the vertex mean curvature mm^-1.")
		("maximal-gauss-curv", po::value<double>()->required(), "Maximal tolerated magnitude of the vertex mean curvature mm^-1.")
		("minimal-gauss-curv", po::value<double>()->required(), "Minimal tolerated magnitude of the vertex mean curvature mm^-1.")
		("maximal-mean-curv-smooth", po::value<double>()->required(), "Maximal tolerated magnitude of the smoothed mean curvature of a vertex in mm^-1.")
		("minimal-mean-curv-smooth", po::value<double>()->required(), "Minimal tolerated magnitude of the smoothed mean curvature of a vertex in mm^-1.")
		("maximal-gauss-curv-smooth", po::value<double>()->required(), "Maximal tolerated magnitude of the smoothed Gauss curvature of a vertex in mm^-2.")
		("minimal-gauss-curv-smooth", po::value<double>()->required(), "Minimal tolerated magnitude of the smoothed Gauss curvature of a vertex in mm^-2.")

		("min-triangle", po::value<double>()->required(), "Specifies the minimal triangle aspect ratio which is still tolerated. Any other triangles are resolved if the thin-triangles parameter is set to ON.")
		("max-refinement", po::value<int>(&max_refinement)->required(), "The maximal number of refinement optimization steps, high number causes long lasting filling of large holes (warning can be produced), very small number results in inappropriate hole shapes.");

	// all other options are supposed to be input files
	po::options_description hidden("Hidden options");
	hidden.add_options()("input-file", po::value<std::vector<std::string>>(), "input file(s)");

	// bind defined options and parameters
	po::options_description cmdline_options;
	cmdline_options.add(generic).add(config).add(hidden);
	po::options_description config_file_options;
	config_file_options.add(config).add(params);
	po::options_description visible("Allowed options");
	visible.add(generic).add(config);
	po::positional_options_description p;
	p.add("input-file", -1);

	try {
		// parse command line options
		store(po::command_line_parser(argc, argv).options(cmdline_options).positional(p).run(), vm);
		notify(vm);

		// handle basic options
		if (vm.count("version")) {
			std::cout << "Automated Face Cleaning " << version << '\n';
			return SUCCESS;
		}
		if (vm.count("help")) {
			std::cout << visible << '\n';
			return SUCCESS;
		}
		if (vm.count("manual")) {
			std::cout << params << '\n';
			return SUCCESS;
		}

		// open and parse config file
		bf::path config_file_p(config_file);
		if (config_file_p.is_relative()) {
			config_file_p = exec_base / config_file_p;
		}
		std::ifstream ifs(config_file_p.c_str(), std::ios::binary);
		if (!ifs) {
			std::cerr << "Exiting: cannot open the configuration file: " << config_file_p.c_str() << '\n';
			std::cout << "To display the list of available options, please use --help" << '\n';
			return ERROR_IN_COMMAND_LINE;
		}
		store(parse_config_file(ifs, config_file_options), vm);
		notify(vm);
	}
	catch (po::required_option& e) {
		std::cerr << "Exiting: missing a required option or parameter, " << e.what() << '\n';
		std::cerr << "Required parameters which should be present in config file:" << '\n' << params << '\n';
		return ERROR_IN_COMMAND_LINE;
	}
	catch (po::error& e) {
		std::cerr << "Exiting: error while reading parameters and command line arguments, " << e.what() << '\n';
		return ERROR_IN_COMMAND_LINE;
	}

	//
	// validation of read parameters:
	//

	if (verbosity > 3 || verbosity < 0) {
		std::cerr << "Exiting: weird verbosity level, please use --help\n";
		return ERROR_IN_COMMAND_LINE;
	}
	if (threads < 0) {
		std::cerr << "Exiting: weird number of threads, please use --help\n";
		return ERROR_IN_COMMAND_LINE;
	}
	if (destination_path != "." && !bf::is_directory(destination_path)) {
		std::cerr << "Exiting: the destination path is invalid, path specified: " << destination_path << '\n';
		return ERROR_IN_COMMAND_LINE;
	}
	if (file_suffix.empty()) {
		std::cerr << "Exiting: the suffix of processed files cannot be empty string\n";
		return ERROR_IN_COMMAND_LINE;
	}
	if (log_file.empty()) {
		std::cerr << "Exiting: the log_file path cannot be empty string\n";
		return ERROR_IN_COMMAND_LINE;
	}
	if (landmark_suffix.empty()) {
		std::cerr << "Exiting: the suffix of landmark files cannot be empty string\n";
		return ERROR_IN_COMMAND_LINE;
	}
	if (max_refinement <= 0) {
		std::cerr << "Exiting: max_refinement_iters argument must be a positive number\n";
		return ERROR_IN_COMMAND_LINE;
	}
	if (vm["scale"].as<double>() <= 0.0) { std::cerr << "Exiting: scale argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["e2e-mean"].as<double>() <= 0.0) { std::cerr << "Exiting: e2e-mean argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["e2e-dev"].as<double>() < 0.0) { std::cerr << "Exiting: e2e-dev argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["m2m-mean"].as<double>() <= 0.0) { std::cerr << "Exiting: m2m-mean argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["m2m-dev"].as<double>() < 0.0) { std::cerr << "Exiting: m2m-dev argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["n2r-mean"].as<double>() <= 0.0) { std::cerr << "Exiting: n2r-mean argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["n2r-dev"].as<double>() < 0.0) { std::cerr << "Exiting: n2r-dev argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }

	if (vm["n2e-over-n2r-mean"].as<double>() < 0.0) { std::cerr << "Exiting: n2e-over-n2r-mean argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["n2e-over-n2r-dev"].as<double>() < 0.0) { std::cerr << "Exiting: n2e-over-n2r-dev argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["r2e-over-e2e-mean"].as<double>() < 0.0) { std::cerr << "Exiting: r2e-over-e2e-mean argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["r2e-over-e2e-dev"].as<double>() < 0.0) { std::cerr << "Exiting: r2e-over-e2e-dev argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["r2m-over-n2r-mean"].as<double>() < 0.0) { std::cerr << "Exiting: r2m-over-n2r-mean argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["r2m-over-n2r-dev"].as<double>() < 0.0) { std::cerr << "Exiting: r2m-over-n2r-dev argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["n2m-over-m2m-mean"].as<double>() < 0.0) { std::cerr << "Exiting: n2m-over-m2m-mean argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["n2m-over-m2m-dev"].as<double>() < 0.0) { std::cerr << "Exiting: n2m-over-m2m-dev argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }

	if (vm["far"].as<double>() <= 0.0) { std::cerr << "Exiting: far argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["near"].as<double>() <= 0.0) { std::cerr << "Exiting: near argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["n2ch-over-n2r-mean"].as<double>() <= 0.0) { std::cerr << "Exiting: n2ch-over-n2r-mean argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["n2ch-over-n2r-dev"].as<double>() < 0.0) { std::cerr << "Exiting: n2ch-over-n2r-dev argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["neck-normal-threshold"].as<double>() < 0.0) { std::cerr << "Exiting: neck-normal-threshold argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["ears-normal-threshold"].as<double>() < 0.0) { std::cerr << "Exiting: ears-normal-threshold argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["neck-angle-threshold"].as<double>() < 0.0) { std::cerr << "Exiting: eck-angle-threshold argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["ears-angle-threshold"].as<double>() < 0.0) { std::cerr << "Exiting: ears-angle-threshold argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["wide-protrusion-range"].as<double>() < 0.0) { std::cerr << "Exiting: wide-protrusion-range argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["wide-protrusion-angle"].as<double>() < 0.0) { std::cerr << "Exiting: wide-protrusion-angle argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["tiny-protrusion-angle"].as<double>() < 0.0) { std::cerr << "Exiting: tiny-protrusion-angle argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }

	if (vm["maximal-angle"].as<double>() < 0.0) { std::cerr << "Exiting: maximal-angle argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["minimal-angle"].as<double>() < 0.0) { std::cerr << "Exiting: minimal-angle argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["min-triangle"].as<double>() < 0.0 || vm["min-triangle"].as<double>() > 1.0) { std::cerr << "Exiting: min-triangle argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	if (vm["max-refinement"].as<int>() <= 0) { std::cerr << "Exiting: max-refinement argument must be a positive number\n"; return ERROR_IN_COMMAND_LINE; }
	
	return SUCCESS;
}

int main(int argc, const char* argv[]) {

	try {
		// location of the program's executable 
		bf::path exec_base(bf::initial_path<bf::path>());
		exec_base = bf::system_complete(bf::path(argv[0])).parent_path();
		std::string app_name = boost::filesystem::basename(argv[0]);

		// object which will keep all parsed options and parameters
		po::variables_map vm;
		// some other results of parsing
		int threads, verbosity, max_refinement;
		std::string log_file, file_suffix, destination_path, landmark_suffix;
		// catch 'em all
		auto parse_result = parse_settings(argc, argv, exec_base, vm, verbosity, threads, max_refinement, log_file, file_suffix, destination_path, landmark_suffix);
		if (parse_result != SUCCESS) return parse_result;

		// set maximal number of threads
		Eigen::initParallel();
		if (threads != 0) {
			omp_set_num_threads(threads);		
			Eigen::setNbThreads(threads);
		}

		// open and set log file
		std::ofstream log_stream(log_file, std::ios::out | std::ios::binary);
		if (!log_stream) {
			std::cerr << "Exiting: log file could not be opened, file specified: " << log_file << '\n';
			return ERROR_IN_COMMAND_LINE;
		}
		LOGCFG = {true, static_cast<typelog>(verbosity), &log_stream};

		//
		// initialize instances of PointClassifier, HoleFiller and Cleaner	

		bf::path nose_tip_p(vm["nose-tip-model"].as<std::string>());
		bf::path nose_root_p(vm["nose-root-model"].as<std::string>());
		bf::path eye_corner_p(vm["eye-corner-model"].as<std::string>());
		bf::path mouth_corner_p(vm["mouth-corner-model"].as<std::string>());

		if (nose_tip_p.is_relative()) nose_tip_p = exec_base / nose_tip_p;
		if (nose_root_p.is_relative()) nose_root_p = exec_base / nose_root_p;
		if (eye_corner_p.is_relative()) eye_corner_p = exec_base / eye_corner_p;
		if (mouth_corner_p.is_relative()) mouth_corner_p = exec_base / mouth_corner_p;

		PointClassifier pc(vm, nose_tip_p.string(), nose_root_p.string(), eye_corner_p.string(), mouth_corner_p.string(), threads);
		if (!pc.load_models()) {
			std::cerr << "Exiting: cannot load classification models, check existence or paths\n";
			return ERROR_IN_COMMAND_LINE;
		}

		HoleFiller hf(max_refinement);
		Cleaner cl(&hf, &pc, vm);

		// create instances of mesh importers / exporters
		WavefrontObjectFiles obj;
		PolygonFileFormat ply;
		IModelHandler* mh;

		// starting message with some summaries
		std::cout << std::string(80, '=') << '\n';
		std::cout << "||  Running Automated Face Cleaning v" << version << ", 2018 by Tomas Nekvinda" << '\n';
		std::vector<std::string> input_files;
		if (vm.count("input-file")) {
			input_files = vm["input-file"].as<std::vector<std::string> >();
			if (input_files.size() == 0) {
				std::cout << "||  No input files!" << '\n';
				std::cout << std::string(80, '=') << '\n';
				return SUCCESS;
			}
			else {
				std::cout << "||  Number of input files:  " << input_files.size() << '\n';
			}
		}
		std::cout << "||  Cleaned files' suffix:  " << file_suffix << '\n';
		std::cout << "||  Destination path:       " << destination_path << '\n';
		std::cout << "||  Log file path:          " << log_file << '\n';
		std::cout << "||  Number of threads used: " << omp_get_num_procs() << '\n';
		std::cout << std::string(80, '=') << '\n';

		// variables tracking progress of the program
		boost::posix_time::ptime start_time = boost::posix_time::second_clock::local_time();
		size_t processed = 0;
		size_t skipped = 0;

		// process all input files
		for (auto&& file : input_files) {

			Scene scene;
			// check the input file existence
			if (!bf::exists(file)) {
				LOG(ERR) << "Non-existing input file: " << file;
				++skipped;
				continue;
			}
			boost::filesystem::path p(file);
			boost::filesystem::path base = p.parent_path();
			std::string filename = p.stem().string();
			std::string extension = p.extension().string();

			// determine input file format and select the right handler
			//
			// TO ADD SUPPORT FOR NEW FILE FORMAT, MODIFY THIS PART !!!
			//
			if (extension == ".obj")      mh = &obj;
			else if (extension == ".ply") mh = &ply;
			else {
				LOG(ERR) << "Not a valid input file extension: " << extension << " provide .obj or .ply file format";
				++skipped;
				continue;
			}

			// open input file
			std::ifstream infile(file, std::ios::binary);
			if (!infile) {
				LOG(ERR) << "Cannot open an input file: " << file;
				++skipped;
				continue;
			}
	
			// destination path -- new meshes and landmarks are saved in there
			std::ofstream outfile;
			boost::filesystem::path destination;
			if (destination_path == ".") destination = base;
			else destination = boost::filesystem::path(destination_path);

			// open output file
			auto real_destination = (destination / (filename + file_suffix + extension)).string();
			outfile.open(real_destination, std::ios::out | std::ios::binary);
			if (!outfile) {
				LOG(ERR) << "Cannot open an output file: " << real_destination;
				++skipped;
				continue;
			}

			// load the input mesh
			LOG(INF) << "Loading: " << file;
			if (!mh->load(infile, scene)) {
				LOG(ERR) << "Cannot load an input file: " << file;
				++skipped;
				continue;
			}

			//
			// RUN CLEANING
			cl.process(scene);

			// save the cleaned mesh
			LOG(INF) << "Saving: " << real_destination;
			if (!mh->save(outfile, scene)) {
				LOG(ERR) << "Cannot save an output file: " << real_destination;
				++skipped;
				continue;
			}
			outfile.close();

			// save landmarks
			if (vm["save-landmarks"].as<bool>()) {

				// open landmarks file
				std::ofstream outfile_landmarks;
				auto landmarks_destination = (destination / (filename + landmark_suffix + ".txt")).string();
				outfile_landmarks.open(landmarks_destination, std::ios::out | std::ios::binary);
				LOG(INF) << "Saving landmarks into a file: " << landmarks_destination;
				if (!outfile_landmarks) {
					LOG(ERR) << "Cannot open landmarks file: " << landmarks_destination;
					++skipped;
					continue;
				}

				// flush detected landmarks
				Vertex * left_eye, *right_eye, *left_lips, *right_lips, *nose, *root;
				auto pc_result = pc.get_face_landmarks(left_eye, right_eye, left_lips, right_lips, nose, root, scene);
				if (pc_result == PointClassifier::ALL) {
					outfile_landmarks << nose->idx() << '\n';
					outfile_landmarks << root->idx() << '\n';
					outfile_landmarks << left_eye->idx() << '\n';
					outfile_landmarks << right_eye->idx() << '\n';
					outfile_landmarks << left_lips->idx() << '\n';
					outfile_landmarks << right_lips->idx() << '\n';
				}
				else if (pc_result == PointClassifier::NOSE) {
					outfile_landmarks << nose->idx();
				}

				outfile_landmarks.close();
			}

			++processed;
		}

		boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
		boost::posix_time::time_duration diff = now - start_time;

		// print ending message
		std::cout << std::string(80, '=') << '\n';
		std::cout << "||  Cleaning completed!" << '\n';
		std::cout << "||  Processing took:   " << diff.total_seconds() << " seconds in total" << '\n';
		std::cout << "||  Faces processed:   " << processed << '\n';
		std::cout << "||  Faces skipped:     " << skipped << '\n';
		std::cout << "||  See log file:      " << log_file << '\n';
		std::cout << std::string(80, '=') << '\n';
	}
	catch (std::exception& e) {
		std::cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit" << std::endl;
		return ERROR_UNHANDLED_EXCEPTION;
	}

	return SUCCESS;
}