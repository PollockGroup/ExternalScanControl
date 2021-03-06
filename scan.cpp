/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                                                                                 *
 * Copyright (c) 2017, Reagents of the University of California                    *
 * All rights reserved.                                                            *
 * Author William C. Lenthe                                                        *
 *                                                                                 *
 * Redistribution and use in source and binary forms, with or without              *
 * modification, are permitted provided that the following conditions are met:     *
 *                                                                                 *
 * 1. Redistributions of source code must retain the above copyright notice, this  *
 *    list of conditions and the following disclaimer.                             *
 *                                                                                 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,    *
 *    this list of conditions and the following disclaimer in the documentation    *
 *    and/or other materials provided with the distribution.                       *
 *                                                                                 *
 * 3. Neither the name of the copyright holder nor the names of its                *
 *    contributors may be used to endorse or promote products derived from         *
 *    this software without specific prior written permission.                     *
 *                                                                                 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"     *
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE       *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE  *
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE    *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL      *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR      *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,   *
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.            *
 *                                                                                 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <iostream>
#include <fstream>

#include "ExternalScan.h"

int main(int argc, char *argv[]) {
	try {
		//optional arguments
		bool snake = true;
		std::string timeLog;
		uInt64 width = 4096, height = 4096;

		//build help string
		std::stringstream ss;
		ss << "usage: " + std::string(argv[0]) + " -x path -y path -e path -s samples -a voltage -o file [-w width] [-h height] [-r] [-t file]\n";
		ss << "\t -x : path to X analog out channel (e.g. 'Dev0/ao0')\n";
		ss << "\t -y : path to Y analog out channel\n";
		ss << "\t -e : path to ETD analog in channel\n";
		ss << "\t -s : samples per pixel\n";
		ss << "\t -a : half amplitude of scan in volts\n";
		ss << "\t -o : output image name (tif format)\n";
		ss << "\t[-w]: scan width in pixels (defaults to " << width << ")\n";
		ss << "\t[-h]: scan height in pixels (defaults to " << height << ")\n";
		ss << "\t[-r]: scan in a raster instead of snake pattern\n";
		ss << "\t[-t]: append image acquisitions times to log file\n";

		//required arguments
		std::string xPath, yPath, ePath, output;
		float64 scanVoltage = 0.0;
		uInt64 samples = 2;

		//parse arguments
		for(int i = 1; i < argc; i++) {
			//make sure flag(s) exist and start with a '-'
			const size_t flagCount = strlen(argv[i]) - 1;
			if('-' != argv[i][0] || flagCount == 0) throw std::runtime_error(std::string("unknown option: ") + argv[i]);

			//parse each option in this group
			for(size_t j = 0; j < flagCount; j++) {
				//check if this flag has a corresponding argument and make sure that argument exists
				bool requiresOption = true;
				switch(argv[i][j+1]) {
					case 'r':
						requiresOption = false;
				}
				if(requiresOption && (i+1 == argc || j+1 != flagCount)) throw std::runtime_error(std::string("missing argument for option ") + argv[i][j]);

				switch(argv[i][j+1]) {
					case 'x': xPath       = std::string(argv[i+1]); break;
					case 'y': yPath       = std::string(argv[i+1]); break;
					case 'e': ePath       = std::string(argv[i+1]); break;
					case 's': samples     = atoi(argv[i+1]); break;
					case 'a': scanVoltage = atof(argv[i+1]); break;
					case 'o': output      = std::string(argv[i+1]); break;
					case 'w': width       = atoi(argv[i+1]); break;
					case 'h': height      = atoi(argv[i+1]); break;
					case 'r': snake       = false; break;
					case 't': timeLog     = std::string(argv[i+1]); break;
				}
				if(requiresOption) ++i;//double increment if the next agrument isn't a flag
			}
		}

		//make sure required arguments were passed
		if(xPath.empty()) throw std::runtime_error(ss.str() + "(x flag missing)");
		if(yPath.empty()) throw std::runtime_error(ss.str() + "(y flag missing)");
		if(ePath.empty()) throw std::runtime_error(ss.str() + "(e flag missing)");
		if(output.empty()) throw std::runtime_error(ss.str() + "(o flag missing)");
		if(0.0 == scanVoltage) throw std::runtime_error(ss.str() + "(a flag missing or empty)");
		
		//create scan opject
		ExternalScan scan(xPath, yPath, ePath, samples, scanVoltage, width, height, snake);

		//execute scan and write image
		std::time_t start = std::time(NULL);
		scan.execute(output);
		std::time_t end = std::time(NULL);

		//append time stamps to log if needed
		if(!timeLog.empty()) {
			//check if log file already exists
			std::ifstream is(timeLog);
			bool exists = is.good();
			is.close();

			//write time stamp to time stamp log
			std::ofstream of(timeLog, std::ios_base::app);
			if(!exists) of << "filename\timage start\timage start (unix)\timage end\timage end (unix)\n";//write header on first entry
			of << output << "\t" << std::asctime(std::localtime(&start)) << "\t" << start << "\t" << std::asctime(std::localtime(&end)) << "\t" << end << "\n";
		}
	} catch (std::exception& e) {
		std::cout << e.what();
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}