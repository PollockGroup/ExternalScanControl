/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                                                                                 *
 * Copyright (c) 2017, William C. Lenthe                                           *
 * All rights reserved.                                                            *
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

#include <filesystem>
#include <iostream>
#include <fstream>
#include <cmath>

#include "tif.hpp"
#include "xcorr.hpp"

//@breif: convert an image to 8 bit
//@param image: image to convert
//@return: converted image
template <typename T> std::vector<std::uint8_t> to8Bit(const std::vector<T>& image) {
	//build histogram bins
	T bins[256];
	std::iota(bins, bins+256, T(1));//fill bins with 1, 2, 3, ..., 256
	auto minMax = std::minmax_element(image.cbegin(), image.cend());
	const T vMin = *minMax.first;
	const T vMax = *minMax.second;
	const double binWidth = double(vMax - vMin) / 256;
	std::transform(bins, bins+256, bins, [binWidth, vMin](const T& v){return (T)std::round(binWidth * v) + vMin;});
	bins[255] = vMax;//rounding error can cause brightest pixel to be black

	//compute histogram bin for each pixel
	std::vector<std::uint8_t> pixelBins(image.size());
	for(size_t i = 0; i < image.size(); i++) pixelBins[i] = (uint8_t)std::distance(bins, std::lower_bound(bins, bins+256, image[i]));
	return pixelBins;
}

//@breif: get all files in current directory with specified extension
//@param ext: file extension
//@return: list of file names
std::vector<std::string> getFiles(std::string ext = "*") {
	std::vector<std::string> fileList;
	std::experimental::filesystem::path p = std::experimental::filesystem::current_path();
	bool any = 0 == ext.compare("*");
	for(auto entry : std::experimental::filesystem::directory_iterator(p))
		if(any || ext == entry.path().extension())
			fileList.push_back(entry.path().stem().string());
	return fileList;
}

int main(int argc, char **argv) {
	try {
		const bool saveCorrected = true;//flag to save corrected time resolved images
		typedef uint16_t PixelType;//input pixel type (narrower types will be silently cast up)
		typedef float RealType;//fft type (wider types will give more precision on fft shift but are slower)
		
		//get all tifs in the current directory
		std::vector<std::string> files = getFiles(".tif");
		std::string outputDir = "corrected/";

		//create output directory and log file
		std::experimental::filesystem::create_directory(std::experimental::filesystem::current_path().append(outputDir));
		std::ofstream os(outputDir + "shifts.txt");
		os << "filename\tgomp_b\tgomp_c\tt0_shift\tt1_shift\t...\n";

		//loop over files
		for(size_t i = 0; i < files.size(); i++) {			
			//read data and print update
			uint32_t width, height;
			std::cout << "\raligning: " << files[i] << "       ";
			std::vector< std::vector<PixelType> > frames = Tif::Read<PixelType>(files[i] + ".tif", width, height);

			//create x coordinates for gompertz fit
			std::vector<RealType> x(frames.size());
			std::iota(x.begin(), x.end(), RealType(0));//create evenly spaced list of collection times (scale doesn't matter)

			//compute / apply shift
			std::vector<RealType> shifts;
			try {
				//compute shift + integrate corrected image
				ScanType scanType = ScanType::Unknown;//compute scan type from first frame (can specify Snake/Raster to impose)
				const RealType maxShift(5);
				const int subpixelResolution = (16);
				std::vector<RealType> avgFrame = correlateRows<RealType, PixelType>(frames, (int)height, (int)width, shifts, scanType, maxShift, subpixelResolution);//compute shifts

				//compute least square gompertz fit and write to log file
				Gompertz<RealType> gomp(x, shifts);//compute least squares fit gompertz function
				os << files[i] << '\t' << gomp.b << '\t' << gomp.c;
				
				//write shifts to log file
				for(auto& d : shifts) os << '\t' << d;
				os << std::endl;

				//write integrated images
				Tif::Write(       avgFrame , width, height, outputDir + files[i] +      ".tif");//as real
				Tif::Write(to8Bit(avgFrame), width, height, outputDir + files[i] + "_8bit.tif");//as 8 bit (some software doesn't accept fp images)

				//save individual aligned time steps as tif stack in addition to integrated result if needed
				if(saveCorrected) {
					alignRows<RealType, PixelType>(frames, (int)height, (int)width, shifts, scanType);//apply computed shifts to original image (could use gompertz fit values instead)
					Tif::Write(frames, width, height, outputDir + files[i] + "_cor.tif");//write result to file (as PixelType)
				}
			} catch (std::exception& e) {
				//don't let an issue with a single image disrupt the entire run
				os << files[i];
				for(auto& d : shifts) os << '\t' << d;
				os << std::endl;
				std::cout << ": failed:\n" << e.what() << '\n';
			}
		}
	} catch(std::exception& e) {
		std::cout << e.what();
		return 1;
	}
	std::cout << '\n';
	return 0;
}