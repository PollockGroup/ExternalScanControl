/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                                                             *
 * ExternalScan.h                                              *
 *                                                             *
 * Created by: William C. Lenthe,                              *
 * Copyright (c) 2016 University of California, Santa Barbara  *
 * All Rights Reserved                                         *
 *                                                             *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef _ExternalScan_H_
#define _ExternalScan_H_

#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include <stdexcept>
#include <ctime>

#include "tif.hpp"

#ifndef NOMINMAX
	#define NOMINMAX//windows min/max definitions conflict with std
#endif
#ifndef WIN32_LEAN_AND_MEAN
	#define WIN32_LEAN_AND_MEAN//limit extra windows includes
#endif
#include <windows.h>
#include "NIDAQmx.h"

class ExternalScan {
	private:
		std::string xPath, yPath;                     //path to analog output channels for scan control
		std::string etdPath;                          //path to analog input channel for etd
		uInt64 samples;                               //samples per pixel (collection occurs at fastest possible speed)
		float64 vRange;                               //voltage range for scan (scan will go from -vRange -> +vRange in the larger direction)
		uInt64 width, height;                         //dimensions of the scan
		bool snake;                                   //true/false to snake/raster
		TaskHandle hInput, hOutput;                   //handles to input and output tasks
		float64 sampleRate;                           //maximum device sample rate
		uInt64 jRow;                                  //current row being collected
		std::vector<int16> buffer;                    //working array to read rows from device buffer
		std::vector< std::vector<int16> > frameImages;//working array to hold entire image

		//@brief: check a DAQmx return code and convert to an exception if needed
		//@param error: return code to check
		//@param message (optional): description of action being attempted to include in exception
		void DAQmxTry(int32 error, std::string message = std::string());

		//@brief: generate interleaved x/y voltages for scan
		//@return: scan data
		std::vector<float64> generateScanData() const;

		//@brief: check scan parameters, configure DAQmx tasks, and write scan pattern to buffer
		void configureScan();

		//@brief: stop and clear configured tasks
		void clearScan();

		//@brief: read row of raw data from buffer (large images with many samples may be too large to hold in the device buffer)
		int32 readRow();

	public:
		static int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void *callbackData) {return reinterpret_cast<ExternalScan*>(callbackData)->readRow();}

		ExternalScan(std::string x, std::string y, std::string e, uInt64 s, float64 a, uInt64 w, uInt64 h, bool sn) : xPath(x), yPath(y), etdPath(e), samples(s), vRange(a), width(w), height(h), snake(sn) {configureScan();}
		~ExternalScan() {clearScan();}

		//@brief: collect and image with the current parameter set and write to disk
		void execute(std::string fileName);
};

void ExternalScan::DAQmxTry(int32 error, std::string message) {
	if(0 != error) {
		//get error message
		int32 buffSize = DAQmxGetExtendedErrorInfo(NULL, 0);
		if(buffSize < 0) buffSize = 8192;//sometimes the above returns an error code itself
		std::vector<char> buff(buffSize, 0);
		DAQmxGetExtendedErrorInfo(buff.data(), (uInt32)buff.size());

		//stop and clear tasks and throw
		if(0 != hInput) {
			DAQmxStopTask(hInput);
			DAQmxClearTask(hInput);
		}
		if(0 != hOutput) {
			DAQmxStopTask(hOutput);
			DAQmxClearTask(hOutput);	
		}
		if(message.empty())
			throw std::runtime_error("NI-DAQmx error " + std::to_string(error) + ":\n" + std::string(buff.data()));
		else
			throw std::runtime_error("NI-DAQmx error " + message + " " + std::to_string(error) + ":\n" + std::string(buff.data()));
	}
}

std::vector<float64> ExternalScan::generateScanData() const {
	//generate uniformly spaced square grid of points from -vRange -> vRange in largest direction
	std::vector<float64> xData((size_t)width), yData((size_t)height);
	std::iota(xData.begin(), xData.end(), 0.0);
	std::iota(yData.begin(), yData.end(), 0.0);
	float64 scale = std::max(xData.back(), yData.back());
	std::for_each(xData.begin(), xData.end(), [scale](float64& v){v /= scale;});//0->100% of scan size
	std::for_each(yData.begin(), yData.end(), [scale](float64& v){v /= scale;});//0->100% of scan size
	scale = xData.back() / 2;
	std::for_each(xData.begin(), xData.end(), [scale](float64& v){v -= scale;});//make symmetric about 0
	scale = yData.back() / 2;
	std::for_each(yData.begin(), yData.end(), [scale](float64& v){v -= scale;});//make symmetric about 0
	scale = 2.0 * vRange;	
	std::for_each(xData.begin(), xData.end(), [scale](float64& v){v *= scale;});//scale so limits are +/- vRange
	std::for_each(yData.begin(), yData.end(), [scale](float64& v){v *= scale;});//scale so limits are +/- vRange
	std::reverse(xData.begin(), xData.end());//+x voltage is left side of image

	//generate single pass scan
	std::vector<float64> scan;
	const uInt64 scanPixels = width * height;
	scan.reserve(2 * (size_t)scanPixels);
	if(snake) {
		for(uInt64 i = 0; i < height; i++) {
			if(i % 2 == 0)
				scan.insert(scan.end(), xData. begin(), xData. end());
			else
				scan.insert(scan.end(), xData.rbegin(), xData.rend());
		}
		for(uInt64 i = 0; i < height; i++) scan.insert(scan.end(), (size_t)width, yData[(size_t)i]);
	} else {
		for(uInt64 i = 0; i < height; i++) scan.insert(scan.end(), xData.begin() , xData.end()     );
		for(uInt64 i = 0; i < height; i++) scan.insert(scan.end(), (size_t)width , yData[(size_t)i]);
	}
	return scan;
}

void ExternalScan::configureScan() {
	//create tasks and channels
	clearScan();//clear existing scan if needed
	DAQmxTry(DAQmxCreateTask("scan generation", &hOutput), "creating output task");
	DAQmxTry(DAQmxCreateTask("etd reading", &hInput), "creating input task");
	DAQmxTry(DAQmxCreateAOVoltageChan(hOutput, (xPath + "," + yPath).c_str(), "", -vRange, vRange, DAQmx_Val_Volts, NULL), "creating output channel");
	DAQmxTry(DAQmxCreateAIVoltageChan(hInput, etdPath.c_str(), "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL), "creating input channel");

	//get the maximum anolog input rate supported by the daq
	DAQmxTry(DAQmxGetSampClkMaxRate(hInput, &sampleRate), "getting device maximum input frequency");
	const float64 effectiveDwell = (1000000.0 * samples) / sampleRate;

	//check scan rate, the microscope is limited to a 300 ns dwell at 768 x 512
	//3.33 x factor of safety -> require at least 768 us to cover full -4 -> +4 V scan
	const float64 minDwell = (768.0 / width) * (4.0 / vRange);//minimum dwell time in us
	if(effectiveDwell < minDwell) throw std::runtime_error("Dwell time too short - dwell must be at least " + std::to_string(minDwell) + " us for " + std::to_string(width) + " pixel scan lines");

	//configure timing
	const uInt64 scanPoints = width * height;
	DAQmxTry(DAQmxCfgSampClkTiming(hOutput, "", sampleRate / samples, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, scanPoints), "configuring output timing");

	//configure device buffer / data transfer
	const uInt64 rowDataPoints = width * samples;
	uInt64 bufferSize = 4 * rowDataPoints;//allocate buffer big enough to hold 4 rows of data
	DAQmxTry(DAQmxSetBufInputBufSize(hInput, (uInt32)bufferSize), "set buffer size");
	DAQmxTry(DAQmxCfgSampClkTiming(hInput, "", sampleRate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, bufferSize), "configuring input timing");
	DAQmxRegisterEveryNSamplesEvent(hInput, DAQmx_Val_Acquired_Into_Buffer, (uInt32)(samples * width), 0, ExternalScan::EveryNCallback, reinterpret_cast<void*>(this));

	//configure start triggering
	std::string trigName = "/" + xPath.substr(0, xPath.find('/')) + "/ai/StartTrigger";//use output trigger to start input
	DAQmxTry(DAQmxCfgDigEdgeStartTrig(hOutput, trigName.c_str(), DAQmx_Val_Rising), "setting start trigger");

	//write scan data to device buffer
	int32 written;
	std::vector<float64> scan = generateScanData();
	DAQmxTry(DAQmxWriteAnalogF64(hOutput, (int32)scanPoints, FALSE, DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByChannel, scan.data(), &written, NULL), "writing scan to buffer");
	if(scanPoints != written) throw std::runtime_error("failed to write all scan data to buffer");

	//allocate arrays to hold single row of data points and entire image
	buffer.assign((size_t)rowDataPoints, 0);
	frameImages.assign((size_t)samples, std::vector<int16>((size_t)width * height));//hold each frame as one block of memory
}

void ExternalScan::clearScan() {
	if(NULL != hInput) {
		DAQmxStopTask(hInput);
		DAQmxClearTask(hInput);
		hInput = NULL;
	}
	if(NULL != hOutput) {
		DAQmxStopTask(hOutput);
		DAQmxClearTask(hOutput);
		hOutput = NULL;
	}
}

int32 ExternalScan::readRow() {
	int32 read;
	DAQmxTry(DAQmxReadBinaryI16(hInput, (int32)buffer.size(), DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByChannel, buffer.data(), (uInt32)buffer.size(), &read, NULL), "reading data from buffer");
	if(jRow >= height) return 0;//input is continuous so samples will be collected after the scan is complete
	std::cout << "\rcompleted row " << jRow+1 << "/" << height;
	if(buffer.size() != read) throw std::runtime_error("failed to read all scan data from buffer");
	if(snake && jRow % 2 == 1) {
		const uInt64 rowOffset = width * jRow + width - 1;//offset of row end
		for(uInt64 k = 0; k < samples; k++)
			for(uInt64 i = 0; i < width; i++)
				frameImages[(size_t)k][(size_t)(rowOffset-i)] = buffer[(size_t)(i*samples+k)];
	} else {
		const uInt64 rowOffset = width * jRow;//offset of row start
		for(uInt64 k = 0; k < samples; k++) {
			for(uInt64 i = 0; i < width; i++) {
				frameImages[(size_t)k][(size_t)(rowOffset+i)] = buffer[(size_t)(i*samples+k)];
			}
		}
	}
	++jRow;
	return 0;
}

/*
template <typename T>
std::vector<std::uint8_t> to8Bit(const std::vector<T>& image) {
	//build histogram bins
	T bins[256];
	std::iota(bins, bins+256, 1);//fill bins with 1, 2, 3, ..., 256
	auto minMax = std::minmax_element(image.cbegin(), image.cend());
	const T vMin = *minMax.first;
	const T vMax = *minMax.second;
	const double binWidth = double(vMax - vMin) / 256;
	std::transform(bins, bins+256, bins, [binWidth, vMin](const T& v){return (T)std::round(binWidth * v) + vMin;});
	bins[255] = vMax;//rounding error can cause brightest pixel to be black

	//compute histogram bin for each pixel
	std::vector<std::uint8_t> pixelBins(image.size());
	for(size_t i = 0; i < image.size(); i++) pixelBins[i] = std::distance(bins, std::lower_bound(bins, bins+256, image[i]));
	return pixelBins;
}
*/

void ExternalScan::execute(std::string fileName) {
	//execute scan
	jRow = 0;
	DAQmxTry(DAQmxStartTask(hOutput), "starting output task");
	DAQmxTry(DAQmxStartTask(hInput), "starting input task");
	
	//wait for scan to complete
	float64 scanTime = float64(width * height * samples) / sampleRate + 5.0;//allow an extra 5s
	std::cout << "imaging (expected duration ~" << scanTime - 5.0 << "s)\n";
	DAQmxTry(DAQmxWaitUntilTaskDone(hOutput, scanTime), "waiting for output task");
	Sleep((DWORD)(1 + (1000 * samples) / sampleRate));//give the input task enough time to be sure that it is finished
	DAQmxTry(DAQmxStopTask(hInput), "stopping input task");
	std::cout << '\n';

	//write image to file
	Tif::Write(frameImages, (uInt32)width, (uInt32)height, fileName);

	//to do real time alignment
	/*
	std::vector<float> shifts;
	std::vector<float> avgFrame = correlateRows<float, int16>(frames, (int)height, (int)width, shifts, 5.0f, 16, &snake);
	Tif::Write(to8Bit(avgFrame), width, height, fileName + "_avg.tif");
	*/
}

#endif