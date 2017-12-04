#include <iostream>
#include "ScanControl.h"
#include <sstream>

template <typename T>
void getNonBlank(T& t, std::istream& is = std::cin) {
    std::string str;
    std::getline(is, str);
    if(!str.empty()) std::istringstream(str) >> t;
}

int main(){
    try {
        //define default parameters
        double etdMin = -1.75, etdMax = 3.0;
        size_t dwellSamples = 40, dim = 4096, dir = 1, type = 1;
        std::string pathX = "Dev1/ao2";
        std::string pathY = "Dev1/ao3";
        std::string pathETD = "Dev1/ai2";


        //get user input and create scan control object
        std::cout << "leave fields blank to use [default option]\n";
        std::cout << "X path [" << pathX << "]: ";
        getNonBlank(pathX);
        std::cout << "Y path [" << pathY << "]: ";
        getNonBlank(pathY);
        std::cout << "ETD path [" << pathETD << "]: ";
        getNonBlank(pathETD);
        ScanControl scan(pathX, pathY, pathETD);

        //get imaging conditions
        std::cout << "etd min (V) [" << etdMin << "]: ";
        getNonBlank(etdMin);
        std::cout << "etd max (V) [" << etdMax << "]: ";
        getNonBlank(etdMax);
        std::cout << "image width (pixels) [" << dim << "]: ";
        getNonBlank(dim);
        std::cout << "dwell samples (" << scan.dwellTime() << " us/sample) [" << dwellSamples << "]: ";
        getNonBlank(dwellSamples);
        std::cout << "scan direction [" << dir << "]:\n";
        std::cout << "\t1 - horizontal\n";
        std::cout << "\t2 - vertical\n";
        getNonBlank(dir);
        std::cout << "scan type [" << type << "]:\n";
        std::cout << "\t1 - snake\n";
        std::cout << "\t2 - raster\n";
        getNonBlank(type);

        //create scan control object and start imaging
        std::cout << "generating scan data and starting imaging (ctrl + s to save)\n";
	    scan.startImaging(dim, dwellSamples, ScanControl::Direction(dir), ScanControl::ScanType(type), etdMin, etdMax);
    } catch(std::exception& err) {
        std::cout << err.what();
    }
    return 0;
}
