#ifndef _tif_h_
#define _tif_h_

#include <fstream>
#include <cstdint>
#include <climits>
#include <array>
#include <sstream>
#include <stdexcept>
#include <limits>
#include <vector>
#include <numeric>

struct Tif {
	std::uint32_t width, height;

	struct IfdEntry {
		std::uint16_t tag;
		std::uint16_t type;
		std::uint32_t valueCount;
		union {
			std::uint32_t _long;
			struct {
				std::uint16_t v0;
				std::uint16_t v1;
			} _short;
		} value;

		void setValue(const std::uint32_t v) {value._long     = v; type = 0x0004;}
		void setValue(const std::uint16_t v) {value._short.v0 = v; type = 0x0003;}
		IfdEntry(const std::uint16_t t, const std::uint32_t v) : tag(t), valueCount(0x0001) {setValue(v);}
		IfdEntry(const std::uint16_t t, const std::uint16_t v) : tag(t), valueCount(0x0001) {setValue(v);}
	};
	static_assert(12 == sizeof(IfdEntry), "IfdEntry must be packed");

	void writeHeader(std::ostream& os) {
		const union {
			std::uint16_t i;
			char c[4];
		} u = {0x0102};
		const bool bigEndian = u.c[0] == 1;
		const char bigMagic[4] = {'M','M',0x00,0x2A};
		const char litMagic[4] = {'I','I',0x2A,0x00};
		const std::uint32_t firstIfd = 0x0008;//first ifd immediately following 8 byte header
		char const * const magicBytes = u.c[0] == 1 ? bigMagic : litMagic;
		os.write(magicBytes, 4);
		os.write((char*)&firstIfd, 4);
	}

	template <typename T>
	std::uint32_t writeIfd(std::ostream& os, const std::uint32_t offset, const bool finalEntry) {
		std::array<char, 126> ifd;//2 bytes for entry count, 10 12 byte entries, 4 bytes for offset to next ifd
		reinterpret_cast<std::uint16_t*>(ifd.data())[0] = 0x000A;//10 entries
		IfdEntry* entries = reinterpret_cast<IfdEntry*>(ifd.data()+2);
		entries[0] = IfdEntry(0x0100, width);//width
		entries[1] = IfdEntry(0x0101, height);//height
		entries[2] = IfdEntry(0x0102, std::uint16_t(CHAR_BIT * sizeof(T)));//bits per sample
		entries[3] = IfdEntry(0x0103, (std::uint16_t)0x0001);//compression: none
		entries[4] = IfdEntry(0x0106, (std::uint16_t)0x0001);//photometric interpretation: black is zero
		entries[5] = IfdEntry(0x0111, offset + 126);//strip offset: data immediately after ifd
		entries[6] = IfdEntry(0x0115, (std::uint16_t)0x0001);//samples per pixel: 1
		entries[7] = IfdEntry(0x0116, height);//rows per strip
		entries[8] = IfdEntry(0x0117, std::uint32_t(width * height * sizeof(T)));//strip byte count
		std::uint16_t format = 0x0004;//unknown
		if(std::numeric_limits<T>::is_integer) {
			if(!std::numeric_limits<T>::is_signed) format = 0x0001;//signed int
			else format = 0x0002;//unsigned int
		} else if(std::numeric_limits<T>::is_iec559) format = 0x0003;//floating point
		entries[9] = IfdEntry(0x0153, format);//sample format
		const std::uint32_t nextOffset = offset + 126 + std::uint32_t(width * height * sizeof(T));
		reinterpret_cast<std::uint32_t*>(ifd.data()+122)[0] = finalEntry ? 0x00000000 : nextOffset;
		os.write(ifd.data(), ifd.size());
		return nextOffset;
	}

	template <typename T>
	void writeSlice(std::ofstream& os, T const * const data) {
		//most readers seem to ignore the orientation flag so data should be written with image convention
		const int rowBytes = (int)width * sizeof(T);
		for(int i = int(height)-1; i >= 0; i--) os.write(reinterpret_cast<char const*>(data) + i * rowBytes, rowBytes);
	}

	template <typename T>
	static void Write(T const * const data, const std::uint32_t w, const std::uint32_t h, std::string fileName) {
		//open file and write header + single ifd
		Tif tif(w, h);
		std::ofstream os(fileName, std::ios::out | std::ios::binary);
		tif.writeHeader(os);
		tif.writeIfd<T>(os, 0x00000008, true);
		tif.writeSlice<T>(os, data);
	}

	template <typename T>
	static void Write(T const * const * const data, const std::uint32_t w, const std::uint32_t h, const std::uint32_t slices, std::string fileName) {
		//open file and write header + single ifd
		Tif tif(w, h);
		std::ofstream os(fileName, std::ios::out | std::ios::binary);
		tif.writeHeader(os);
		std::uint32_t offset = 0x00000008;
		for(std::uint32_t i = 0; i < slices; i++) {
			offset = tif.writeIfd<T>(os, offset, i+1 == slices);
			tif.writeSlice<T>(os, data[i]);
		}
	}

	template <typename T> static void Write(std::vector<T>& buff, const std::uint32_t w, const std::uint32_t h, std::string fileName) {Write(buff.data(), w, h, fileName);}
	template <typename T>	static void Write(std::vector< std::vector<T> >& buff, const std::uint32_t w, const std::uint32_t h, std::string fileName) {
		std::vector<T const *> slicePointers(buff.size());
		for(size_t i = 0; i < buff.size(); i++) slicePointers[i] = buff[i].data();
		Write(slicePointers.data(), w, h, (std::uint32_t)slicePointers.size(), fileName);
	}

	private:
		Tif(const std::uint32_t w, const std::uint32_t h) : width(w), height(h) {}
};

#endif//_tif_h_