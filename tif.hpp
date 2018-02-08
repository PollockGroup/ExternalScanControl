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
	private:
		Tif() {}
		struct Ifd;
		std::vector<Ifd> ifds;

		struct Ifd {//a tiff file is composed of image file directories

			//baseline TIFF tag variables
			uint16_t compression, photometricInterpretation, threshholding, cellWidth, cellLength, fillOrder, orientation, samplesPerPixel, planarConfiguration, grayResponseUnit, resolutionUnit;
			uint32_t newSubfileType, subfileType, imageWidth, imageLength, rowsPerStrip;
			std::vector<uint16_t> bitsPerSample, minSampleValue, maxSampleValue, grayResponseCurve, colorMap, extraSamples;
			std::vector<uint32_t> stripOffsets, stripByteCounts, freeOffsets, freeByteCounts;
			std::string imageDescription, make, model, software, dateTime, artist, hostComputer, copyright;//datetime as YYYY:MM:DD HH:MM:SS, others unspecified
			std::pair<uint32_t, uint32_t> xResolution, yResolution;

			//extension TIFF tag variables
			std::vector<uint16_t> sampleFormat;

			//image data buffer
			std::vector<char> data;

			//ifd metadata as list of entries
			struct Entry;
			std::vector<Entry> entries;

			//constructor fills defualt values where appropriate
			Ifd() : newSubfileType(0), subfileType(0), compression(1), threshholding(1), fillOrder(1), orientation(1), samplesPerPixel(1), rowsPerStrip(0xFFFFFFFF), minSampleValue(1, 0), maxSampleValue(1, 0xFFFF), planarConfiguration(1), resolutionUnit(1), sampleFormat(1, 1) {}

			struct Entry {//and image file directory is composed of entries (+data)
				enum class Tag : uint16_t {
					//baseline TIFF tags
					NewSubfileType            = 0x00FE,//A general indication of the kind of data contained in this subfile.
					SubfileType               = 0x00FF,//A general indication of the kind of data contained in this subfile. (deprecated)
					ImageWidth                = 0x0100,//The number of columns in the image, i.e., the number of pixels per row.
					ImageLength               = 0x0101,//The number of rows of pixels in the image.
					BitsPerSample             = 0x0102,//Number of bits per component.
					Compression               = 0x0103,//Compression scheme used on the image data.
					PhotometricInterpretation = 0x0106,//The color space of the image data.
					Threshholding             = 0x0107,//For black and white TIFF files that represent shades of gray, the technique used to convert from gray to black and white pixels.
					CellWidth                 = 0x0108,//The width of the dithering or halftoning matrix used to create a dithered or halftoned bilevel file.
					CellLength                = 0x0109,//The length of the dithering or halftoning matrix used to create a dithered or halftoned bilevel file.
					FillOrder                 = 0x010A,//The logical order of bits within a byte.
					ImageDescription          = 0x010E,//A string that describes the subject of the image.
					Make                      = 0x010F,//The scanner manufacturer.
					Model                     = 0x0110,//The scanner model name or number.
					StripOffsets              = 0x0111,//For each strip, the byte offset of that strip.
					Orientation               = 0x0112,//The orientation of the image with respect to the rows and columns.
					SamplesPerPixel           = 0x0115,//The number of components per pixel.
					RowsPerStrip              = 0x0116,//The number of rows per strip.
					StripByteCounts           = 0x0117,//For each strip, the number of bytes in the strip after compression.
					MinSampleValue            = 0x0118,//The minimum component value used.
					MaxSampleValue            = 0x0119,//The maximum component value used.
					XResolution               = 0x011A,//The number of pixels per ResolutionUnit in the ImageWidth direction.
					YResolution               = 0x011B,//The number of pixels per ResolutionUnit in the ImageLength direction.
					PlanarConfiguration       = 0x011C,//How the components of each pixel are stored.
					FreeOffsets               = 0x0120,//For each string of contiguous unused bytes in a TIFF file, the byte offset of the string.
					FreeByteCounts            = 0x0121,//For each string of contiguous unused bytes in a TIFF file, the number of bytes in the string.
					GrayResponseUnit          = 0x0122,//The precision of the information contained in the GrayResponseCurve.
					GrayResponseCurve         = 0x0123,//For grayscale data, the optical density of each possible pixel value.
					ResolutionUnit            = 0x0128,//The unit of measurement for XResolution and YResolution.
					Software                  = 0x0131,//Name and version number of the software package(s) used to create the image.
					DateTime                  = 0x0132,//Date and time of image creation.
					Artist                    = 0x013B,//Person who created the image.
					HostComputer              = 0x013C,//The computer and/or operating system in use at the time of image creation.
					ColorMap                  = 0x0140,//A color map for palette color images.
					ExtraSamples              = 0x0152,//Description of extra components.
					Copyright                 = 0x8298,//Copyright notice.

					//extension TIFF tags
					SampleFormat              = 0x0153//Specifies how to interpret each data sample in a pixel.
				};

				//enumeration of possible tiff tag types
				enum class Type : uint16_t {
					BYTE      = 0x0001,//uint8_t
					ASCII     = 0x0002,//null terminated ascii string (count field includes null terminator but not padding byte if needed)
					SHORT     = 0x0003,//uint16_t
					LONG      = 0x0004,//uint32_t
					RATIONAL  = 0x0005,//fractional real number (2 sequential uint32_t: numerator, denominator)
					SBYTE     = 0x0006,//int8_t
					UNDEFINED = 0x0007,//arbitrary 8 bit byte
					SSHORT    = 0x0008,//int16_t
					SLONG     = 0x0009,//int32_t
					SRATIONAL = 0x000A,//as rational but with int32_t
					FLOAT     = 0x000B,//IEEE float
					DOUBLE    = 0x000C //IEEE double
				};
				static_assert(std::numeric_limits<float >::is_iec559, "Tif requires IEEE compliant float" );
				static_assert(std::numeric_limits<double>::is_iec559, "Tif requires IEEE compliant double");

				//entry members
				Tag tag;//tiff tag
				Type type;//tiff data type
				uint32_t valueCount;//number of data values
				std::vector<char> valueBuffer;//data buffer

				//@brief: convert template -> tiff type enumeration
				//@return Tif::Ifd::Entry::Type corresponding to template parameter
				template <typename T> static Type GetType() {
					static_assert(
						std::is_same<T,                      uint8_t  >::value ||
						std::is_same<T,                  std::string  >::value ||
						std::is_same<T,                     uint16_t  >::value ||
						std::is_same<T,                     uint32_t  >::value ||
						std::is_same<T, std::pair<uint32_t, uint32_t> >::value ||
						std::is_same<T,                       int8_t  >::value ||
						std::is_same<T,                      int16_t  >::value ||
						std::is_same<T,                      int32_t  >::value ||
						std::is_same<T, std::pair< int32_t,  int32_t> >::value ||
						std::is_same<T,                        float  >::value ||
						std::is_same<T,                       double  >::value , "Entry type must be one of Tif::Ifd::Entry::Type");
					     if(std::is_same<T,                      uint8_t  >::value) return Type::BYTE;
					else if(std::is_same<T,                  std::string  >::value) return Type::ASCII;
					else if(std::is_same<T,                     uint16_t  >::value) return Type::SHORT;
					else if(std::is_same<T,                     uint32_t  >::value) return Type::LONG;
					else if(std::is_same<T, std::pair<uint32_t, uint32_t> >::value) return Type::RATIONAL;
					else if(std::is_same<T,                       int8_t  >::value) return Type::SBYTE;
					else if(std::is_same<T,                      int16_t  >::value) return Type::SSHORT;
					else if(std::is_same<T,                      int32_t  >::value) return Type::SLONG;
					else if(std::is_same<T, std::pair< int32_t,  int32_t> >::value) return Type::SRATIONAL;
					else if(std::is_same<T,                        float  >::value) return Type::FLOAT;
					else if(std::is_same<T,                       double  >::value) return Type::DOUBLE;
				}

				//@brief: get width of entry data type
				//@return bytes of type
				uint32_t typeBytes() const {
					switch(type) {
						case Type::BYTE:
						case Type::SBYTE:
						case Type::ASCII:
						case Type::UNDEFINED:
						default:
							return 0x00000001;
						case Type::SHORT:
						case Type::SSHORT:
							return 0x00000002;
						case Type::LONG:
						case Type::SLONG:
						case Type::FLOAT:
							return 0x00000004;
						case Type::RATIONAL:
						case Type::SRATIONAL:
						case Type::DOUBLE:
							return 0x00000008;
					}
				}

				//@brief: construct an Ifd entry
				//@param tg: tiff tag
				//@param tp: tiff data type
				//@param count: number of data values
				Entry(const Tag& tg, const Type& tp, const uint32_t count) : tag(tg), type(tp), valueCount(count) {valueBuffer.resize(valueCount * typeBytes());}
				Entry() {}

				friend bool operator<(const Entry& lhs, const Ifd::Entry& rhs) {return (uint16_t)lhs.tag < (uint16_t)rhs.tag;}//entries are supposed to be sorted by tag

				//@brief: build an ifd entry from data
				//@param tag: tiff tag
				//@param values: data (type is determined from template)
				template <typename T>
				static Entry Build(const Tag& t, const std::vector<T>& values) {
					Entry e(t, GetType<T>(), values.size());
					std::memcpy((void*)e.valueBuffer.data(), (void*)values.data(), e.valueBuffer.size());
					return e;
				}

				//@brief: build an ifd entry from datum
				//@param tag: tiff tag
				//@param value: datum (type is determined from template)
				template <typename T>
				static Entry Build(const Tag& t, const T& value) {
					Entry e(t, GetType<T>(), 0x00000001);
					std::memcpy((void*)e.valueBuffer.data(), (void*)&value, e.valueBuffer.size());
					return e;
				}

				//@brief: compute file bytes required to hold this entry (header + data)
				//@return: file bytes required
				uint32_t size() const {
					if(typeBytes() * valueCount <= 0x00000004) return 0x0000000C;
					else return 0x0000000C + (uint32_t)valueBuffer.size() + ((uint32_t)valueBuffer.size() % 2);//tiff requires blocks that are a multiple of 16 bit
				}

				//@brief: read an ifd entry from a file
				//@param is: istream to read from (@current position)
				//@param: true/false if file endedness doesn't/does match system endedness
				void read(std::istream& is, const bool wrongEndian) {
					//read entire ifd entry
					char buff[12];
					is.read(buff, 12);

					//parse tag, type, and number of values and allocate memory to hold data
					if(wrongEndian) {
						std::swap(buff[0], buff[1]);//byteswap tag
						std::swap(buff[2], buff[3]);//byteswap type
						std::reverse(buff+4, buff+8);//byteswap value count
					}
					tag  = reinterpret_cast<Tag* >(buff+0)[0];//first 2 bytes are tag
					type = reinterpret_cast<Type*>(buff+2)[0];//next 2 bytes are type
					valueCount = reinterpret_cast<uint32_t*>(buff+4)[0];//next 4 bytes are number of values
					valueBuffer.resize(valueCount * typeBytes());

					//read ifd entry data
					if(valueBuffer.size() <= 4) {//actual data is stored in next for bytes if it fits
						std::copy(buff + 8, buff + 8 + valueBuffer.size(), valueBuffer.begin());
					} else {//otherwise next 4 bytes is offset to data
						if(wrongEndian)	std::reverse(buff+8, buff+12);//byteswap offset
						std::streamoff curPos = is.tellg();//save current position
						is.seekg(reinterpret_cast<uint32_t*>(buff+8)[0]);//go to data start
						is.read(valueBuffer.data(), valueBuffer.size());//read data
						is.seekg(curPos);//return to end of ifd
					}

					//byteswap ifd data if needed
					if(wrongEndian && typeBytes() > 0x00000001)
						for(uint32_t i = 0; i < valueCount; i++)
							std::reverse(valueBuffer.begin() + i * typeBytes(), valueBuffer.begin() + (i+1) * typeBytes());
				}

				//@brief: write an ifd to a file
				//@param os: ostream to write to (@current position)
				//@param: true/false if file endedness doesn't/does match system endedness
				void write(std::ostream& os, const bool wrongEndian) const {
					//write tag, type, and number of values to buffer
					char buff[12] = {0x0000};
					reinterpret_cast<uint16_t*>(buff  )[0] = (uint16_t) tag;
					reinterpret_cast<uint16_t*>(buff+2)[0] = (uint16_t) type;
					reinterpret_cast<uint32_t*>(buff+4)[0] = valueCount;
					if(wrongEndian) {
						std::swap(buff[0], buff[1]);//byteswap tag
						std::swap(buff[2], buff[3]);//byteswap type
						std::reverse(buff+4, buff+8);//byteswap value count
					}

					//byteswap ifd data if needed
					std::vector<char> vBuff(valueBuffer.begin(), valueBuffer.end());
					if(wrongEndian && typeBytes() > 0x00000001)
						for(uint32_t i = 0; i < valueCount; i++)
							std::reverse(vBuff.begin() + i * typeBytes(), vBuff.begin() + (i+1) * typeBytes());

					if(vBuff.size() <= 4) {
						std::copy(vBuff.begin(), vBuff.end(), buff+8);
						os.write(buff, 12);
					} else {
						reinterpret_cast<uint32_t*>(buff+8)[0] = (uint32_t) os.tellp() + 0x00000004;
						os.write(buff, 12);
						if(1 == vBuff.size() % 2) vBuff.resize(vBuff.size() + 1);//maintain 16 bit word boundaries
						os.write(vBuff.data(), vBuff.size());
					}
				}


				//@brief: get a single valued short entry's data
				//@return: entry value
				uint16_t getShort() const {
					if(0x0000001 != valueCount) throw std::runtime_error("cannot access vector ifd as scalar");
					if(Type::BYTE == type) return (uint16_t) reinterpret_cast<uint8_t const*>(valueBuffer.data())[0];
					else if(Type::SHORT == type) return reinterpret_cast<uint16_t const*>(valueBuffer.data())[0];
					throw std::runtime_error("mismatched ifd type access");
				}

				//@brief: get a single valued long entry's data
				//@return: entry value
				uint16_t getLong() const {
					if(0x0000001 != valueCount) throw std::runtime_error("cannot access vector ifd as scalar");
					if(Type::BYTE == type) return (uint32_t) reinterpret_cast<uint8_t const*>(valueBuffer.data())[0];
					else if(Type::SHORT == type) return (uint32_t) reinterpret_cast<uint16_t const*>(valueBuffer.data())[0];
					else if(Type::LONG == type) return reinterpret_cast<uint32_t const*>(valueBuffer.data())[0];
					throw std::runtime_error("mismatched ifd type access");
				}

				//@brief: get a single valued rational entry's data
				//@return: entry value
				std::pair<uint32_t, uint32_t> getRational() const {
					if(0x0000001 != valueCount) throw std::runtime_error("cannot access vector ifd as scalar");
					if(Type::RATIONAL == type) {
						uint32_t const * const pi32 = reinterpret_cast<uint32_t const *>(valueBuffer.data());
						return std::pair<uint32_t, uint32_t>(pi32[0], pi32[1]);
					}
					throw std::runtime_error("mismatched ifd type access");
				}

				//@brief: get an entry's data
				//@return: entry string
				std::string getAscii() const {
					if(Type::ASCII == type) return std::string(valueBuffer.begin(), valueBuffer.end());
					throw std::runtime_error("mismatched ifd type access");
				}

				//@brief: get a mutli valued short entry's data
				//@return: entry data
				std::vector<uint16_t> getShorts() const {
					if(Type::BYTE == type) return std::vector<uint16_t>(reinterpret_cast<uint8_t const*>(valueBuffer.data()), reinterpret_cast<uint8_t const*>(valueBuffer.data())+valueCount);
					else if(Type::SHORT == type) return std::vector<uint16_t>(reinterpret_cast<uint16_t const*>(valueBuffer.data()), reinterpret_cast<uint16_t const*>(valueBuffer.data())+valueCount);
					throw std::runtime_error("mismatched ifd type access");
				}

				//@brief: get a mutli valued long entry's data
				//@return: entry data
				std::vector<uint32_t> getLongs() const {
					if(Type::BYTE == type) return std::vector<uint32_t>(reinterpret_cast<uint8_t const*>(valueBuffer.data()), reinterpret_cast<uint8_t const*>(valueBuffer.data())+valueCount);
					else if(Type::SHORT == type) return std::vector<uint32_t>(reinterpret_cast<uint16_t const*>(valueBuffer.data()), reinterpret_cast<uint16_t const*>(valueBuffer.data())+valueCount);
					else if(Type::LONG == type) return std::vector<uint32_t>(reinterpret_cast<uint32_t const*>(valueBuffer.data()), reinterpret_cast<uint32_t const*>(valueBuffer.data())+valueCount);
					throw std::runtime_error("mismatched ifd type access");
				}
			};

			//@brief: map entry from tiff tag -> member variable
			//@param e: entry to map data from
			void parseEntry(const Entry& e) {
				switch(e.tag) {
					//baseline tags
					case Entry::Tag::NewSubfileType           : newSubfileType = e.getLong(); break;
					case Entry::Tag::SubfileType              : subfileType = e.getLong(); break;
					case Entry::Tag::ImageWidth               : imageWidth = e.getLong(); break;
					case Entry::Tag::ImageLength              : imageLength = e.getLong(); break;
					case Entry::Tag::BitsPerSample            : bitsPerSample = e.getShorts(); break;
					case Entry::Tag::Compression              : compression = e.getShort(); break;
					case Entry::Tag::PhotometricInterpretation: photometricInterpretation = e.getShort(); break;
					case Entry::Tag::Threshholding            : threshholding = e.getShort(); break;
					case Entry::Tag::CellWidth                : cellWidth = e.getShort(); break;
					case Entry::Tag::CellLength               : cellLength = e.getShort(); break;
					case Entry::Tag::FillOrder                : fillOrder = e.getShort(); break;
					case Entry::Tag::ImageDescription         : imageDescription = e.getAscii(); break;
					case Entry::Tag::Make                     : make = e.getAscii(); break;
					case Entry::Tag::Model                    : model = e.getAscii(); break;
					case Entry::Tag::StripOffsets             : stripOffsets = e.getLongs(); break;
					case Entry::Tag::Orientation              : orientation = e.getShort(); break;
					case Entry::Tag::SamplesPerPixel          : samplesPerPixel = e.getShort(); break;
					case Entry::Tag::RowsPerStrip             : rowsPerStrip = e.getLong(); break;
					case Entry::Tag::StripByteCounts          : stripByteCounts = e.getLongs(); break;
					case Entry::Tag::MinSampleValue           : minSampleValue = e.getShorts(); break;
					case Entry::Tag::MaxSampleValue           : maxSampleValue = e.getShorts(); break;
					case Entry::Tag::XResolution              : xResolution = e.getRational(); break;
					case Entry::Tag::YResolution              : yResolution = e.getRational(); break;
					case Entry::Tag::PlanarConfiguration      : planarConfiguration = e.getShort(); break;
					case Entry::Tag::FreeOffsets              : freeOffsets = e.getLongs(); break;
					case Entry::Tag::FreeByteCounts           : freeByteCounts = e.getLongs(); break;
					case Entry::Tag::GrayResponseUnit         : grayResponseUnit = e.getShort(); break;
					case Entry::Tag::GrayResponseCurve        : grayResponseCurve = e.getShorts(); break;
					case Entry::Tag::ResolutionUnit           : resolutionUnit = e.getShort(); break;
					case Entry::Tag::Software                 : software = e.getAscii(); break;
					case Entry::Tag::DateTime                 : dateTime = e.getAscii(); break;
					case Entry::Tag::Artist                   : artist = e.getAscii(); break;
					case Entry::Tag::HostComputer             : hostComputer = e.getAscii(); break;
					case Entry::Tag::ColorMap                 : colorMap = e.getShorts(); break;
					case Entry::Tag::ExtraSamples             : extraSamples = e.getShorts(); break;
					case Entry::Tag::Copyright                : copyright = e.getAscii(); break;

					//extension tags
					case Entry::Tag::SampleFormat             : sampleFormat = e.getShorts(); break;
					default:
						std::cout << "tif warning: unsupported TIFF tag " << (uint16_t) e.tag << '\n';
				}
			}

			//@brief: read an ifd header from a file
			//@param is: istream to read from
			//@param: true/false if file endedness doesn't/does match system endedness
			void readHeader(std::istream& is, const bool wrongEndian) {
				uint16_t numEntries;
				is.read((char*)&numEntries, 2);//first 2 bytes of ifd are number of entries
				if(wrongEndian) std::swap(reinterpret_cast<uint8_t*>(&numEntries)[0], reinterpret_cast<uint8_t*>(&numEntries)[1]);
				entries.resize(numEntries);
				for(uint16_t i = 0; i < numEntries; i++) {
					entries[i].read(is, wrongEndian);//parse entries
					parseEntry(entries[i]);
				}
			}

			//@brief: read ifd image data from file
			//@param is: istream to read from
			//@param: true/false if file endedness doesn't/does match system endedness
			void readData(std::istream& is, const bool wrongEndian) {
				//read data
				std::streamoff curPos = is.tellg();
				std::vector<uint32_t> buffOffsets(stripByteCounts.size(), 0);
				std::partial_sum(stripByteCounts.begin(), stripByteCounts.end()-1, buffOffsets.begin()+1);
				data.resize(buffOffsets.back() + stripByteCounts.back());//allocate buffer
				if(stripOffsets.size() != stripByteCounts.size()) throw std::runtime_error("mismatched strip byte counts / strip offsets");
				for(size_t i = 0; i < stripOffsets.size(); i++) {
					is.seekg(stripOffsets[i]);//go to strip start
					is.read(data.data() + buffOffsets[i], stripByteCounts[i]);
				}
				is.seekg(curPos);//return to original position

				//image -> scientific origin
				size_t rowBytesCounts = std::accumulate(bitsPerSample.begin(), bitsPerSample.end(), 0) * imageWidth / CHAR_BIT;
				for(uint32_t i = 0; i < imageLength / 2; i++) {
					std::swap_ranges(data.begin() + i * rowBytesCounts, data.begin() + (i+1) * rowBytesCounts, data.begin() + (imageLength - 1 - i) * rowBytesCounts);
				}

				//byteswap data if needed
				std::vector<uint16_t> sampleBytes(bitsPerSample);
				if(1 != compression) throw std::runtime_error("unsupported compression");
				for(uint16_t& bits : sampleBytes) {
					if(0 != bits % CHAR_BIT) throw std::runtime_error("non integer bytes per sample are unsupported");
					bits /= CHAR_BIT;
				}
				if(wrongEndian && *max_element(sampleBytes.begin(), sampleBytes.end()) > 1) {
					std::vector<uint16_t> pixelOffsets(1,0);
					std::partial_sum(sampleBytes.begin(), sampleBytes.end(), pixelOffsets.begin()+1);
					for(uint32_t i = 0; i < imageWidth * imageLength; i++) {
						auto pixelStart = data.begin() + pixelOffsets.back() * i;
						for(size_t j = 0; j < sampleBytes.size(); j++) {
							std::reverse(pixelStart + pixelOffsets[i], pixelStart + pixelOffsets[i+1]);
						}
					}
				}
			}

			//@brief: read an ifd from a file
			//@param is: istream to read from
			//@param: true/false if file endedness doesn't/does match system endedness
			void read(std::istream& is, const bool wrongEndian) {
				readHeader(is, wrongEndian);
				readData(is, wrongEndian);
			}
		};

		//@brief: read tif data into vector of ifds
		//@paream fileName: name of tif file to read
		void read(std::string fileName) {
			const union {
				uint16_t i;
				char c[4];
			} u = {0x0102};
			const bool systemBigEndian = u.c[0] == 1;

			//read header
			std::ifstream is(fileName, std::ios::binary);
			if(!is) throw std::runtime_error("tif file doesn't exist");
			char header[4];
			is.read(header, 4);
			const char bigMagic[4] = {'M','M',0x00,0x2A};
			const char litMagic[4] = {'I','I',0x2A,0x00};
			const bool fileBigEndian = std::equal(header, header+4, bigMagic) ? true : false;

			if( fileBigEndian && !std::equal(header, header+4, bigMagic)) throw std::runtime_error("bad tif header");
			if(!fileBigEndian && !std::equal(header, header+4, litMagic)) throw std::runtime_error("bad tif header");
			const bool wrongEndian = systemBigEndian != fileBigEndian;

			//read ifds
			ifds.clear();
			uint32_t offset;
			is.read((char*)&offset, 4);//get offset of first ifd
			if(wrongEndian) std::reverse(reinterpret_cast<char*>(&offset), reinterpret_cast<char*>(&offset)+4);//byte swap if needed
			do {
				is.seekg(offset);//go to next ifd start
				ifds.resize(ifds.size()+1);//allocate ifd
				ifds.back().read(is, wrongEndian);//read ifd
				is.read((char*)&offset, 4);//get offset to next ifd
				if(wrongEndian) std::reverse(reinterpret_cast<char*>(&offset), reinterpret_cast<char*>(&offset)+4);//byte swap if needed
			} while(0x00000000 != offset);//offset to next ifd = 0 on last ifd
		}

	public:
		//read a multi image tif and convert it to a voxel grid of the selected type
		template <typename T> static std::vector< std::vector<T> > Stack(std::string fileName, uint32_t& width, uint32_t& height) {
			static_assert(
				std::is_same<T,   int8_t>::value ||
				std::is_same<T,  uint8_t>::value ||
				std::is_same<T,  int16_t>::value ||
				std::is_same<T, uint16_t>::value ||
				std::is_same<T,  int32_t>::value ||
				std::is_same<T, uint32_t>::value ||
				std::is_same<T,  int64_t>::value ||
				std::is_same<T, uint64_t>::value ||
				std::is_same<T,    float>::value ||
				std::is_same<T,   double>::value, "Tif::Stack must be templated on (u)int(8/16/32/64) or float/double");

			//read file
			Tif tif;
			tif.read(fileName);

			//make sure that every slice is the same dimensions and data type
			for(size_t i = 0; i < tif.ifds.size(); i++) {
				//check layout (only scalar image reading is supported for now)
				if(1 != tif.ifds[i].samplesPerPixel) throw std::runtime_error("multiple samples per pixel not supported");
				if(0 != tif.ifds[i].photometricInterpretation && 1 != tif.ifds[i].photometricInterpretation) throw std::runtime_error("only scalar images supported");
				if(!tif.ifds[i].extraSamples.empty()) throw std::runtime_error("extra samples not supported");

				//check consistency with first image
				if(tif.ifds[i].imageWidth != tif.ifds[0].imageWidth || tif.ifds[i].imageLength != tif.ifds[0].imageLength) throw std::runtime_error("mismatched image dimensions");
				if(tif.ifds[i].bitsPerSample != tif.ifds[0].bitsPerSample) throw std::runtime_error("mismatched image bitdepths");
				if(tif.ifds[i].sampleFormat != tif.ifds[0].sampleFormat) throw std::runtime_error("mismatched image sample formats");
				if(tif.ifds[i].data.size() != tif.ifds[0].data.size()) throw std::runtime_error("mismatched data sizes");
			}

			//check if tif is acceptable type
			const uint16_t bits = tif.ifds[0].bitsPerSample[0];
			const uint16_t format = tif.ifds[0].sampleFormat[0];//1->uint, 2->int, 3->ieee fp, 4-> undef
			size_t dType = 0;
			switch(bits) {
				case  8:
					switch(format) {
						case 0x0001: dType =  1; break;//u8
						case 0x0002: dType =  2; break;//i8
					} break;
				case 16:
					switch(format) {
						case 0x0001: dType =  3; break;//u16
						case 0x0002: dType =  4; break;//i16
					} break;
				case 32:
					switch(format) {
						case 0x0001: dType =  5; break;//u32
						case 0x0002: dType =  6; break;//i32
						case 0x0003: dType = 10; break;//f32
					} break;
				case 64:
					switch(format) {
						case 0x0001: dType =  7; break;//u64
						case 0x0002: dType =  8; break;//i64
						case 0x0003: dType = 11; break;//f64
					} break;
			}
			if(dType == 0) throw std::runtime_error("unsupported sample format");

			//make sure format can be safely cast to T
			if(std::numeric_limits<T>::is_integer) {//T is int
				if(0x0003 == format) throw std::runtime_error("cannot safely cast IEEE fp -> (u)int");
				else { // integer type tif
					if(bits / 8 > sizeof(T)) throw std::runtime_error("cannot safely cast wide -> narrow (u)int");
					else if(bits / 8 == sizeof(T)) {//same width int
						if(0x0001 == format &&  std::numeric_limits<T>::is_signed) throw std::runtime_error("cannot safely cast uint -> int");
						if(0x0002 == format && !std::numeric_limits<T>::is_signed) throw std::runtime_error("cannot safely cast int -> uint");
					}
				}
			} else if(std::numeric_limits<T>::is_iec559) {//T fp type
				if(0x003 == format) {//tif is fp
					if(bits / 8 > sizeof(T)) throw std::runtime_error("cannot safely cast wide -> narrow fp");
				} else { //tif is int
					if(bits / 8 >= sizeof(T)) throw std::runtime_error("cannot safely cast wide int -> narrow fp");
				}
			}

			//convert each slice to correct type
			width = tif.ifds[0].imageWidth;
			height = tif.ifds[0].imageLength;
			std::vector< std::vector<T> > frames(tif.ifds.size());
			for(size_t i = 0; i < tif.ifds.size(); i++)  {
				switch(dType) {
					case  1: frames[i].assign(reinterpret_cast<uint8_t *>(tif.ifds[i].data.data()), reinterpret_cast<uint8_t *>(tif.ifds[i].data.data()) + width * height); break;
					case  2: frames[i].assign(reinterpret_cast< int8_t *>(tif.ifds[i].data.data()), reinterpret_cast< int8_t *>(tif.ifds[i].data.data()) + width * height); break;
					case  3: frames[i].assign(reinterpret_cast<uint16_t*>(tif.ifds[i].data.data()), reinterpret_cast<uint16_t*>(tif.ifds[i].data.data()) + width * height); break;
					case  4: frames[i].assign(reinterpret_cast< int16_t*>(tif.ifds[i].data.data()), reinterpret_cast< int16_t*>(tif.ifds[i].data.data()) + width * height); break;
					case  5: frames[i].assign(reinterpret_cast<uint32_t*>(tif.ifds[i].data.data()), reinterpret_cast<uint32_t*>(tif.ifds[i].data.data()) + width * height); break;
					case  6: frames[i].assign(reinterpret_cast< int32_t*>(tif.ifds[i].data.data()), reinterpret_cast< int32_t*>(tif.ifds[i].data.data()) + width * height); break;
					case  7: frames[i].assign(reinterpret_cast<uint64_t*>(tif.ifds[i].data.data()), reinterpret_cast<uint64_t*>(tif.ifds[i].data.data()) + width * height); break;
					case  8: frames[i].assign(reinterpret_cast< int64_t*>(tif.ifds[i].data.data()), reinterpret_cast< int64_t*>(tif.ifds[i].data.data()) + width * height); break;
					case 10: frames[i].assign(reinterpret_cast<float   *>(tif.ifds[i].data.data()), reinterpret_cast<float   *>(tif.ifds[i].data.data()) + width * height); break;
					case 11: frames[i].assign(reinterpret_cast<double  *>(tif.ifds[i].data.data()), reinterpret_cast<double  *>(tif.ifds[i].data.data()) + width * height); break;
				}
				tif.ifds[i].data.clear();
			}
			return frames;
		}

		template <typename T> static std::vector< std::vector<T> > Read(std::string fileName, uint32_t& width, uint32_t& height) {return Stack<T>(fileName, width, height);}

		//@brief: write a 3d image to a tiff file
		//@param data: image data
		//@param w: image width
		//@param h: image height
		//@param d: image depth (slices)
		//@param fileName: name of file to write
		//@param sampsPerPix: samples per pixel
		template <typename T> static void Write(T const * const * const data, const uint32_t w, const uint32_t h, const uint32_t d, std::string fileName, uint16_t sampsPerPix = 1) {
			//build shared tif ifd tags
			const uint16_t format =	std::numeric_limits<T>::is_integer ? (std::numeric_limits<T>::is_signed ? 0x0002 : 0x0001) : (std::numeric_limits<T>::is_iec559 ? 0x0003 : 0x0004);
			const uint16_t photo = 3 == sampsPerPix ? 0x0002 : 0x0001;//rgb / black is zero
			std::vector<Ifd::Entry> sharedEntries;
			sharedEntries.push_back(Ifd::Entry::Build(Ifd::Entry::Tag::ImageWidth               ,          w                         ));
			sharedEntries.push_back(Ifd::Entry::Build(Ifd::Entry::Tag::ImageLength              ,          h                         ));
			sharedEntries.push_back(Ifd::Entry::Build(Ifd::Entry::Tag::BitsPerSample            , uint16_t(CHAR_BIT * sizeof(T))     ));
			sharedEntries.push_back(Ifd::Entry::Build(Ifd::Entry::Tag::Compression              , uint16_t(1)                        ));//no compression
			sharedEntries.push_back(Ifd::Entry::Build(Ifd::Entry::Tag::PhotometricInterpretation,          photo                     ));
			sharedEntries.push_back(Ifd::Entry::Build(Ifd::Entry::Tag::SamplesPerPixel          , uint16_t(sampsPerPix)              ));
			sharedEntries.push_back(Ifd::Entry::Build(Ifd::Entry::Tag::RowsPerStrip             ,          h                         ));
			sharedEntries.push_back(Ifd::Entry::Build(Ifd::Entry::Tag::StripByteCounts          , uint32_t(h*w*sizeof(T)*sampsPerPix)));
			sharedEntries.push_back(Ifd::Entry::Build(Ifd::Entry::Tag::SampleFormat             ,          format                    ));

			//check if little/big endian and build magic number
			union {
				uint32_t i;
				char c[4];
			} u = {0x01020304};
			const bool bigEndian = u.c[0] == 1;
			char bigMagic[4] = {'M','M',0x00,0x2A};
			char litMagic[4] = {'I','I',0x2A,0x00};

			//open file and write header
			std::ofstream of(fileName.c_str(), std::ios::out | std::ios::binary);
			uint32_t offset = d * sizeof(T) * h * w * sampsPerPix + 8;//header, image, ifds
			of.write(bigEndian ? bigMagic : litMagic, 4);
			of.write((char*)&offset, 4);

			//write image data
			for(uint32_t i = 0; i < d; i++)
				for(uint32_t j = h-1; j < h; j--)
					of.write((char*)&data[i][w * j * sampsPerPix], sizeof(T) * w * sampsPerPix);

			//write ifds
			for(uint32_t i = 0; i < d; i++) {
				std::vector<Ifd::Entry> sliceEntries(sharedEntries);
				sliceEntries.push_back(Ifd::Entry::Build(Ifd::Entry::Tag::StripOffsets, uint32_t(i * sizeof(T) * h * w * sampsPerPix + 8)));
				std::sort(sliceEntries.begin(), sliceEntries.end());
				const uint16_t numEntries = (uint16_t)sliceEntries.size();
				of.write((char*)&numEntries, 2);//# entries
				for(const Ifd::Entry& e : sliceEntries) e.write(of, false);//actual entries
				offset += 2 + 12 * numEntries + 4;//assumes no entries needing appended data
				const uint32_t nextOffset = i+1 == d ? 0 : offset;
				of.write((char*)&nextOffset, 4);//# entries
			}
		}

		//@brief: write a 2d image to a tiff file
		//@param data: image data
		//@param w: image width
		//@param h: image height
		//@param fileName: name of file to write
		//@param sampsPerPix: samples per pixel
		template <typename T> static void Write(T const * const data, const uint32_t w, const uint32_t h, std::string fileName, uint16_t sampsPerPix = 1) {Write(&data, w, h, 1, fileName, sampsPerPix);}

		//@brief: write a 3d image to a tiff file
		//@param buff: image data
		//@param w: image width
		//@param h: image height
		//@param fileName: name of file to write
		//@param sampsPerPix: samples per pixel
		template <typename T> static void Write(const std::vector<T>& buff, const uint32_t w, const uint32_t h, std::string fileName, uint16_t sampsPerPix = 1) {Write(buff.data(), w, h, fileName, sampsPerPix);}

		//@brief: write a 2d image to a tiff file
		//@param buff: image data
		//@param w: image width
		//@param h: image height
		//@param fileName: name of file to write
		//@param sampsPerPix: samples per pixel
		template <typename T> static void Write(const std::vector< std::vector<T> >& buff, const uint32_t w, const uint32_t h, std::string fileName, uint16_t sampsPerPix = 1) {
			std::vector<T const *> pData(buff.size());
			for(size_t i = 0; i < buff.size(); i++) pData[i] = buff[i].data();
			Write(pData.data(), w, h, buff.size(), fileName, sampsPerPix);
		}

};

#endif//_tif_h_