/********************************************************************************
 *                                                                              *
 * ScanControl.h                                                         *
 *                                                                              *
 * Created by: William C. Lenthe, Alessandro Mottura , and McLean Echlin        *
 * Copyright (c) 2016 University of California, Santa Barbara                   *
 * All Rights Reserved                                                          *
 *                                                                              *
 ********************************************************************************/
#ifndef _ScanControl_H_
#define _ScanControl_H_

#include <string>
#include <vector>
#include <cstdint>
#include <mutex>
#include <thread>
#include <queue>
#include <numeric>

#ifndef NOMINMAX
    #define NOMINMAX
#endif
#include <windows.h>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include "NIDAQmx.h"

/////////////////////////////////////////////////////////////////////////////////
//wrapper class for fast steering mirror
/////////////////////////////////////////////////////////////////////////////////
class ScanControl {
    public:
        enum class Direction : size_t {Horizontal = 1, Vertical = 2};
        enum class ScanType : size_t {Snake = 1, Raster = 2};
        ScanControl(std::string nidaqPathX = "Dev1/ao2", std::string nidaqPathY = "Dev1/ao3", std::string nidaqPathETD = "Dev1/ai2") : m_pathX(nidaqPathX), m_pathY(nidaqPathY), m_pathETD(nidaqPathETD), m_handleOutput(0), m_handleInput(0) {
            //create dummy task using analog input channel
            TaskHandle handle;
            DAQmxTry(DAQmxCreateTask("", &handle));
            DAQmxTry(DAQmxCreateAIVoltageChan(handle, m_pathETD.c_str(), "", DAQmx_Val_Cfg_Default, -5, 5, DAQmx_Val_Volts, NULL));

            //get max sample rate and return dwell
            DAQmxTry(DAQmxGetSampClkMaxRate(handle, &m_rate));
            DAQmxStopTask(handle);
            DAQmxClearTask(handle);
        }

        void startImaging(size_t dim, size_t dwellSamples, Direction dir, ScanType type, double etdMin, double etdMax, size_t dispDim = 512);//dwellSamples in multiples of dwellTime
        ~ScanControl() {clearTasks();}

        //dwell time units in microseconds
        double dwellTime() {return 1e6/m_rate;}

    private:
        // helper class to keep track of the next pixel to update in a snake raster
        class PixelIndex {
            public:
                PixelIndex(int size, ScanType t) : x(0), y(0), dim(size), xDir(1), yDir(1), type(t) {}

                //increments current pixel and returns true if a frame has just been completed(y reversal)
                bool increment() {
                    x += xDir;//increment x
                    switch(type) {
                        case ScanType::Snake: {
                            if(-1 == x || dim == x) {
                                x -= xDir;//bring back in bounds
                                xDir = -xDir;//switch direction
                                y += yDir;//increment y
                                if(-1 == y || dim == y) {
                                    y -= yDir;//bring back in bounds
                                    yDir = -yDir;
                                    return true;
                                }
                            }
                        } break;

                        case ScanType::Raster: {
                            if(dim == x) {
                                x = 0;//return to line start
                                y++;//increment y
                                if(dim == y) {
                                    y = 0;//return to image start
                                    return true;
                                }
                            }
                        } break;

                        default: throw std::runtime_error("invalid scan type");
                    }
                    return false;
                }
                int x, y, xDir, yDir, dim;
                ScanType type;
        };

        void clearTasks();
        void DAQmxTry(int32 result);//if result is nonzero: get daqmx error info, stop tasks, and throw
        void configureImaging(size_t dim, size_t dwellSamples, Direction dir, ScanType type);

        static void SaveImage(sf::Image image);
        static int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void *callbackData);

        std::mutex m_mut;
        std::string m_pathX, m_pathY,m_pathETD;
        TaskHandle m_handleOutput, m_handleInput;
        std::queue<double> m_bufRead, m_bufWrite;
        double m_rate;
        size_t m_sampsPerPix;
};

void ScanControl::startImaging(size_t dim, size_t dwellSamples, Direction dir, ScanType type, double etdMin, double etdMax, size_t dispDim)
{
    //configure image contrast and scaling
    double range = (etdMax - etdMin) / 255.0;

    //create blank image
    PixelIndex index(dim, type);
    sf::Image image, prevFrame;
    image.create(dim, dim, sf::Color::Black);
    prevFrame.create(dim, dim, sf::Color::Black);

    //create sprite and texture to draw image in window
    sf::Sprite sprite;
    sf::Texture texture;
    sprite.setScale(512.0f / dim, 512.0f / dim);

    //configure imaging and start aquisition
    configureImaging(dim, dwellSamples, dir, type);
    DAQmxTry(DAQmxStartTask(m_handleOutput));
    DAQmxTry(DAQmxStartTask(m_handleInput));

    //create window and handle events while the window stays open
    sf::RenderWindow window(sf::VideoMode(dispDim, dispDim), "Image", sf::Style::Titlebar | sf::Style::Close);
    window.requestFocus();
    while (window.isOpen()) {
        sf::Event event;
        while(window.pollEvent(event)) {//handle accumulated events
            switch(event.type) {
                case sf::Event::Closed:
                    window.close();
                    break;
                case sf::Event::KeyPressed:
                    if(event.key.code == sf::Keyboard::Escape) window.close();//close with escape
                    else if(event.key.code == sf::Keyboard::S && event.key.control){//save image with ctrl + 's'
                        std::thread saveThread(SaveImage, prevFrame);
                        saveThread.detach();
                    }
                    break;
            }
        }

        //copy any new data from buffer to image
        m_mut.lock();
        std::swap(m_bufRead, m_bufWrite);
        m_mut.unlock();
        while(!m_bufRead.empty()) {
            double dPix = std::round( (m_bufRead.front() - etdMin) / range );
            dPix = (dPix < 0) ? 0 : dPix;
            dPix = (dPix > 255) ? 255 : dPix;
            uint8_t vPix = static_cast<uint8_t>(dPix);
            switch(dir) {
                case Direction::Horizontal:
                    image.setPixel(index.x, index.y, sf::Color(vPix, vPix, vPix));
                    break;
                case Direction::Vertical:
                    image.setPixel(index.y, index.x, sf::Color(vPix, vPix, vPix));
                    break;
                default: throw std::runtime_error("invalid scan direction");
            }
            if(index.increment()) prevFrame.copy(image, 0, 0);//save frame on y direction reversal
            m_bufRead.pop();
        }

        //redraw image
        texture.loadFromImage(image);
        sprite.setTexture(texture, true);
        window.draw(sprite);
        window.display();
    }
    clearTasks();
}

void ScanControl::clearTasks()
{
    if(0 != m_handleInput) {
        DAQmxStopTask(m_handleInput);
        DAQmxClearTask(m_handleInput);
    }
    if(0 != m_handleOutput) {
        DAQmxStopTask(m_handleOutput);
        DAQmxClearTask(m_handleOutput);
    }
}

void ScanControl::DAQmxTry(int32 result)
{  
    if(0 != result) {
        int32 buffSize = DAQmxGetExtendedErrorInfo(NULL, 0);
        if(buffSize < 0) buffSize = 8192;//sometimes the above returns an error code itself
        std::vector<char> buff(buffSize, 0);
        DAQmxGetExtendedErrorInfo(buff.data(), buff.size());
        clearTasks();
        throw std::runtime_error("NI-DAQmx error " + std::to_string(result) + ":\n" + std::string(buff.data()));
    }
}

void ScanControl::configureImaging(size_t dim, size_t dwellSamples, Direction dir, ScanType type)
{
    //check scan rate, the microscope is limited to a 300 ns dwell at 768 x 512
    //3.33 x factor of safety -> require at least 768 us to cover full -4 -> +4 V scan
    double dwellTime = dwellSamples * this->dwellTime();
    m_sampsPerPix = dwellSamples;
    double minDwell = 768.0 / dim;//minimum dwell time in us
    if(dwellTime < minDwell) throw std::runtime_error("Dwell time too short - dwell must be at least " + std::to_string(minDwell) + " us for " + std::to_string(dim) + " x " + std::to_string(dim) + " images");

    //stop imaging and save changes to imaging conditions
    clearTasks();

    //create scan data
    std::vector<double> data;
        //linearly interpolate from vMin to vMax
        double vMin = -4.0, vMax = 4.0;
        std::vector<double> x(dim, vMin);
        double delta = (vMax - vMin) / (dim - 1);
        for(size_t i = 1; i < dim; i++)
            x[i] += i * delta;
        
        switch(type) {
            case ScanType::Snake: {
                //create vector for data and create single row / column (square image)
                std::vector<double> dataY;
                data.reserve(4 * dim * dim);
                dataY.reserve(2 * dim * dim);

                //generate single pass scan
                for(size_t i = 0; i < dim; i++) {
                    if(0 == i % 2)
                        data.insert(data.end(), x.begin(), x.end());
                    else
                        data.insert(data.end(), x.rbegin(), x.rend());
                    dataY.insert(dataY.end(), dim, x[i]);
                }

                //repeat scan from bottom to top
                data.insert(data.end(), data.rbegin(), data.rend());
                dataY.insert(dataY.end(), dataY.rbegin(), dataY.rend());

                //combine x and y data
                data.insert(data.end(), dataY.rbegin(), dataY.rend());
            } break;

            case ScanType::Raster: {
                //create vector for data and create single row / column (square image)
                std::vector<double> dataY;
                data.reserve(2 * dim * dim);
                dataY.reserve(dim * dim);

                //generate single pass scan
                for(size_t i = 0; i < dim; i++) {
                    data.insert(data.end(), x.begin(), x.end());
                    dataY.insert(dataY.end(), dim, x[i]);
                }

                //combine x and y data
                data.insert(data.end(), dataY.rbegin(), dataY.rend());
            } break;

            default: throw std::runtime_error("invalid scan type");
        }

    //create tasks
    DAQmxTry(DAQmxCreateTask("", &m_handleInput));
    DAQmxTry(DAQmxCreateTask("", &m_handleOutput));

    //create channels
    DAQmxTry(DAQmxCreateAIVoltageChan(m_handleInput, m_pathETD.c_str(), "", DAQmx_Val_Cfg_Default, -5, 5, DAQmx_Val_Volts, NULL));
    std::string channels;
    switch(dir) {
        case Direction::Horizontal:
            channels = m_pathX + std::string(",") + m_pathY;
            break;
        case Direction::Vertical:
            channels = m_pathY + std::string(",") + m_pathX;
            break;
        default: throw std::runtime_error("invalid scan direction");
    }
    DAQmxTry(DAQmxCreateAOVoltageChan(m_handleOutput, channels.c_str(), "", vMin, vMax, DAQmx_Val_Volts, NULL));

    //configure timing
    // float64 sampleRate = m_rate / dwellSamples;
    DAQmxTry(DAQmxCfgSampClkTiming(m_handleOutput, "", m_rate / dwellSamples, DAQmx_Val_Rising, DAQmx_Val_ContSamps, data.size() / 2));
    DAQmxTry(DAQmxCfgSampClkTiming(m_handleInput, "", m_rate, DAQmx_Val_Falling, DAQmx_Val_ContSamps, data.size() / 2));

    //setup one task to start automatically when the other does
    std::string trigName;
    uInt32 numDevices;
    DAQmxGetTaskNumDevices(m_handleInput,&numDevices);
    for(uInt32 i = 1; i <= numDevices; i++) {
        char device[256];
        int32 productCategory;
        DAQmxGetNthTaskDevice(m_handleInput, i++, device, 256);
        DAQmxGetDevProductCategory(device, &productCategory);
        if(productCategory != DAQmx_Val_CSeriesModule && productCategory != DAQmx_Val_SCXIModule) {
            trigName = "/" + std::string(device) + "/ai/StartTrigger";
            break;
        }
    }
    DAQmxTry(DAQmxCfgDigEdgeStartTrig(m_handleOutput, trigName.c_str(), DAQmx_Val_Rising));

    //setup callback for when input gets data
    size_t updateSamples = dwellSamples * (size_t) std::round(40000 / dwellTime);//25 Hz refresh rate
    DAQmxTry(DAQmxCfgInputBuffer(m_handleInput, 8 * updateSamples));
    DAQmxTry(DAQmxRegisterEveryNSamplesEvent(m_handleInput, DAQmx_Val_Acquired_Into_Buffer, updateSamples, 0, ScanControl::EveryNCallback, reinterpret_cast<void*>(this)));//pass class pointer to callback function

    //write scan data
    int32 written;
    DAQmxTry(DAQmxWriteAnalogF64(m_handleOutput, data.size() / 2, FALSE, DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByChannel, data.data(), &written, NULL));
    if( written != data.size() / 2 ) throw std::runtime_error("failed to write scan data to buffer");
}

void ScanControl::SaveImage(sf::Image image)
{
    TCHAR szFilePathName[_MAX_PATH] = "";
    TCHAR szCurrentDirectory[_MAX_PATH];
    GetCurrentDirectory(_MAX_PATH, szCurrentDirectory);
    OPENFILENAME ofn = {0};
    ofn.lStructSize = sizeof(OPENFILENAME);
    ofn.lpstrFilter = "PNG (*.png)\0*.png\0Bitmap (*.bmp)\0*.bmp\0JPEG (*.jpg)\0*.jpg\0\0";
    ofn.lpstrFile = szFilePathName;
    ofn.nMaxFile = _MAX_PATH;
    ofn.lpstrInitialDir = szCurrentDirectory;
    ofn.lpstrTitle = "Save Image";
    ofn.Flags = OFN_OVERWRITEPROMPT;
    ofn.lpstrDefExt = "";
    if(GetSaveFileName(&ofn)) image.saveToFile(ofn.lpstrFile);
}

int32 CVICALLBACK ScanControl::EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void *callbackData)
{
    //convert callback data to class pointer and samples per pixel
    ScanControl* pScanControl = reinterpret_cast<ScanControl*>(callbackData);
    size_t sampsPerPixel = pScanControl->m_sampsPerPix;
    if(NULL == pScanControl) return 1;

    //read samples from hardware buffer
    int32 read;
    std::vector<double> data(nSamples, 0);
    int32 daqmxError = DAQmxReadAnalogF64(taskHandle, nSamples, DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByChannel, data.data(), data.size(), &read, NULL);
    if(nSamples != read) return 2;

    //average pixel data
    std::vector<double> averageData(nSamples / sampsPerPixel, 0);
    for(size_t i = 0; i < averageData.size(); i++) {
        size_t idx = i * sampsPerPixel;
        averageData[i] = std::accumulate(&data[i * sampsPerPixel], &data[(i + 1) * sampsPerPixel], 0.0) / sampsPerPixel;
    }

    //aquire lock, read data into buffer, and release lock
    pScanControl->m_mut.lock();
    for(std::vector<double>::iterator iter = averageData.begin(); iter != averageData.end(); ++iter)
        pScanControl->m_bufWrite.push(*iter);
    pScanControl->m_mut.unlock();
    return 0;
}

#endif