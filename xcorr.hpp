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

#ifndef _xcorr_h_
#define _xcorr_h_

#include <complex>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <algorithm>
#include <numeric>
#include <limits>
#include <thread>
#include <vector>

#include <fftw3.h>

//helper class to wrap fftw in template
template <typename Real>
struct FFTW {static_assert(std::is_same<Real, float>::value || std::is_same<Real, double>::value || std::is_same<Real, long double>::value, "Real must be float, double, or long double");};
template<>
struct FFTW<float> {
	fftwf_plan pFor, pInv;
	FFTW(const int n, const unsigned int flag = FFTW_MEASURE) {//can use FFTW_ESTIMATE for small numbers of ffts
		std::vector<float> testSig(n);
		std::vector< std::complex<float> > testFft(n);
		pFor = fftwf_plan_dft_r2c_1d(n,                 testSig.data(), (fftwf_complex*)testFft.data(), flag);
		pInv = fftwf_plan_dft_c2r_1d(n, (fftwf_complex*)testFft.data(),                 testSig.data(), flag);
	}
	~FFTW() {
		fftwf_destroy_plan(pFor);
		fftwf_destroy_plan(pInv);
	}
	void forward(float* data, std::complex<float>* fft) const {fftwf_execute_dft_r2c(pFor, data               , (fftwf_complex*)fft);}
	void inverse(float* data, std::complex<float>* fft) const {fftwf_execute_dft_c2r(pInv, (fftwf_complex*)fft, data               );}
};

template<>
struct FFTW<double> {
	fftw_plan pFor, pInv;
	FFTW(const int n, const unsigned int flag = FFTW_MEASURE) {
		std::vector<double> testSig(n);
		std::vector< std::complex<long double> > testFft(n);
		pFor = fftw_plan_dft_r2c_1d(n,                testSig.data(), (fftw_complex*)testFft.data(), flag);
		pInv = fftw_plan_dft_c2r_1d(n, (fftw_complex*)testFft.data(),                testSig.data(), flag);
	}
	~FFTW() {
		fftw_destroy_plan(pFor);
		fftw_destroy_plan(pInv);
	}
	void forward(double* data, std::complex<double>* fft) const {fftw_execute_dft_r2c(pFor, data              , (fftw_complex*)fft);}
	void inverse(double* data, std::complex<double>* fft) const {fftw_execute_dft_c2r(pInv, (fftw_complex*)fft, data              );}
};

template<>
struct FFTW<long double> {
	fftwl_plan pFor, pInv;
	FFTW(const int n, const unsigned int flag = FFTW_MEASURE) {
		std::vector<long double> testSig(n);
		std::vector< std::complex<long double> > testFft(n);
		pFor = fftwl_plan_dft_r2c_1d(n,                 testSig.data(), (fftwl_complex*)testFft.data(), flag);
		pInv = fftwl_plan_dft_c2r_1d(n, (fftwl_complex*)testFft.data(),                 testSig.data(), flag);
	}
	~FFTW() {
		fftwl_destroy_plan(pFor);
		fftwl_destroy_plan(pInv);
	}
	void forward(long double* data, std::complex<long double>* fft) const {fftwl_execute_dft_r2c(pFor, data               , (fftwl_complex*)fft);}
	void inverse(long double* data, std::complex<long double>* fft) const {fftwl_execute_dft_c2r(pInv, (fftwl_complex*)fft, data               );}
};

//enumeration of possible scan patterns
enum class ScanType {
	Raster,
	Snake,
	Unknown
};

//class to hold upsampling kernel (computation is relatively expensive and the kernel is reused many times)
//based on Guizar-Sicairos, M., Thurman, S. T., & Fienup, J. R. (2008). Efficient subpixel image registration algorithms. Optics letters, 33(2), 156-158.
template <typename Real>
struct UpsampleKernel {
	const FFTW<Real> fftw;
	const int cols, upsampleFactor, fftSize, kernelSize, kernelSize2;
	std::vector<int> inds;//fft shifted inds
	std::vector< std::vector< std::complex<Real> > > kernel, kernel2;//upsampling kernels for 1/upsampleFactor and 1/2 pixel resolution

	//@brief: construct upsampling kernel
	//@param c: row length
	//@param f: upsampling factor (sub-pixel resolution of 1/f)
	//@param maxShift: maximum allowable shift in pixels
	UpsampleKernel(const int c, const int f, const Real maxShift) : fftw(c), cols(c), upsampleFactor(f), kernelSize((int)std::ceil(maxShift * f)), kernelSize2((int)std::ceil(maxShift * 2)), fftSize(cols / 2 + 1) {
		inds.resize(fftSize);
		std::iota(inds.begin(), inds.end(), 0);
		if(0 == cols % 2) inds.back() = -inds.back();
		kernel .resize(2 * kernelSize  + 1);
		kernel2.resize(2 * kernelSize2 + 1);
		fillKernel(kernel , upsampleFactor);
		fillKernel(kernel2, 2             );
	}

	//@brief: compute upsampling kernel
	//@param kern: vector sized to hold kernel
	//@param factor: upsampling factor
	void fillKernel(std::vector< std::vector< std::complex<Real> > >& kern, const int factor) {
		//"Efficient subpixel image registration algorithms," Opt. Lett. 33, 156-158 (2008).
		//modified to account for conjugate symmetry
		const size_t kernSize = (kern.size() - 1) / 2;
		const Real k = Real(-6.2831853071795864769252867665590057683943387987502) / (cols * factor);
		for(int i = 0; i <= kernSize; i++) {
			const Real ki = k * i;
			kern[kernSize - i].reserve(fftSize);
			kern[kernSize + i].reserve(fftSize);
			std::transform(inds.begin(), inds.end(), std::back_inserter(kern[kernSize + i]), [ki](const int& x){return std::complex<Real>(std::cos(ki*x), std::sin(ki*x));});
			if(i > 0) std::transform(kern[kernSize + i].begin(), kern[kernSize + i].end(), std::back_inserter(kern[kernSize - i]), [](const std::complex<Real>& v){return std::conj(v);});
		}
	}

	//@brief: compute the upsampled fft value for a single subpixel
	//@param kernel: upsampling kernel for subpixel
	//@param xCorr: fft of cross correlation
	//@return: upsampled fft value
	static inline Real UpsampledValue(const std::vector< std::complex<Real> > & k, const std::vector< std::complex<Real> >& xCorr) {
		//abs(dot(2 conjugate symmetric values)) -> all complex components cancel (center complex component doesn't but it is vanishingly small compared to real part for relevant sizes)
		//to be fully rigourous could check for even length half cross correlations and handle)
		return std::inner_product(xCorr.begin()+1, xCorr.end(), k.begin()+1, Real(0), std::plus<Real>(), [](const std::complex<Real>& a, const std::complex<Real>& b){
			return a.real() * b.real() - a.imag() * b.imag();//only accumulate the real part since the imaginary part will cancel out from conjugate symmetry
		}) * Real(2) + xCorr.front().real();//multiply by 2 to account for symmetry and add first entry (k[0] is always 1)
	}

	//@brief: compute the highest correlation sub pixel shift
	//@param xCorr: fft of cross correlation
	//@param shift: initial search position in upsampled kernel (relative to kernel center)
	//@return: highest correlation sub pixel shift (relative to kernel center)
	int computeSubpixelShift(const std::vector< std::complex<Real> >& xCorr, int shift = 0) const {
		//this operation is relatively expensive to brute force and for dic speckle the cross correlation is well behaved for small shifts, so a linear search should work well
		if     (shift <= 1-kernelSize) shift = 2-kernelSize;
		else if(shift >= kernelSize-1) shift = kernelSize-2;
		const int s = shift;
		Real negCor = UpsampledValue(kernel[kernelSize+shift-1], xCorr);//compute cross correlation for single sub pixel shift in negative direction
		Real maxCor = UpsampledValue(kernel[kernelSize+shift  ], xCorr);//compute cross correlation for previous sub pixel shift
		Real posCor = UpsampledValue(kernel[kernelSize+shift+1], xCorr);//compute cross correlation for single sub pixel shift in positive direction
		if(negCor > maxCor || posCor > maxCor) {
			const bool neg = negCor > posCor;//determine which direction to search in
			Real curCor = neg ? negCor : posCor;
			neg ? --shift : ++shift;
			while(curCor > maxCor) {//search until the maximum is passed
				maxCor = curCor;
				neg ? --shift : ++shift;
				if(shift == kernelSize || -shift == kernelSize) throw std::runtime_error("maxima not found within window");//the end of the window is reached
				curCor = UpsampledValue(kernel[kernelSize+shift], xCorr);//compute cross correlation for single sub pixel shift in positive direction
			}
			neg ? ++shift : --shift;//walk back to maxima
		}
		return shift;
	}

	//@brief: compute the highest correlation sub pixel shift for each row, average, and apply the result to all rows
	//@param frame: the frame to align
	//@param refFrame: conj(fft(frame to align to))
	//@param rows: frame height
	//@param ScanType: Raster/Snake to enforce same / alternating shifts, Unknown to detect
	//@return: the applied shift
	template <typename T>
	Real alignFrame(const std::vector<T>& frame, std::vector<Real>& alignedSum, const std::vector< std::complex<Real> >& refFrame, const int rows, ScanType& scanType) const {
		//compute fft of each row of moving frame
		const int fftSizePad = (cols + 2) / 1;//odd size offsets can cause fftw to crash or prevent use of SIMD instructions
		std::vector<Real> rowData(cols);
		std::vector< std::complex<Real> > movFrame;
		movFrame.reserve(fftSizePad * rows);
		for(int i = 0; i < rows; i++) {
			std::copy(frame.cbegin() + i * cols, frame.cbegin() + (i+1) * cols, rowData.begin());//copy data to Real
			fftw.forward(rowData.data(), movFrame.data() + i * fftSizePad);//compute fft
		}

		//upsample convolved ffts near origin to find best shift for each row
		Real meanShiftOdd(0), meanShiftEven(0);
		std::vector<Real> xCorr2(kernel2.size());
		std::vector< std::complex<Real> > xCorr(fftSize);
		int goodRows = 0;
		for(int i = 0; i < rows; i++) {
			std::transform(refFrame.begin() + i * fftSizePad, refFrame.begin() + i * fftSizePad + fftSize, movFrame.data() + i * fftSizePad, xCorr.begin(), std::multiplies< std::complex<Real> >());//first half of cross correlation
			std::transform(kernel2.begin(), kernel2.end(), xCorr2.begin(), [&xCorr](const std::vector< std::complex<Real> >& k){return UpsampledValue(k, xCorr);});//compute subpixel shift with half pixel resolution

			//find largest local maxima in half pixel resolution cross correlation
			size_t jMaxima;
			Real maxima = std::numeric_limits<Real>::lowest();
			for(size_t j = 1; j < xCorr2.size() - 1; j++) {
				if(xCorr2[j] > xCorr2[j-1] && xCorr2[j] > xCorr2[j+1] && xCorr2[j] > maxima) {//local maxima
					jMaxima = j;
					maxima = xCorr2[j];
				}
			}
			const int shift2 = int(jMaxima) - kernelSize2;
			try {
				const int shift = computeSubpixelShift(xCorr, shift2 * upsampleFactor / 2 + 5);//search from nearest half pixel
				if(0 == i % 2) meanShiftEven += Real(shift) / upsampleFactor;
				else meanShiftOdd += Real(shift) / upsampleFactor;
				goodRows++;
			} catch (std::runtime_error& e) {
				//silently handle uncorrelated rows (from noisy/low contrast images) by not incrementing goodRows
			}
		}
		if(double(goodRows) / rows < 0.95) throw std::runtime_error("too few good rows");
		if(ScanType::Unknown == scanType) scanType = std::signbit(meanShiftOdd) != std::signbit(meanShiftEven) ? ScanType::Snake : ScanType::Raster;//check if this frame was a snake or raster if unknown
		const Real meanShift = (meanShiftEven + (ScanType::Snake == scanType ? -meanShiftOdd : meanShiftOdd)) / goodRows;//compute average shift accounting for scan type

		//apply shift in fourier space
		const Real vMin(std::numeric_limits<T>::lowest());
		const Real vMax(std::numeric_limits<T>::max());
		const Real k = Real(-6.2831853071795864769252867665590057683943387987502 * meanShift) / cols;
		std::vector< std::complex<Real> > phaseShift(fftSize);
		std::transform(inds.begin(), inds.end(), phaseShift.begin(), [k](const int& x){return std::complex<Real>(std::cos(k*x), std::sin(k*x));});//build phase shift vector
		if(ScanType::Snake == scanType) {//phase shift rows of snake scan in alternating direction
			for(int i = 0; i < rows; i+=2) std::transform(phaseShift.begin(), phaseShift.end(), movFrame.data() + i * fftSizePad, movFrame.data() + i * fftSizePad, std::multiplies< std::complex<Real> >());
			std::for_each(phaseShift.begin(), phaseShift.end(), [](std::complex<Real>& v){v = std::conj(v);});
			for(int i = 1; i < rows; i+=2) std::transform(phaseShift.begin(), phaseShift.end(), movFrame.data() + i * fftSizePad, movFrame.data() + i * fftSizePad, std::multiplies< std::complex<Real> >());
		} else {//phase shift rows of raster scan in same direction
			for(int i = 0; i < rows; i++) std::transform(phaseShift.begin(), phaseShift.end(), movFrame.data() + i * fftSizePad, movFrame.data() + i * fftSizePad, std::multiplies< std::complex<Real> >());
		}
		for(int i = 0; i < rows; i++) {
			fftw.inverse(rowData.data(), movFrame.data() + i * fftSizePad);//compute inverse fft
			std::transform(rowData.begin(), rowData.end(), alignedSum.begin() + i * cols, alignedSum.begin() + i * cols, std::plus<Real>());//accumulate aligned result
		}
		return meanShift;//fftw convention
	}
};

//@brief: helper function to call kernel::alignFrame in parallel (exception handling)
//@param kernel: upsampling kernel
//@param frames: images to align
//@param alignedSum: vector to hold sum of aligned frames
//@param scanType: Snake/Raster
//@param shifts: vector to hold computed+applied shifts
//@param bounds: range of frames to align (bounds[0]->bounds[1])
//@param pExp: pointer for any caught exceptions
template <typename Real, typename T>
inline void alignFrames(const UpsampleKernel<Real>& kernel, std::vector< std::vector<T> >& frames, std::vector<Real>& alignedSum, const std::vector< std::complex<Real> >& refFrame, const int rows, ScanType scanType, std::vector<Real>& shifts, int const * const bounds, std::exception_ptr& pExp) {
	try {
		if(ScanType::Unknown == scanType) throw std::runtime_error("unknown scan type not allowed for parallel alignment");
		for(int i = bounds[0]; i < bounds[1]; i++) shifts[i-1] = kernel.alignFrame(frames[i-1], alignedSum, refFrame, rows, scanType);
	} catch (...) {
		pExp = std::current_exception();
	}
}

//the beam shift follows a gompertz type function
template <typename Real>
struct Gompertz {
	Real b, c;//Gompertz fit parameters

	//@brief: compute the value of a Gompertz function
	//@param x: position to evaluate function
	//@return: function value
	Real evaluate(const Real& x) const {return Real(1) - std::exp(-b * std::exp(-c * x) );}

	//@brief: construct a Gompertz struct from known paramters
	//@param vB: value of b parameter
	//@param vC: value of c parameter
	Gompertz(const Real vB, const Real vC) : b(vB), c(vC) {}

	//@brief: construct a Gompertz struct by least squares fit
	//@param x: x coordinates of points to fit
	//@param y: y coordinates of points to fit
	Gompertz(const std::vector<Real>& x, const std::vector<Real>& y) {leastSquares(x, y);}

	//@brief: compute the mean squared deviation of this fit
	//@param x: x coordinates of points
	//@param y: y coordinates of points
	//@return: mean squared deviation
	Real msd(const std::vector<Real>& x, const std::vector<Real>& y) const {
		Real r2(0);
		for(size_t i = 0; i < x.size(); i++) {
			const Real ri = y[i] - evaluate(x[i]);
			r2 += ri * ri;
		}
		return r2;
	}

	private:
		//@brief: compute a least squares fit Gompertz function by Gauss-Newton iteration
		//@param x: x coordinates of points to fit
		//@param y: y coordinates of points to fit
		//@param maxIter: maximum number of Gauss-Newton iterations
		//@param thr: convergence threshold
		void leastSquares(const std::vector<Real>& x, const std::vector<Real>& y, const size_t maxIter = 50, const Real thr = 0.0001) {
			//get initial estimate of parameters by computing least squares fit of z = -ln(1-y) = b * exp (-c * x)
			//this should be an exact solution for the least squares fit of points between 0 and 1
			Real sX2Z(0), sZlnZ(0), sXZ(0), sXZlnZ(0), sZ(0);
			for(size_t i = 0; i < x.size(); i++) {
				if(Real(1) > y[i] && y[i] > Real(0)) {
					const Real z = -std::log(Real(1) - y[i]);
					const Real lnZ = std::log(z);
					sX2Z += x[i] * x[i] * z;
					sZlnZ += z * lnZ;
					sXZ += x[i] * z;
					sXZlnZ += x[i] * z * lnZ;
					sZ += z;
				}
			}
			const Real den = sZ * sX2Z - sXZ * sXZ;
			b = (sX2Z * sZlnZ - sXZ * sXZlnZ) / den;
			b = std::exp((sX2Z * sZlnZ - sXZ * sXZlnZ) / den);
			c = -(sZ * sXZlnZ - sXZ * sZlnZ) / den;

			//gauss newton iterate with all points until convergence
			Real ssRes, metricPrev = std::numeric_limits<Real>::max();
			for(size_t i = 0; i < maxIter; i++) {
				//compute jacobian^T * jacobian and jacobian^T * residuals
				Real ss = 0;
				Real jTj[2][2] = {Real(0)}, jTr[2] = {Real(0)};
				for(size_t i = 0; i < x.size(); i++) {
					//compute residual and corresponding row of jacobian
					const Real ri = y[i] - evaluate(x[i]);//compute residual
					const Real cxi = -c * x[i];
					const Real dRd0 = -std::exp(-b * std::exp(cxi) + cxi);
					const Real dRd1 = -b * x[i] * dRd0;
					ss += ri;

					jTj[0][0] += dRd0 * dRd0;
					jTj[0][1] += dRd0 * dRd1;
					jTj[1][1] += dRd1 * dRd1;

					//accumulate jacobian^T * residuals
					jTr[0] += ri * dRd0;
					jTr[1] += ri * dRd1;
				}

				//invert jacobian
				const Real det = jTj[0][0] * jTj[1][1] - jTj[0][1] * jTj[0][1];
				jTj[0][0] /= det;
				jTj[1][1] /= det;
				std::swap(jTj[0][0], jTj[1][1]);
				jTj[0][1] /= -det;
				jTj[1][0] = jTj[0][1];

				//compute step
				b -= jTj[0][0] * jTr[0] + jTj[0][1] * jTr[1];
				c -= jTj[1][0] * jTr[0] + jTj[1][1] * jTr[1];
				if(b < Real(0)) b = c;//if a large step in c is required b can overshoot to a negative value which doesn't make sense

				//check for convergence
				const Real metric = std::abs(Real(1) - ssRes / ss);
				if(metric >= metricPrev && metric < thr) return;
				metricPrev = metric;
				ssRes = ss;
			}
			throw std::runtime_error("failed to converged within allowed iterations");
		}
};

//@breif: align frames by row correlation and return the integrated result
//@parame frames: frames to align
//@param rows: frame height
//@param cols: frame width
//@param frameShifts: output vector of shifts applied
//@param scanType: Snake/Raster to impose alignment type, Unknown to measure from first frame
//@param maxShift: largest allowable shift
//@param isSnake: optional location to copy if the scan is detected as a snake or raster
//@return: integration of aligned frames
template <typename Real, typename T>
std::vector<Real> correlateRows(std::vector< std::vector<T> >& frames, const int rows, const int cols, std::vector<Real>& frameShifts, ScanType& scanType, const Real maxShift = 1.25, const int upsampleFactor = 16) {
	const UpsampleKernel<Real> kernel(cols, upsampleFactor, maxShift);

	//compute fft of each row of final frame
	std::vector<Real> rowData(cols);
	const int fftSize = cols / 2 + 1;
	const int fftSizePad = (cols + 2) / 1;//odd size offsets can cause fftw to crash or prevent use of SIMD instructions
	std::vector< std::complex<Real> > refFrame(fftSizePad * rows);
	for(int i = 0; i < rows; i++) {
		std::copy(frames.back().begin() + i * cols, frames.back().begin() + (i+1) * cols, rowData.begin());//copy data to Real
		kernel.fftw.forward(rowData.data(), refFrame.data() + i * fftSizePad);//compute fft
	}
	for(std::complex<Real>& v : refFrame) v = std::conj(v);//need complex conjugate of reference fft

	//compute and apply subpixel shift for first frame and check if the image was collected by snake or raster
	frameShifts.resize(frames.size());
	frameShifts.back() = Real(0);
	std::vector<Real> integratedImage(frames.back().begin(), frames.back().end());
	frameShifts[0] = kernel.alignFrame(frames[0], integratedImage, refFrame, rows, scanType);

	static const bool parallel = true;
	if(parallel) {
		//build indices of threads
		const size_t threadCount = std::max<size_t>(std::thread::hardware_concurrency(), 1);
		std::vector<int> workerInds(threadCount + 1);
		std::vector<std::exception_ptr> expPtrs(threadCount, NULL);
		const Real frameCount = Real(frames.size()) / threadCount;
		for(size_t i = 0; i < workerInds.size(); i++) workerInds[i] = (int)std::round(frameCount * i);
		std::replace(workerInds.begin(), workerInds.end(), 0, 2);
		std::vector< std::vector<Real> > alignedSums(threadCount, std::vector<Real>(frames.front().size(), Real(0)));

		//compute and apply subpixel shift for each frame in parallel, integrating blocks of frames aligned by the same thread
		std::vector<std::thread> workers((size_t)threadCount);
		for(size_t i = 0; i < workers.size(); i++) workers[i] = std::thread(alignFrames<Real, T>, std::ref(kernel), std::ref(frames), std::ref(alignedSums[i]), std::ref(refFrame), rows, scanType, std::ref(frameShifts), workerInds.data() + i, std::ref(expPtrs[i]));
		for(size_t i = 0; i < workers.size(); i++) workers[i].join();
		for(size_t i = 0; i < workers.size(); i++)
			if(NULL != expPtrs[i]) std::rethrow_exception(expPtrs[i]);

		//integrate result from each thread
		for(size_t i = 0; i < alignedSums.size(); i++) std::transform(integratedImage.begin(), integratedImage.end(), alignedSums[i].begin(), integratedImage.begin(), std::plus<Real>());
	} else {//serial
		std::vector<Real> alignedFrame(frames.front().size());
		for(int i = 2; i < frames.size(); i++) {
			frameShifts[i-1] = kernel.alignFrame(frames[i-1], alignedFrame, refFrame, rows, scanType);//skip first frame
			std::transform(integratedImage.begin(), integratedImage.end(), alignedFrame.begin(), integratedImage.begin(), std::plus<Real>());
		}
	}
	
	//check that the shifts are reasonable by computing mean squared deviation of least squares fit Gompertz function and return
	std::vector<Real> x(frameShifts.size());
	std::iota(x.begin(), x.end(), Real(0));//create evenly spaced list of collection times (scale doesn't matter)
	Gompertz<Real> gomp(x, frameShifts);//compute least squares fit gompertz function
	if(gomp.msd(x, frameShifts) > Real(1.5) / upsampleFactor) throw std::runtime_error("row shifts don't fit gompertz function well");//heuristic cutoff
	return integratedImage;
}

//@breif: align frames using specified shifts
//@parame frames: frames to align
//@param rows: frame height
//@param cols: frame width
//@param frameShifts: shifts to apply
//@param scanType: Snake/Raster
template <typename Real, typename T>
void alignRows(std::vector< std::vector<T> >& frames, const int rows, const int cols, const std::vector<Real>& frameShifts, const ScanType scanType) {
	//time fft and get constants
	const FFTW<Real> fftw(cols);
	const int fftSize = cols / 2 + 1;
	const Real vMin(std::numeric_limits<T>::lowest());
	const Real vMax(std::numeric_limits<T>::max());

	//allocate arrays
	std::vector<Real> rowData(cols);
	std::vector< std::complex<Real> > rowFft(fftSize), phaseShift(fftSize), phaseShiftConj(fftSize);
	std::vector<int> inds(fftSize);
	std::iota(inds.begin(), inds.end(), 0);
	if(0 == cols % 2) inds.back() = -inds.back();

	//loop over frames
	if(ScanType::Unknown == scanType) throw std::runtime_error("unknown scan type not allowed for precomputed alignment");
	for(size_t i = 0; i < frames.size(); i++) {
		//compute phase shift onces
		const Real k = Real(6.2831853071795864769252867665590057683943387987502 * frameShifts[i]) / cols;
		std::transform(inds.begin(), inds.end(), phaseShift.begin(), [k](const int& x){return std::complex<Real>(std::cos(k*x), std::sin(k*x));});
		std::transform(phaseShift.begin(), phaseShift.end(), phaseShiftConj.begin(), [](const std::complex<Real>& v){return std::conj(v);});

		//apply shift row by row
		for(int j = 0; j < rows; j++) {
			std::copy(frames[i].begin() + j * cols, frames[i].begin() + (j+1) * cols, rowData.begin());//copy data to Real
			fftw.forward(rowData.data(), rowFft.data());//compute fft
			if(ScanType::Snake == scanType && 1 == j % 2)
				std::transform(phaseShift    .begin(), phaseShift    .end(), rowFft.begin(), rowFft.begin(), std::multiplies< std::complex<Real> >());//fft shift
			else
				std::transform(phaseShiftConj.begin(), phaseShiftConj.end(), rowFft.begin(), rowFft.begin(), std::multiplies< std::complex<Real> >());//fft shift
			fftw.inverse(rowData.data(), rowFft.data());//inverse fft
			std::transform(rowData.begin(), rowData.end(), frames[i].begin() + j * cols, [vMin, vMax, cols](const Real& v) {return (T)std::max(vMin, std::min(vMax, std::round(v / cols)));});
		}
	}
}

#endif//_xcorr_h_