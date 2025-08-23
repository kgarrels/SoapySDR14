#include "SoapySDR14.hpp"
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Formats.hpp>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cstddef>
#include <iostream>

SoapySDR::Stream *SoapySDR14::setupStream(
	const int direction,
	const std::string &format,
	const std::vector<size_t> &channels,
	const SoapySDR::Kwargs &args )
{
	std::lock_guard<std::mutex> lock(_device_mutex);

	fprintf(stderr, "setupStream direction %d format %s channels %lu args %lu\n",direction,format.c_str(),
			channels.size(),args.size());

	if (direction != SOAPY_SDR_RX)return NULL;
	if (format != SOAPY_SDR_CF32)return NULL;

	return RX_STREAM;
}

int SoapySDR14::activateStream(
	SoapySDR::Stream *stream,
	const int flags,
	const long long timeNs,
	const size_t numElems )
{
	std::lock_guard<std::mutex> lock(_device_mutex);
	fprintf(stderr, "activateStream + start %p %d %lld %zu\n",stream,flags,timeNs,numElems);

	datacount = 0;
	start();

	return 0;
}

int SoapySDR14::readStream(
	SoapySDR::Stream *stream,
	void * const *buffs,
	const size_t numElems,
	int &flags,
	long long &timeNs,
	const long timeoutUs )
{
	float *out = (float *)buffs[0];

	if ( ! _running )
		return 0;

	if ( numElems > 0 )
	{
		std::unique_lock<std::mutex> lock(_fifo_lock);

		/* Wait until we have the requested number of samples */
		size_t n_samples_avail = _fifo->size();

		while ( n_samples_avail < numElems )
		{
			_samp_avail.wait(lock);
			n_samples_avail = _fifo->size();
		}

		for ( size_t i = 0; i < 2*numElems; i+=2 )
		{
			out[i] = (float)_fifo->at(0).real();
			out[i+1] = (float)_fifo->at(0).imag();
			_fifo->pop_front();
		}
	}
	return(numElems);
}

int SoapySDR14::deactivateStream(
	SoapySDR::Stream *stream,
	const int flags,
	const long long timeNs )
{
	std::lock_guard<std::mutex> lock(_device_mutex);
	fprintf(stderr, "deactivateStream\n");
	stop();
	return 0;
}

void SoapySDR14::closeStream( SoapySDR::Stream *stream )
{
	std::lock_guard<std::mutex> lock(_device_mutex);
	fprintf(stderr, "closeStream\n");
}

size_t SoapySDR14::getStreamMTU( SoapySDR::Stream *stream ) const
{
	fprintf(stderr, "getStreamMTU\n");
	// 16 bit Contiguous Mode: 256 elements
	// 24 bit Contiguous Mode: 240 elements
	return datasize;
}

std::string SoapySDR14::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const
{
	fullScale = 1.0;
	fprintf(stderr, "getNativeStreamFormat\n");
	return SOAPY_SDR_CF32;
}

std::vector<std::string> SoapySDR14::getStreamFormats(const int direction, const size_t channel) const
{
	std::vector<std::string> formats;

	formats.push_back(SOAPY_SDR_CF32);
	fprintf(stderr, "getStreamFormats\n");
	return formats;
}
