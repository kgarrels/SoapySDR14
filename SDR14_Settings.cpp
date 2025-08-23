#include "SoapySDR14.hpp"
#include <cstdio>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <cerrno>
#include <vector>
#include <chrono>
#include <thread>

#define DEFAULT_HOST  "127.0.0.1" /* We assume a running "siqs" from CuteSDR project */
#define DEFAULT_PORT  50000

SoapySDR14::SoapySDR14(const SoapySDR::Kwargs &args):
  //_tcp(-1),
  //_udp(-1),
  _running(false),
  _keep_running(false),
  _sequence(0),
  _nchan(1),
  _sample_rate(0),
  _bandwidth(0.0),
  _gain(0),
  datasize(240)
{
 	const SoapySDR::Kwargs options;
	/*
  if (args.size()) {
		printf("driver args %lu ",args.count("driver"));
		if (args.count("driver"))printf(" %s \n",args.at("driver").c_str());
		printf("label args %lu ",args.count("label"));
		if (args.count("label"))printf(" %s \n",args.at("label").c_str());
		printf("SDR14 args %lu ",args.count("SDR14"));
		if (args.count("SDR14"))printf(" %s \n",args.at("SDR14").c_str());
	}
  */

  struct ftdi_context *ftdi;
  if ((ftdi = ftdi_new()) == 0)
  {
      printf("ftdi_new failed\n");
      return;
  }

  int ret;
  if ((ret = ftdi_usb_open(ftdi, 0x0403, 0xf728)) < 0)    // sdr_14
  {
      printf("unable to open ftdi device: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
      ftdi_free(ftdi);
      return;
  }
  _ftdi = ftdi;
  //ftdi_usb_purge_buffers(_ftdi);

  _run_usb_read_task = true;
  _fifo = new boost::circular_buffer<gr_complex>( 200000 );
  if ( ! _fifo )
    throw std::runtime_error( "Failed to allocate sample FIFO" );

  _thread = std::thread(&SoapySDR14::usb_read_task, this);

    /* request & print device information */

  std::vector< unsigned char > response;

  {
    std::cerr << "Using ";

    unsigned char name[] = { 0x04, 0x20, 0x01, 0x00 }; /* SDR14 4.1.1 Target Name */
    if ( transaction( name, sizeof(name), response ) )
      std::cerr << "RFSPACE " << &response[sizeof(name)] << " ";

    unsigned char sern[] = { 0x04, 0x20, 0x02, 0x00 }; /* SDR14 4.1.2 Target Serial Number */
    if ( transaction( sern, sizeof(sern), response ) )
      std::cerr << "SN " << &response[sizeof(sern)] << " ";
  }

  /* SDR14 4.1.4 Hardware/Firmware Versions */

  unsigned char bootver[] = { 0x05, 0x20, 0x04, 0x00, 0x00 };
  if ( transaction( bootver, sizeof(bootver), response ) )
    std::cerr << "BOOT " << *((uint16_t *)&response[sizeof(bootver)]) << " ";

  unsigned char firmver[] = { 0x05, 0x20, 0x04, 0x00, 0x01 };
  if ( transaction( firmver, sizeof(firmver), response ) )
    std::cerr << "FW " << *((uint16_t *)&response[sizeof(firmver)]) << "\n";

  getFrequencyRange2(SOAPY_SDR_RX, 0);
}


size_t SoapySDR14::read_bytes( char *data, size_t size, bool &run )
{
  size_t nbytes = 0;
  
  while ( nbytes < size && run )
  {
    int nread=0; 

    nread = ftdi_read_data( _ftdi, (unsigned char *)&data[nbytes], 1);                // SDR-14
    if (nread < 0 ) std::cerr << " nread " << nread <<  " " <<_ftdi->error_str <<"\n";

    if ( nread == 0 )
      continue;   

    if ( nread < 0 )
      break;

    nbytes++;
  }
  return nbytes;
}

bool SoapySDR14::ack()
{
  //std::cerr << ".";
  unsigned char ack[] = { 0x03, 0x60, 0x00 };
  return ftdi_write_data(_ftdi, ack, 3);
}

void SoapySDR14::usb_read_task()
{
  uint cnt = 0;
  char data[1024*10];
  size_t n_avail, to_copy;

  while ( _run_usb_read_task )
  {
    size_t nbytes;
    nbytes = read_bytes( data, 2, _run_usb_read_task );
    if ( nbytes != 2 )
      continue;

    size_t length = ((data[1] << 8) | data[0]) & 0x1fff;
    if ( 0 == length ) /* SDR-IQ 5.4.1 Output Data Item 0 */
      length = 1024*8 + 2;

    if ( length <= 2 )
      continue;

    length -= 2; /* subtract header */

    if ( length > sizeof(data) - 2 )
    {
      _run_usb_read_task = false;
      continue;
    }

    nbytes = read_bytes( data + 2, length, _run_usb_read_task );
    if ( nbytes != length )
      continue;

    if ( 1024*8 == length )
    {
      // SDR-14 needs ack
      if(cnt++ > 10) 
      {
        ack();
        cnt = 0;
      }
    
      /* push samples into the fifo */
      _fifo_lock.lock();

      size_t num_samples = length / 4;
      n_avail = _fifo->capacity() - _fifo->size();
      to_copy = (n_avail < num_samples ? n_avail : num_samples);

      #define SCALE_16  (1.0f/32768.0f)

      int16_t *sample = (int16_t *)(data + 2);

      for ( size_t i = 0; i < to_copy; i++ )
      {
        /* Push sample to the fifo */
       _fifo->push_back( gr_complex( *(sample+0) * SCALE_16,
                                      *(sample+1) * SCALE_16 ) );
        /* offset to the next I+Q sample */
        sample += 2;
      }

      #undef SCALE_16

      _fifo_lock.unlock();

      /* We have made some new samples available to the consumer in work() */
      if (to_copy) {
        //std::cerr << "+" << std::flush;
        _samp_avail.notify_one();
      }

      /* Indicate overrun, if neccesary */
      if (to_copy < num_samples)
        std::cerr << "O" << std::flush;
    }
    else
    {
      /* copy response & signal transaction */
      _resp_lock.lock();
      _resp.clear();
      _resp.resize( length + 2 );
      memcpy( _resp.data(), data, length + 2 );
      _resp_lock.unlock();
      _resp_avail.notify_one();
    }
  }
}



bool SoapySDR14::transaction( const unsigned char *cmd, size_t size )
{
  std::vector< unsigned char > response;

  if ( ! transaction( cmd, size, response ) )
    return false;

  /* comparing the contents is not really feasible due to protocol */
  if ( response.size() == size ) /* check response size against request */
    return true;

  return false;
}

bool SoapySDR14::transaction( const unsigned char *cmd, size_t size,
                                   std::vector< unsigned char > &response )
{
  size_t rx_bytes = 0;
  unsigned char data[1024*10];
  int ret=0;

  response.clear();

//define VERBOSE
#ifdef VERBOSE
  printf("< ");
  for (size_t i = 0; i < size; i++)
    printf("%02x ", (unsigned char) cmd[i]);
  printf("\n");
#endif

  if ( (ret = ftdi_write_data(_ftdi, cmd, size))  )
  {
    //std::cerr << "written " << ret << "\n";
    if (ret != (int)size) return false;
  }

  std::unique_lock<std::mutex> lock(_resp_lock);
  _resp_avail.wait(lock);

  rx_bytes = _resp.size();
  memcpy( data, _resp.data(), rx_bytes );

  response.resize( rx_bytes );
  memcpy( response.data(), data, rx_bytes );

#ifdef VERBOSE
  printf("> ");
  for (size_t i = 0; i < rx_bytes; i++)
    printf("%02x ", (unsigned char) data[i]);
  printf("\n");
#endif

  return true;
}

SoapySDR14::~SoapySDR14(void)
{
  std::cerr << "sdr-14 closing.\n";

  _run_usb_read_task = false;
  _thread.join();

  ftdi_free(_ftdi);
  if ( _fifo )
  {
    delete _fifo;
    _fifo = NULL;
  }  
}
void SoapySDR14::setAntenna( const int direction, const size_t channel, const std::string &name )
{
	return;
}
void SoapySDR14::setGainMode( const int direction, const size_t channel, const bool automatic )
{
	/* enable AGC if the hardware supports it, or remove this function */
}
bool SoapySDR14::getGainMode( const int direction, const size_t channel ) const
{
	return(false);
	/* ditto for the AGC */
}

#define BANDWIDTH 34e6

void SoapySDR14::setBandwidth( const int direction, const size_t channel, const double bandwidth )
{
  return;
}


void SoapySDR14::setFrequency( const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args )
{
	std::lock_guard<std::mutex> lock(_device_mutex);

  uint32_t u32_freq = (uint32_t)frequency;
  std::cerr << "setFrequency " << frequency/1000 << "kHz\n";

  /* SDR14 4.2.3 Receiver Frequency */
  unsigned char tune[] = { 0x0A, 0x00, 0x20, 0x00, 0x00, 0xb0, 0x19, 0x6d, 0x00, 0x01 };

  tune[sizeof(tune)-5] = u32_freq >>  0;
  tune[sizeof(tune)-4] = u32_freq >>  8;
  tune[sizeof(tune)-3] = u32_freq >> 16;
  tune[sizeof(tune)-2] = u32_freq >> 24;
  tune[sizeof(tune)-1] = 0;

  transaction( tune, sizeof(tune) );

}
void SoapySDR14::setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs &args)
{
	std::lock_guard<std::mutex> lock(_device_mutex);
  std::cerr << "setFrequency " << frequency/1000 << "kHz\n";

  uint32_t u32_freq = (uint32_t)frequency;

  /* SDR14 4.2.3 Receiver Frequency */
  unsigned char tune[] = { 0x0A, 0x00, 0x20, 0x00, 0x00, 0xb0, 0x19, 0x6d, 0x00, 0x01 };

  tune[sizeof(tune)-5] = u32_freq >>  0;
  tune[sizeof(tune)-4] = u32_freq >>  8;
  tune[sizeof(tune)-3] = u32_freq >> 16;
  tune[sizeof(tune)-2] = u32_freq >> 24;
  tune[sizeof(tune)-1] = 0;

  transaction( tune, sizeof(tune) );
}

bool SoapySDR14::setDsp()
{
  int row;
  std::vector< unsigned char > response;

  for (row=0; row < (int)BWKHZ_150.size(); row++)
  {
    if ( ! transaction((unsigned char *)&BWKHZ_150[row][0], BWKHZ_150[row].size(), response)) 
    {
      std::cerr << "setDsp() failed.\n";
      return false;
    }
  }
  std::cerr << "setDsp() done.\n";
  return true;
}



void SoapySDR14::setSampleRate( const int direction, const size_t channel, const double rate )
{
  if ( _running )
  {
    _keep_running = true;
    stop();
  }

  //sdr-14 needs to load AD6620
  if ( !setDsp())
    throw std::runtime_error("setDsp() failed");
  _sample_rate = 158730;

  if ( _running )
  {
    start();
  }

  if ( rate != _sample_rate )
    std::cerr << "Radio reported a sample rate of " << (uint32_t)_sample_rate << " Hz"
              << "Requested rate " << rate << " Hz" <<std::endl;

}


bool SoapySDR14::start()
{
  _sequence = 0;
  _running = true;
  _keep_running = false;

  /* SDR-IP 4.2.1 Receiver State */
  /* SDR14 4.2.1 Receiver State */
  unsigned char start[] = { 0x08, 0x00, 0x18, 0x00, 0x81, 0x02, 0x00, 0x01 };

  // set if gain
  std::cerr << "start, set if gain\n";
  unsigned char ifgain[] = { 0x06, 0x00, 0x40, 0x00, 0x00, 0x18 }; 
  transaction( ifgain, sizeof(ifgain) );

  return transaction( start, sizeof(start) );
}

bool SoapySDR14::stop()
{
  if ( ! _keep_running )
    _running = false;
  _keep_running = false;

  if ( _fifo )
    _fifo->clear();

  /* SDR-IP 4.2.1 Receiver State */
  /* SDR14 4.2.1 Receiver State */
  unsigned char stop[] = { 0x08, 0x00, 0x18, 0x00, 0x81, 0x01, 0x00, 0x00 };
  return transaction( stop, sizeof(stop) );
}
void SoapySDR14::setGain( const int direction, const size_t channel, const double gain )
{
	std::lock_guard<std::mutex> lock(_device_mutex);

  /* SDR-IQ 5.2.5 RF Gain */
  /* SDR-IP 4.2.3 RF Gain */
  /* SDR14 4.2.6 RF Gain */
  unsigned char atten[] = { 0x06, 0x00, 0x38, 0x00, 0x00, 0x00 };

  if ( gain <= -20 )
    atten[sizeof(atten)-1] = 0xE2;
  else if ( gain <= -10 )
    atten[sizeof(atten)-1] = 0xEC;
  else if ( gain <= 0 )
    atten[sizeof(atten)-1] = 0xF6;
  else /* +10 dB */
    atten[sizeof(atten)-1] = 0x00;

  _gain = gain;
  transaction( atten, sizeof(atten) );
}

void SoapySDR14::setGain( const int direction, const size_t channel, const std::string &name, const double gain )
{
    setGain(direction,channel,gain );
}

std::string SoapySDR14::getAntenna( const int direction, const size_t channel ) const
{
	return("RX");
}

double SoapySDR14::getBandwidth( const int direction, const size_t channel ) const
{
	return (_bandwidth);
}

double SoapySDR14::getFrequency( const int direction, const size_t channel, const std::string &name )
{
	std::lock_guard<std::mutex> lock(_device_mutex);
  /* SDR-IQ 5.2.2 Receiver Frequency */
  /* SDR-IP 4.2.2 Receiver Frequency */
  /* SDR14 4.2.3 Receiver Frequency */
  unsigned char freq[] = { 0x05, 0x20, 0x20, 0x00, 0x00 };

  std::vector< unsigned char > response;

  if ( ! transaction( freq, sizeof(freq), response ) )
    throw std::runtime_error("get_center_freq failed");

  uint32_t frequency = 0;
  frequency |= response[response.size()-5] <<  0;
  frequency |= response[response.size()-4] <<  8;
  frequency |= response[response.size()-3] << 16;
  frequency |= response[response.size()-2] << 24;

  return frequency;
}
double SoapySDR14::getFrequency(const int direction, const size_t channel)
{
	std::lock_guard<std::mutex> lock(_device_mutex);
  /* SDR-IQ 5.2.2 Receiver Frequency */
  /* SDR-IP 4.2.2 Receiver Frequency */
  /* SDR14 4.2.3 Receiver Frequency */
  unsigned char freq[] = { 0x05, 0x20, 0x20, 0x00, 0x00 };

  std::vector< unsigned char > response;

  if ( ! transaction( freq, sizeof(freq), response ) )
    throw std::runtime_error("get_center_freq failed");

  uint32_t frequency = 0;
  frequency |= response[response.size()-5] <<  0;
  frequency |= response[response.size()-4] <<  8;
  frequency |= response[response.size()-3] << 16;
  frequency |= response[response.size()-2] << 24;

  return frequency;
}

std::string SoapySDR14::getDriverKey( void ) const
{
	return("sdr-14");
}
std::vector<std::string> SoapySDR14::listGains( const int direction, const size_t channel ) const
{
	std::vector<std::string> options;
	options.push_back( "ATT" );						// RX: rf_gain
	return(options);
}
SoapySDR::RangeList SoapySDR14::getFrequencyRange( const int direction, const size_t channel, const std::string &name ) const
{
	return(getFrequencyRange(direction,channel));
}
SoapySDR::RangeList SoapySDR14::getFrequencyRange(const int direction, const size_t channel) const
{
	return(_list);
}

SoapySDR::RangeList SoapySDR14::getFrequencyRange2( const int direction, const size_t channel, const std::string &name )
{
	return(getFrequencyRange(direction,channel));
}

SoapySDR::RangeList SoapySDR14::getFrequencyRange2(const int direction, const size_t channel)
{
  /* query freq range(s) of the radio */
  uint32_t min = 0;
  uint32_t max = 30000000;

  _list.push_back(SoapySDR::Range( min, max ) );

	return(_list);
}

size_t SoapySDR14::getNumChannels( const int dir ) const
{
	//std::cerr << "nchan\n";
  if (dir == SOAPY_SDR_RX)return(_nchan);
	return(0);
}

bool SoapySDR14::getFullDuplex( const int direction, const size_t channel ) const
{
	return(false);
}
std::string SoapySDR14::getHardwareKey( void ) const
{
	return("SDR-14");
}

SoapySDR::Kwargs SoapySDR14::getHardwareInfo( void ) const
{
	std::lock_guard<std::mutex> lock(_device_mutex);
	SoapySDR::Kwargs info;
 
  info["origin"] = "https://github.com/kgarrels/SoapySDR14";
  info["index"] = "";
	return(info);

}

std::vector<std::string> SoapySDR14::listAntennas( const int direction, const size_t channel ) const
{
	std::vector<std::string> options;
	options.push_back( "RX" );
	return(options);
}
std::vector<double> SoapySDR14::listSampleRates( const int direction, const size_t channel ) const
{
	std::vector<double> options;
		options.push_back( 158730 );
	return(options);
}

SoapySDR::Range SoapySDR14::getGainRange( const int direction, const size_t channel, const std::string &name ) const
{
	return(SoapySDR::Range( -30.0, 0 ) );
}

SoapySDR::Range SoapySDR14::getGainRange( const int direction, const size_t channel) const
{
	return(SoapySDR::Range( -30.0, 0 ) );
}
double SoapySDR14::getGain( const int direction, const size_t channel, const std::string &name ) 
{
	return(getGain(direction,channel));
}

double SoapySDR14::getGain( const int direction, const size_t channel) 
{
	std::lock_guard<std::mutex> lock(_device_mutex);
  fprintf(stderr,"SoapySDR14::getGain gain\n");

  /* SDR14 4.2.6 RF Gain */
  unsigned char atten[] = { 0x05, 0x20, 0x38, 0x00, 0x00 };

  std::vector< unsigned char > response;

  if ( !transaction( atten, sizeof(atten), response ) )
      throw std::runtime_error("get_gain failed");

  unsigned char code = response[response.size()-1];
  double gain = code;

  if ( code & 0x80 )
    gain = (code & 0x7f) - 0x80;
  gain += 10;
  _gain = gain;

  return gain;
}

double SoapySDR14::getSampleRate( const int direction, const size_t channel ) const
{
	std::lock_guard<std::mutex> lock(_device_mutex);
	return(_sample_rate);
}

std::vector<double> SoapySDR14::listBandwidths( const int direction, const size_t channel ) const
{
	std::vector<double> options;
	options.push_back( 30e6 );
	return(options);
}

std::vector<std::string> SoapySDR14::listFrequencies( const int direction, const size_t channel ) const
{
	std::vector<std::string> names;
	names.push_back( "RF" );
	return(names);
}
