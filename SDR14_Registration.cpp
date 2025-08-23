#include "SoapySDR14.hpp"
#include <SoapySDR/Registry.hpp>
#include <cstdio>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <cerrno>
#include <vector>

#ifdef _MSC_VER
#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

typedef struct
{
	std::string name;
	std::string sn;
	std::string addr;
	uint16_t port;
} unit_t;

static std::vector < unit_t > discover_sdr14();

/***********************************************************************
 * Find available devices
 **********************************************************************/
static SoapySDR::KwargsList find_sdr14(const SoapySDR::Kwargs &args)
{
	
	SoapySDR::KwargsList results;

	//locate the device on the system...
	//return a list of 0, 1, or more argument maps that each identify a device

	SoapySDR::Kwargs devInfo;
	for (const auto &unit : discover_sdr14()){

		//filter by serial when provided
		if (args.count("serial") != 0 and args.at("serial") != unit.sn) continue;

		devInfo["name"] = unit.name;
		devInfo["serial"] = unit.sn;
		devInfo["label"] = "RFSAPCE SDR-14";

		//filter out duplicates in the discovery
		int push=1;
		for(unsigned long n=0;n<results.size();++n){
			 if(results[n] == devInfo){
				push=0;
				break;
			}
		}
		if(push)results.push_back(devInfo);

	}
	return results;
}

static std::vector < unit_t > discover_sdr14()
{
  std::vector < unit_t > units;
  struct ftdi_context *ftdi;

  //std::cerr << "in discover_sdr_14 ftdi\n";

  int ret;
  struct ftdi_version_info version;
  if ((ftdi = ftdi_new()) == 0)
  {
      std::cerr << "ftdi_new failed\n";
      return units;
  }

  version = ftdi_get_library_version();
  std::cerr << "Initialized libftdi " << version.version_str << "\n";

  if ((ret = ftdi_usb_open(ftdi, 0x0403, 0xf728)) < 0)
  {
      printf("unable to open ftdi device: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
      ftdi_free(ftdi);
      return units;
  }

  //unsigned int chipid;
  //printf("ftdi_read_chipid: %d\n", ftdi_read_chipid(ftdi, &chipid));
  //printf("ftdi chipid: %X\n", chipid);

  // was only discover, close 
  if ((ret = ftdi_usb_close(ftdi)) < 0)
  {
      printf("unable to close ftdi device: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
      ftdi_free(ftdi);
      return units;
  }
  ftdi_free(ftdi);

  unit_t unit;
  unit.name = "sdr-14";
  unit.sn = "";
  unit.addr = "";
  unit.port = 0;
  units.push_back( unit );

  return units;
}

/***********************************************************************
 * Make device instance
 **********************************************************************/
static SoapySDR::Device *make_sdr14(const SoapySDR::Kwargs &args)
{
	return new SoapySDR14(args);
}

/***********************************************************************
 * Registration
 **********************************************************************/
static SoapySDR::Registry register_sdr14("sdr-14", &find_sdr14, &make_sdr14, SOAPY_SDR_ABI_VERSION);
