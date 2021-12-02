#include "tof_cam635_image.h"
#include "util.h"

using namespace std;

namespace ComLib
{

static const unsigned int NUM_INTEGRATION_TIME = 3;

using namespace CommunicationConstants::TofCam635Header;


void HeaderTemporalFilterSettings::readValues(const std::vector<uint8_t> &data, const unsigned int index)
{
  factor = Util::getUint16LittleEndian(data, index);
  threshold = Util::getUint16LittleEndian(data, index+2);
}

unsigned int HeaderTemporalFilterSettings::getFactor() const
{
  return factor;
}

unsigned int HeaderTemporalFilterSettings::getThreshold() const
{
  return threshold;
}



void HeaderModulationSettings::readValues(const std::vector<uint8_t> &data)
{
  frequency = data[INDEX_MODULATION_FREQUENCY];
  channel = data[INDEX_MODULATION_CHANNEL];
}

unsigned int HeaderModulationSettings::getFrequencyMhz() const
{
  switch(frequency)
  {
    case CommunicationConstants::ModulationFrequency::VALUE_10MHZ:
      return 10;
      break;
    case CommunicationConstants::ModulationFrequency::VALUE_20MHZ:
      return 20;
      break;
    default:
      break;
  }

  return 0;
}

string HeaderModulationSettings::getFrequencyString() const
{
  return std::to_string(getFrequencyMhz()) + "MHz";
}

unsigned int HeaderModulationSettings::getChannel() const
{
  return channel;
}

bool HeaderFlags::readFlag(const uint16_t value, const uint16_t mask)
{
  if (value & mask)
  {
    return true;
  }

  return false;
}

uint16_t HeaderFlags::getFlags() const{
  return static_cast<uint16_t>(flags);
}

void HeaderFlags::readValues(const vector<uint8_t> &data)
{  
  flags = Util::getUint16LittleEndian(data, INDEX_FLAGS);
  compensated = readFlag(static_cast<uint16_t>(flags), CommunicationConstants::Flags::MASK_COMPENSATED);
  gaussianFilterEnabled = readFlag(static_cast<uint16_t>(flags), CommunicationConstants::Flags::MASK_GAUSSIAN_FILTER);
  averageFilterEnabled = readFlag(static_cast<uint16_t>(flags), CommunicationConstants::Flags::MASK_DCS_FILTER);
  autoIntegrationTimeEnabled = readFlag(static_cast<uint16_t>(flags), CommunicationConstants::Flags::MASK_AUTO_INTEGRATION_TIME);
  autoModulationEnabled = readFlag(static_cast<uint16_t>(flags), CommunicationConstants::Flags::MASK_AUTO_MODULATION);
}

bool HeaderFlags::getAutoModulationEnabled() const
{
  return autoModulationEnabled;
}

bool HeaderFlags::getAutoIntegrationTimeEnabled() const
{
  return autoIntegrationTimeEnabled;
}

bool HeaderFlags::getDcsFilterEnabled() const
{
  return averageFilterEnabled;
}

bool HeaderFlags::getGaussianFilterEnabled() const
{
  return gaussianFilterEnabled;
}

bool HeaderFlags::getCompensated() const
{
  return compensated;
}

TofCam635Image::TofCam635Image(const vector<uint8_t> &data)
{
  this->data = data;

  extractData(this->data);
}

void TofCam635Image::changePixel(TofCam635Image &imageSource, const unsigned int index)
{

}

TofCam635Image::TofCam635BeamType_e TofCam635Image::extractBeamType(const unsigned int value)
{
  switch(value)
  {
    case Beam::BEAM_A:
      return TofCam635BeamType_e::TOFCAM635_BEAM_TYPE_A;
      break;
    case Beam::BEAM_B:
      return TofCam635BeamType_e::TOFCAM635_BEAM_TYPE_B;
      break;
    case Beam::BEAM_NONE:
      return TofCam635BeamType_e::TOFCAM635_BEAM_TYPE_NONE;
      break;
    default:
      return TofCam635BeamType_e::TOFCAM635_BEAM_TYPE_NONE;
      break;
  }

  return TofCam635BeamType_e::TOFCAM635_BEAM_TYPE_NONE;
}

void TofCam635Image::extractData(const vector<uint8_t> &data)
{
  this->frameCounter = data[INDEX_VERSION];
  this->frameCounter = static_cast<uint>(Util::getInt16LittleEndian(data, INDEX_FRAME_COUNTER));
  this->timestamp = Util::getUint16LittleEndian(data, INDEX_TIMESTAMP);

  this->firmwareVersion = Util::getUint32LittleEndian(data, INDEX_FIRMWARE_VERSION);
  this->hardwareVersion = data[INDEX_HARDWARE_VERSION];
  this->chipId = Util::getUint16LittleEndian(data, INDEX_CHIP_ID);

  this->width = Util::getUint16LittleEndian(data, INDEX_WIDTH);
  this->height = Util::getUint16LittleEndian(data, INDEX_HEIGHT);
  this->originX = Util::getUint16LittleEndian(data, INDEX_ORIGIN_X);
  this->originY = Util::getUint16LittleEndian(data, INDEX_ORIGIN_Y);

  currentIntegrationTime3dWf = Util::getUint16LittleEndian(data, INDEX_CURRENT_INTEGRATION_TIME_3D_WF);
  currentIntegrationTime3dNf = Util::getUint16LittleEndian(data, INDEX_CURRENT_INTEGRATION_TIME_3D_NF);
  currentIntegrationTimeGrayscale = Util::getUint16LittleEndian(data, INDEX_CURRENT_INTEGRATION_TIME_GRAYSCALE);

  //The integration times can be extracted in a loop, because they come one after each other
  for (unsigned int i = 0; i < NUM_INTEGRATION_TIME_3D; i++)
  {
    integrationTime3d[i] = Util::getUint16LittleEndian(data, INDEX_INTEGRATION_TIME_0 + (i * sizeof(uint16_t)));
  }

  this->amplitudeLimit0 = Util::getUint16LittleEndian(data, INDEX_AMPLITUDE_LIMIT_0);
  this->amplitudeLimit1 = Util::getUint16LittleEndian(data, INDEX_AMPLITUDE_LIMIT_1);
  this->amplitudeLimit2 = Util::getUint16LittleEndian(data, INDEX_AMPLITUDE_LIMIT_2);
  this->amplitudeLimit3 = Util::getUint16LittleEndian(data, INDEX_AMPLITUDE_LIMIT_3);
  this->amplitudeLimit4 = Util::getUint16LittleEndian(data, INDEX_AMPLITUDE_LIMIT_4);

  this->offset = Util::getInt16LittleEndian(data, INDEX_OFFSET);
  this->binning = data[INDEX_BINNING];

  this->temporalFilterDistance.readValues(data, INDEX_DISTANCE_TEMPORAL_FILTER_FACTOR);
  this->temporalFilterSingleValue.readValues(data, INDEX_SINGLE_VALUE_TEMPORAL_FILTER_FACTOR);

  this->modulation.readValues(data);

  this->flags.readValues(data);

  this->temperature = Util::getInt16LittleEndian(data, INDEX_TEMPERATURE);
  this->beamType = extractBeamType(data[INDEX_ILLUMINATION_BEAM]);

  this->singleValueDistance = Util::getUint16LittleEndian(data, INDEX_BEAM_B_DISTANCE);
  this->singleValueAmplitude = Util::getUint16LittleEndian(data, INDEX_BEAM_B_AMPLITUDE);

  this->value1 = Util::getUint16LittleEndian(data, INDEX_BEAM_B_AMPLITUDE+2);
  this->value2 = Util::getUint16LittleEndian(data, INDEX_BEAM_B_AMPLITUDE+4);

}

unsigned int TofCam635Image::getHeaderVersion() const
{
  return headerVersion;
}

unsigned int TofCam635Image::getFrameCounter() const
{
  return frameCounter;
}

unsigned int TofCam635Image::getTimestamp() const
{
  return timestamp;
}


unsigned int TofCam635Image::getHardwareVersion() const
{
  return hardwareVersion;
}

unsigned int TofCam635Image::getChipID() const
{
  return chipId;
}

unsigned int TofCam635Image::getNumPixel() const
{
  return width * height;
}

unsigned int TofCam635Image::getWidth() const
{
  return width;
}

unsigned int TofCam635Image::getHeight() const
{
  return height;
}

unsigned int TofCam635Image::getOriginX() const
{
  return originX;

}

unsigned int TofCam635Image::getOriginY() const
{
  return originY;
}

unsigned int TofCam635Image::getNumIntegrationTimeUsed() const
{
  unsigned int numIntegrationTime = 0;

  //Search for the first interation time not zero
  for (unsigned int i = 0; i < NUM_INTEGRATION_TIME; i++)
  {
      if (integrationTime3d[i] == 0)
      {
          break;
      }
      numIntegrationTime++;
  }

  return numIntegrationTime;
}

unsigned int TofCam635Image::getCurrentIntegrationTime3DWF() const
{
  return currentIntegrationTime3dWf;
}

unsigned int TofCam635Image::getCurrentIntegrationTime3DNF() const
{
  return currentIntegrationTime3dNf;
}

unsigned int TofCam635Image::getCurrentIntegrationTimeGrayscale() const
{
  return currentIntegrationTimeGrayscale;
}

unsigned int TofCam635Image::getIntegrationTimeGrayscale() const
{
  return integrationTimeGrayscale;
}

unsigned int TofCam635Image::getIntegrationTime3d(const unsigned int index) const
{
  if (index >= NUM_INTEGRATION_TIME_3D)
  {
    return 0;
  }

  return integrationTime3d[index];
}

unsigned int TofCam635Image::getAmplitudeLimit(int index) const
{
  switch(index){
    case 0: return amplitudeLimit0;
    case 1: return amplitudeLimit1;
    case 2: return amplitudeLimit2;
    case 3: return amplitudeLimit3;
    case 4: return amplitudeLimit4;
  }

  return 0;
}

int TofCam635Image::getOffset() const
{
  return offset;
}

unsigned int TofCam635Image::getBinningType() const
{
  return binning;
}

HeaderModulationSettings TofCam635Image::getModulation() const
{
  return modulation;
}


HeaderTemporalFilterSettings TofCam635Image::getTemporalFilterDistance() const
{
  return temporalFilterDistance;
}

HeaderTemporalFilterSettings TofCam635Image::getTemporalFilterSingleValue() const
{
  return temporalFilterSingleValue;
}

HeaderFlags TofCam635Image::getHeaderFlags() const
{
  return flags;
}


int TofCam635Image::getTemperature() const
{
  return temperature;
}

string TofCam635Image::getTemperatureC() const
{
  return std::to_string(static_cast<double>(getTemperature()) / 100.0) + "°C";
}

unsigned int TofCam635Image::getSingleValueDistance() const
{
  return singleValueDistance;
}

string TofCam635Image::getSingleValueDistanceMm() const
{
  return convertToMmNoErrorCode(singleValueDistance);
}

unsigned int TofCam635Image::getSingleValueAmplitude() const
{
  return singleValueAmplitude;
}

unsigned int TofCam635Image::getSingleValue(int index) const{

  if(index == 0) return value1;
  else return value2;
}

bool TofCam635Image::getPixelIsSaturated(const unsigned int index) const
{
  if (getDistanceOfPixel(index) == CommunicationConstants::PixelTofCam635::VALUE_SATURATION)
  {
    return true;
  }

  return false;
}

bool TofCam635Image::getPixelIsAdcOverflow(const unsigned int index) const
{
  if (getDistanceOfPixel(index) == CommunicationConstants::PixelTofCam635::VALUE_ADC_OVERFLOW)
  {
    return true;
  }

  return false;
}

bool TofCam635Image::getPixelIsLowAmplitude(const unsigned int index) const
{
  if (getDistanceOfPixel(index) == CommunicationConstants::PixelTofCam635::VALUE_LOW_AMPLITUDE)
  {
    return true;
  }

 return false;
}

unsigned int TofCam635Image::getAmplitudeOfPixel(const unsigned int index __attribute__((unused))) const
{
  return 0;
}

unsigned int TofCam635Image::getDistanceOfPixel(const unsigned int index __attribute__((unused))) const
{
  return 0;
}

string TofCam635Image::getDistanceOfPixelMm(const unsigned int index) const
{
  return convertToMm(getDistanceOfPixel(index));
}

string TofCam635Image::convertToMm(const unsigned int distance) const
{
  if (distance == CommunicationConstants::PixelTofCam635::VALUE_ADC_OVERFLOW)
  {
    return "AdcOverflow";
  }
  if (distance == CommunicationConstants::PixelTofCam635::VALUE_SATURATION)
  {
    return "Saturation";
  }
  if (distance == CommunicationConstants::PixelTofCam635::VALUE_LOW_AMPLITUDE)
  {
    return "LowAmplitude";
  }

  string str;
  return str;     //QString::number(distance) + "mm";
}

string TofCam635Image::convertToMmNoErrorCode(const unsigned int distance) const
{
  if (distance == CommunicationConstants::PixelTofCam635::VALUE_ADC_OVERFLOW)
  {
    return "";
  }
  if (distance == CommunicationConstants::PixelTofCam635::VALUE_SATURATION)
  {
    return "";
  }
  if (distance == CommunicationConstants::PixelTofCam635::VALUE_LOW_AMPLITUDE)
  {
    return "";
  }

  //Cut off distance lower than 1m
  if (distance < 1000)
  {
    return string("");
  }
  else
  {    
    return std::to_string(distance) + "mm";
  }
}

unsigned int TofCam635Image::getGrayscaleOfPixel(const unsigned int index) const
{
  unsigned int dataOffset = CommunicationConstants::TofCam635Header::SIZE;

  //Make a pointer to the distance
  uint8_t *grayscale =  (uint8_t *)(&data.data()[dataOffset]);

  return grayscale[index];
}

TofCam635Image::TofCam635BeamType_e TofCam635Image::getBeamType() const
{
  return beamType;
}

uint16_t TofCam635Image::getFirmwareVersion() const
{
  return static_cast<uint16_t>(firmwareVersion);
}

double TofCam635Image::getFirmwareRelease() const
{
  double release = static_cast<double>((firmwareVersion >> 16)) + (static_cast<double>(firmwareVersion & 0xFFFF)) / 100.0;

  return release;
}

bool TofCam635Image::getPixelIsInvalid(const unsigned int value)
{
  if (value < CommunicationConstants::PixelTofCam635::LIMIT_VALID_PIXEL)
  {
    return false;
  }

  return true;
}

std::vector<uint8_t>& TofCam635Image::getData()
{
  return data;
}

}
