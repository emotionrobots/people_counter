
#include <iostream>
#include "communication_635.h"
#include "communication_constants.h"
#include "util.h"


using namespace std;

namespace ComLib
{

///Fixed baud rate
const unsigned int BAUDRATE = 10000000;

Communication635::Communication635()
{
    hdrMode = 0;
    indexIntegrationTime = 0;
}

/**
 * @brief Process distance amplitude
 *
 * This function processes distance amplitude specific for TofRange/Frame 635
 *
 * @param array Array with received data
 */
void Communication635::processDistanceAmplitude(const vector<uint8_t> &array)
{
    std::shared_ptr<TofCam635Image> header(new TofCam635DistanceAmplitudeImage(array));

    if(hdrMode == HDR_TEMPORAL) //temporal hdr mode
    {
        processTemporalHdr(header);
    }

    sigReceivedDistanceAmplitude(header);
}

/**
 * @brief Process distance data
 *
 * This function processes distance specific for epc635
 *
 * @param array Array with received data
 */
void Communication635::processDistance(const vector<uint8_t> &array)
{
    std::shared_ptr<TofCam635Image> header(new TofCam635DistanceImage(array));

    if(hdrMode == HDR_TEMPORAL) //temporal hdr mode
    {
        processTemporalHdr(header);
    }

    //Forward the data
    sigReceivedDistance(header);
}

/**
 * @brief Process grayscale  data
 *
 * This function processes grayscale data specific for TofRange/Frame 635
 *
 * @param array Array with received data
 */
void Communication635::processGrayscale(const vector<uint8_t> &array)
{
    std::shared_ptr<TofCam635Image> header(new TofCam635GrayscaleImage(array));

    //Forward the data
    sigReceivedGrayscale(header);
}

void Communication635::processDistanceGrayscale(const std::vector<uint8_t> &array)
{
    std::shared_ptr<TofCam635Image> header(new TofCam635DistanceGrayscaleImage(array));

    if(hdrMode == HDR_TEMPORAL) //temporal hdr mode
    {
        processTemporalHdr(header);
    }

    //Forward the data
    sigReceivedDistanceGrayscale(header);
}


void Communication635::processLensCalibrationData(const std::vector<uint8_t> &array)
{
    sigReceivedLensCalibrationData(array);
}

/**
 * @brief Setup device
 *
 * Tasks:
 * - Check, if it is a device we are looking for
 * - Setup default values per device. This is so far:
 *    - The operation mode
 *
 * @retval true It is a correct device: TofCam635
 * @retval false No correct/wanted device
 * @return Correct device true/false
 */
bool Communication635::setupDevice(const Device_e device)
{
  bool deviceIsOk = false;

  switch(device)
  {
  case Device_e::DEVICE_TOFCAM635:
    deviceIsOk = true;
    break;
  case Device_e::DEVICE_TOFCC635:
    deviceIsOk = true;
    break;
  case Device_e::DEVICE_UNKNOWN:
    //Device unknown is also ok! This is for a fabric new device and the user must be able to connect to it
    deviceIsOk = true;
    break;
  default:
    break;
  }

  return deviceIsOk;
}

/**
 * @brief Set mode
 *
 * Call this function to set the operation mode
 *
 * @param mode Mod eto set
 */
ErrorNumber_e Communication635::setMode(const unsigned int mode)
{
  switch(mode)
  {
  case TofCam635Mode_e::MODE_BEAM_A:
    sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_MODE, CommunicationConstants::ModeTofCam635::MODE_BEAM_A);
    break;
  case TofCam635Mode_e::MODE_BEAM_B_MANUAL:
    sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_MODE, CommunicationConstants::ModeTofCam635::MODE_BEAM_B_MANUAL);
    break;
  case TofCam635Mode_e::MODE_BEAM_AB_AUTO_RESULT:
    sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_MODE, CommunicationConstants::ModeTofCam635::MODE_BEAM_AB_AUTO_RESULT);
    break;
  case TofCam635Mode_e::MODE_BEAM_AB_INTERLEAVED_DATA:
    sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_MODE, CommunicationConstants::ModeTofCam635::MODE_BEAM_AB_INTERLEAVED_DATA);
    break;
  case TofCam635Mode_e::MODE_BEAM_AB_RESULT:
    sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_MODE, CommunicationConstants::ModeTofCam635::MODE_BEAM_AB_RESULT);
    break;
  case TofCam635Mode_e::MODE_BEAM_B_RESULT:
    sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_MODE, CommunicationConstants::ModeTofCam635::MODE_BEAM_B_RESULT);
    break;
  case TofCam635Mode_e::MODE_BEAM_B_RESULT_DATA:
    sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_SET_MODE, CommunicationConstants::ModeTofCam635::MODE_BEAM_B_RESULT_DATA);
    break;
  default:
    return ErrorNumber_e::ERROR_NUMBER_INVALID_PARAMETER;
  }
  return ErrorNumber_e::ERROR_NUMMBER_NO_ERROR;
}

/**
 * @brief Get the baud rate
 *
 * This function returns the baud rate for this device.
 *
 * @return baud rate
 */
unsigned int Communication635::getBaudRate()
{
  return BAUDRATE;
}



void Communication635::processTemporalHdr(std::shared_ptr<TofCam635Image> &image)
{
    unsigned int numIntegrationTimeUsed = image->getNumIntegrationTimeUsed();

    //Add the image to the buffer
    if(indexIntegrationTime < NUM_HDR_IMAGE){
        hdrImage[indexIntegrationTime] = image;
    }

    indexIntegrationTime++;

    //Last image received. Now patch them to one image.
    if((allImagesFilled(numIntegrationTimeUsed)) &&  (indexIntegrationTime >= numIntegrationTimeUsed))
    {
        indexIntegrationTime = 0;
        resultImage = hdrImage[0];

        for(uint i = 1; i < numIntegrationTimeUsed; i++)
        {
            patchImage(resultImage, hdrImage[i]);
        }
    }

    //Change to patched image only if there are enough images received
    if((allImagesFilled(numIntegrationTimeUsed)) && (resultImage.get() != nullptr))
    {
        image = resultImage;
    }
}

void Communication635::patchImage(std::shared_ptr<TofCam635Image> &imageTarget, std::shared_ptr<TofCam635Image> imageCompare)
{
    unsigned int numPixel = imageTarget->getNumPixel();

    for(uint i = 0; i < numPixel; i++)
    {
        if(imageCompare->getPixelIsInvalid(imageCompare->getDistanceOfPixel(i)) == false)
            imageTarget.get()->changePixel(*(imageCompare.get()), i);
    }
}

bool Communication635::allImagesFilled(const unsigned int numImage)
{
    for(uint i = 0; i < numImage; i++)
    {
        if(hdrImage[i].get() == nullptr)
            return false;
    }

    return true;
}







} //end namespace ComLib
