#ifndef TOFCAM635_DISTANCE_AMPLITUDE_IMAGE_H
#define TOFCAM635_DISTANCE_AMPLITUDE_IMAGE_H

#include "tof_cam635_image.h"

namespace ComLib
{

class TofCam635DistanceAmplitudeImage: public TofCam635Image
{
  public:
    using TofCam635Image::TofCam635Image;
    void changePixel(TofCam635Image &imageSource, const unsigned int index);
    TofCam635ImageType_e getType();
    unsigned int getDistanceOfPixel(const unsigned int index) const;
    unsigned int getAmplitudeOfPixel(const unsigned int index) const;
};

}

#endif // TOFCAM635_DISTANCE_AMPLITUDE_IMAGE_H
