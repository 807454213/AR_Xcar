#ifndef APP_LOST_TRACK_STEER_H
#define APP_LOST_TRACK_STEER_H

#include "imgprocess.h"

namespace LostTrackSteer {

enum class Side : int8_t { Unknown = 0, Left = -1, Right = 1 };

void reset();
void onValidTrack(const CenterLineResult& tr, int width, int height, float lap_turn_deg);
Side rememberedSide();
bool yawTurnError(float& out_error);
float fallbackError();
const char* sideTag(Side s);

}  // namespace LostTrackSteer

#endif // APP_LOST_TRACK_STEER_H
