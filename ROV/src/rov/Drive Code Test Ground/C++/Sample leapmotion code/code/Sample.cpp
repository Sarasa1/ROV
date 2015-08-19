/******************************************************************************\
* Copyright (C) 2012-2014 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/

#include <iostream>
#include <stdio.h>
#include <string.h>
#include "Leap.h"

using namespace Leap;

int main(int argc, char** argv) {
  Controller controller;
  Frame frame;
  HandList hands;
  Hand h1;
  FingerList fingers;
  Finger index;
  Finger thumb;
  PointableList pointables;
  float x = 0, y = 0, z = 0;
  float pitch = 0, yaw = 0, roll = 0;
  
  _sleep(1000);
  while (1)
  {
	  while (controller.isConnected() == true)
	  {
		  frame = controller.frame();
		  hands = frame.hands();
		  h1 = hands[0];
		  fingers = frame.fingers();
		  thumb = fingers[0];
		  index = fingers[1];
		  pointables = frame.pointables();

		  x = h1.palmPosition().x;
		  y = h1.palmPosition().y;
		  z = h1.palmPosition().z;
		  pitch = h1.direction().pitch()*RAD_TO_DEG;
		  yaw = h1.direction().yaw()*RAD_TO_DEG;
		  roll = h1.direction().roll()*RAD_TO_DEG;

		  //std::cout << x << " " << y << " " << z << " " << pitch << " " << yaw << " "<< roll << std::endl;
		  std::cout << thumb.stabilizedTipPosition() << "\t" << index.stabilizedTipPosition() << std::endl;
		  _sleep(1000);
	  }

	  while (controller.isConnected() == false)
	  {
		  printf("No Connection ");
	  }
  }

  return 0;
}
