/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define IMG_SIZE NUM_COLORS*OSC_CAM_MAX_IMAGE_WIDTH*OSC_CAM_MAX_IMAGE_HEIGHT

const int nc = OSC_CAM_MAX_IMAGE_WIDTH;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT;

int16 imgDx[IMG_SIZE];
int16 imgDy[IMG_SIZE];

int TextColor;

float bgrImg[IMG_SIZE];

const float avgFac = 0.95;

/* skip pixel at border */
const int Border = 2;

/* after this number of steps object is set to background */
const int frgLimit = 100;

/* minimum size of objects (sum of all pixels) */
const int MinArea = 500;

struct OSC_VIS_REGIONS ImgRegions;/* these contain the foreground objects */

void ChangeDetection();
//void SetBackground();
void Erode_3x3(int InIndex, int OutIndex);
void Dilate_3x3(int InIndex, int OutIndex);
void DetectRegions();
void DrawBoundingBoxes();
void DetectAngles();

void ResetProcess() {
	//SetBackground();
}

void ProcessFrame() {
	//initialize counters
	if (data.ipc.state.nStepCounter == 1) {
//		SetBackground();

	} else {

		ChangeDetection();
		DetectAngles();

		Erode_3x3(THRESHOLD, INDEX0);
		Dilate_3x3(INDEX0, THRESHOLD);

		DetectRegions();
		DrawBoundingBoxes();

	}
}

void ChangeDetection() {
	int r, c;
	//set result buffer to zero
	memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);

	//loop over the rows
	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
		//loop over the columns
		for (c = Border; c < (nc - Border); c++) {
			//	float pImg = data.u8TempImage[SENSORIMG][r+c];
			//	float pBgr = bgrImg[r+c];
			//	float Dif = fabs(pImg-pBgr);

			unsigned char* p = &data.u8TempImage[SENSORIMG][r + c];

			/* implement Sobel filter in x-direction */
			int32 dx = -(int32) *(p - nc - 1) + (int32) *(p - nc + 1)
					- 2 * (int32) *(p - 1) + 2 * (int32) *(p + 1)
					- (int32) *(p + nc - 1) + (int32) *(p + nc + 1);

			/* implement Sobel filter in y-direction */
			int32 dy = -(int32) *(p - nc - 1) - 2 * (int32) *(p - nc)
					- (int32) *(p - nc + 1) + (int32) *(p + nc - 1)
					+ 2 * (int32) *(p + nc) + (int32) *(p + nc + 1);

			//		int32 dy = 0;
//			/* check if norm is larger than threshold */
//			int32 df2 = dx * dx + dy * dy;
//			int32 thr2 = data.ipc.state.nThreshold * data.ipc.state.nThreshold;

//		int32 dy = 0;
			/* check if norm is larger than threshold */
			int32 df2 = dx * dx + dy * dy;
			int32 thr2 = data.ipc.state.nThreshold * data.ipc.state.nThreshold;
			if (df2 > thr2) {		//avoid square root

				//if the difference is larger than threshold value (can be changed on web interface)
//			if(Dif > data.ipc.state.nThreshold) {
//				//set pixel value to 255 in BACKGROUND image (only the blue plane)
//				data.u8TempImage[THRESHOLD][r+c] = 255;
//				//increase foreground counter
//				data.u8TempImage[INDEX1][r+c]++;
//				//check whether limit is reached
//				if(data.u8TempImage[INDEX1][r+c] == frgLimit) {
//					//set pixel to background
//					bgrImg[r+c] = (float) data.u8TempImage[SENSORIMG][r+c];
//					data.u8TempImage[INDEX1][r+c] = 0;
//				}
//			} else {
//				// update background image
//				bgrImg[r+c] = avgFac*bgrImg[r+c] + (1-avgFac)*(float) data.u8TempImage[SENSORIMG][r+c];
//				// set value for display
//				data.u8TempImage[BACKGROUND][r+c] = (unsigned char) bgrImg[r+c];
//				//set foreground counter to zero
//				data.u8TempImage[INDEX1][r+c] = 1;
//			}
//			}
//		}
//	}
				//set pixel value to 255 in THRESHOLD image for gui
				data.u8TempImage[THRESHOLD][r + c] = 255;
			}
//store derivatives (int16 is enough)
			imgDx[r + c] = (int16) dx;
			imgDy[r + c] = (int16) dy;
//possibility to visualize data
			data.u8TempImage[BACKGROUND][r + c] = (uint8) MAX(0,
					MIN(255, 128+dx));
		}
	}
}

//void SetBackground() {
//	int r, c;
//
//	//loop over the rows
//	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
//		//loop over the columns
//		for (c = Border; c < (nc - Border); c++) {
//			bgrImg[r + c] = (float) data.u8TempImage[SENSORIMG][r + c];
//		}
//	}
//	//set all counters to zero
//	memset(data.u8TempImage[INDEX1], 0, IMG_SIZE);
//}

void DetectAngles() {

	int o, c;
	int angles[4] = { 0, 45, 90, 135 };
	int angleCounter[4] = { 0, 0, 0, 0 };
	int maxBinInd;

	//loop over objects
	for (o = 0; o < ImgRegions.noOfObjects; o++) {
		if (ImgRegions.objects[o].area > MinArea) {
			//get pointer to root run of current object
			struct OSC_VIS_REGIONS_RUN* currentRun = ImgRegions.objects[o].root;
			//loop over runs of current object
			do {
				//loop over pixel of current run
				for (c = currentRun->startColumn; c <= currentRun->endColumn;
						c++) {
					int r = currentRun->row;
					//processing for individual pixel at row r and column c
					double angle = atan2(imgDy[r * nc + c], imgDx[r * nc + c]);
					angle = angle * 180 / M_PI;

					if (angle < 0) {
						angle = angle + 180;
					}

					if (angle <= 22.5 || angle > 157.5) {
						angleCounter[0]++;
					}
					if (angle > 22.5 && angle <= 67.5) {
						angleCounter[1]++;
					}
					if (angle > 67.5 && angle <= 112.5) {
						angleCounter[2]++;
					}
					if (angle > 112.5 && angle <= 157.5) {
						angleCounter[3]++;
					}

				}
				currentRun = currentRun->next; //get net run of current object
			} while (currentRun != NULL); //end of current object

			int maxBin = 0;
			for (int i = 0; i < 4; i++) {

				if (angleCounter[i] > maxBin) {
					maxBin = angleCounter[i];
					maxBinInd = i;
				}
				angleCounter[i] = 0;
			}

			uint16 centerX = ImgRegions.objects[o].centroidX;
			uint16 centerY = ImgRegions.objects[o].centroidY;
			char bin[8];
			uint16 length = sprintf(bin, "%d deg", angles[maxBinInd]);
			DrawString(centerX, centerY, length, LARGE, BLUE, bin);

		}
	}

}

void Erode_3x3(int InIndex, int OutIndex) {
	int c, r;

	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
		for (c = Border; c < (nc - Border); c++) {
			unsigned char* p = &data.u8TempImage[InIndex][r + c];
			data.u8TempImage[OutIndex][r + c] = *(p - nc - 1) & *(p - nc)
					& *(p - nc + 1) & *(p - 1) & *p & *(p + 1) & *(p + nc - 1)
					& *(p + nc) & *(p + nc + 1);
		}
	}
}

void Dilate_3x3(int InIndex, int OutIndex) {
	int c, r;

	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
		for (c = Border; c < (nc - Border); c++) {
			unsigned char* p = &data.u8TempImage[InIndex][r + c];
			data.u8TempImage[OutIndex][r + c] = *(p - nc - 1) | *(p - nc)
					| *(p - nc + 1) | *(p - 1) | *p | *(p + 1) | *(p + nc - 1)
					| *(p + nc) | *(p + nc + 1);
		}
	}
}

void DetectRegions() {
	struct OSC_PICTURE Pic;
	int i;

//set pixel value to 1 in INDEX0 because the image MUST be binary (i.e. values of 0 and 1)
	for (i = 0; i < IMG_SIZE; i++) {
		data.u8TempImage[INDEX0][i] = data.u8TempImage[THRESHOLD][i] ? 1 : 0;
	}

//wrap image INDEX0 in picture struct
	Pic.data = data.u8TempImage[INDEX0];
	Pic.width = nc;
	Pic.height = nr;
	Pic.type = OSC_PICTURE_BINARY;

//now do region labeling and feature extraction
	OscVisLabelBinary(&Pic, &ImgRegions);
	OscVisGetRegionProperties(&ImgRegions);
}

void DrawBoundingBoxes() {
	uint16 o;
	for (o = 0; o < ImgRegions.noOfObjects; o++) {
		if (ImgRegions.objects[o].area > MinArea) {
			DrawBoundingBox(ImgRegions.objects[o].bboxLeft,
					ImgRegions.objects[o].bboxTop,
					ImgRegions.objects[o].bboxRight,
					ImgRegions.objects[o].bboxBottom, false, GREEN);
		}
	}
}
