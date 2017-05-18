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

#define IMG_SIZE NUM_COLORS*OSC_CAM_MAX_IMAGE_WIDTH*OSC_CAM_MAX_IMAGE_HEIGHT

const int nc = OSC_CAM_MAX_IMAGE_WIDTH;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT;

bool ManualThreshold;

/* skip pixel at border */
const int Border = 2;

/* minimum size of objects (sum of all pixels) */
const int MinArea = 500;

/* size of centroid marker */
const int SizeCross = 10;

struct OSC_VIS_REGIONS ImgRegions;/* these contain the foreground objects */
#if NUM_COLORS == 1
unsigned char OtsuThreshold(int InIndex);
void Binarize(unsigned char threshold);
#endif
void Erode_3x3(int InIndex, int OutIndex);
void Dilate_3x3(int InIndex, int OutIndex);
int* DetectRegions();
void DrawBoundingBoxes(int* color);
void ChangeDetection(void);

void ResetProcess() {
	//called when "reset" button is pressed
	if (ManualThreshold == false)
		ManualThreshold = true;
	else
		ManualThreshold = false;
}

void ProcessFrame() {
	//initialize counters
	if (data.ipc.state.nStepCounter == 1) {
		ManualThreshold = false;
	} else {
#if NUM_COLORS == 3 //if color is used, the image threshold is stored in index1

		ChangeDetection();
		int* BoxColor = DetectRegions();
		DrawBoundingBoxes(BoxColor);
		free(BoxColor);

		char Text[] = "manual threshold";
		DrawString(20, 20, strlen(Text), SMALL, CYAN, Text);

#elif NUM_COLORS == 1 //if the image is in BW, use Otsu's Method to determine the threshold

		unsigned char Threshold = OtsuThreshold(SENSORIMG);
		Binarize(Threshold);
		Erode_3x3(THRESHOLD, INDEX0);
		Dilate_3x3(INDEX0, THRESHOLD);
		if (ManualThreshold) {
			char Text[] = "manual threshold";
			DrawString(20, 20, strlen(Text), SMALL, CYAN, Text);
		} else {
			char Text[] = " Otsu's threshold";
			DrawString(20, 20, strlen(Text), SMALL, CYAN, Text);
		}

#endif


	}
}

#if NUM_COLORS == 1
void Binarize(unsigned char threshold) {
	int r, c;
	//set result buffer to zero
	memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);

	//loop over the rows
	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
		//loop over the columns
		for (c = Border; c < (nc - Border); c++) {
			//manual threshold?
			if (ManualThreshold) {
				if (data.u8TempImage[SENSORIMG][r + c]
						< data.ipc.state.nThreshold) {
					data.u8TempImage[THRESHOLD][r + c] = 255;
				}
			} else {
				if (data.u8TempImage[SENSORIMG][r + c] < threshold) {
					data.u8TempImage[THRESHOLD][r + c] = 255;
				}
			}
		}
	}
}

unsigned char OtsuThreshold(int InIndex) {
	//first part: extract gray value histogram
	unsigned int i1, best_i, K;
	float Hist[256];
	unsigned char* p = data.u8TempImage[InIndex];
	float best;
	memset(Hist, 0, sizeof(Hist));

	for (i1 = 0; i1 < nr * nc; i1++) {
		Hist[p[i1]] += 1;
	}
	//second part: determine threshold according to Otsu's method
	best = 0;
	best_i = 0;
	for (K = 0; K < 255; K++) {
		//the class accumulators
		float w0 = 0, mu0 = 0, w1 = 0, mu1 = 0;
		float bestloc;

		//class 0 and 1 probabilities and averages
		for (i1 = 0; i1 <= K; i1++) {
			w0 += Hist[i1];
			mu0 += (Hist[i1] * i1);
		}
		for (; i1 <= 255; i1++) {
			w1 += Hist[i1];
			mu1 += (Hist[i1] * i1);
		}
		//do normalization of average values
		mu0 /= w0;
		mu1 /= w1;

		bestloc = w0 * w1 * (mu0 - mu1) * (mu0 - mu1);
		if (bestloc > best) {
			best = bestloc;
			best_i = K;
			//OscLog(INFO, "%d %d %d %d %d\n", i1, w0, w1, mu0, mu1);
		}
	}
	//OscLog(INFO, "%d %f\n", best_i, best);
	return (unsigned char) best_i;
}

#endif

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

int* DetectRegions() {
	struct OSC_PICTURE Pic;
	int i;
	//set pixel value to 1 in INDEX0 because the image MUST be binary (i.e. values of 0 and 1)
	for (i = 0; i < IMG_SIZE; i++) {
		data.u8TempImage[INDEX0][i] = data.u8TempImage[INDEX1][i] ? 1 : 0;
	}

	//wrap image INDEX0 in picture struct
	Pic.data = data.u8TempImage[INDEX0];
	Pic.width = nc;
	Pic.height = nr;
	Pic.type = OSC_PICTURE_BINARY;

	//now do region labeling and feature extraction
	OscVisLabelBinary(&Pic, &ImgRegions);
	OscVisGetRegionProperties(&ImgRegions);
#if NUM_COLORS == 3
	unsigned int Hist[NUM_CHROM][256];
	unsigned int best[NUM_CHROM];
	unsigned int bestIndex[NUM_CHROM];
	int* boxColor = (int *) malloc(sizeof(int) * (ImgRegions.noOfObjects + 1));
	memset(boxColor, 0, sizeof(sizeof(int) * ImgRegions.noOfObjects + 1));
	if (boxColor == NULL) {
		return 0;
	}
	//loop over objects
	for (int o = 0; o < ImgRegions.noOfObjects; o++) {
		//get pointer to root run of current object
		struct OSC_VIS_REGIONS_RUN* currentRun = ImgRegions.objects[o].root;
		//loop over runs of current object
		memset(Hist, 0, sizeof(Hist));
		memset(best, 0, sizeof(best));
		memset(bestIndex, 0, sizeof(bestIndex));
		do {
			//loop over pixel of current run
			for (int c = currentRun->startColumn; c <= currentRun->endColumn;
					c++) {
				int r = currentRun->row;
				//loop over color planes of pixel
				for (int p = 0; p < NUM_CHROM; p++) {
					//Do as Histogram for cb and cr values
					int HistIndex = data.u8TempImage[THRESHOLD][(r * nc + c)
							* NUM_COLORS + p + 1]; //+1 to ignore y
					Hist[p][HistIndex] += 1; //increment the histogram at the corresponding cb, cr value

					//check if current histogram position is bigger than the max value
					if (Hist[p][HistIndex] > best[p]) {
						//new max value found!
						//set best to current histogram value and store the index
						best[p] = Hist[p][HistIndex];
						bestIndex[p] = HistIndex;
					}
				}
			}
			currentRun = currentRun->next;
		} while (currentRun != NULL);
		//is cr value greater than zero? (greater than 128), if true assume RED object
		(bestIndex[1] > 128) ? (*(boxColor + o) = RED) : (*(boxColor + o) = BLUE);
		//write current object to console
		printf("Cb value for object %d is %d\n", o, bestIndex[0]);
		printf("Cr value for object %d is %d\n", o, bestIndex[1]);
		if (*(boxColor + o) == RED) {
			printf("color of object %d is RED\n", o);
		} else {
			printf("color of object %d is BLUE\n", o);
		}
	}
	//clear console screen, only valid on POSIX
	printf("\e[1;1H\e[2J");
	return boxColor;
#elif NUM_COLORS == 1
	return 0;
#endif
}
void DrawBoundingBoxes(int* color) {
	uint16 o;
	for (o = 0; o < ImgRegions.noOfObjects; o++) {
		int currentColor = *(color + o);
		if (ImgRegions.objects[o].area > MinArea) {
			DrawBoundingBox(ImgRegions.objects[o].bboxLeft,
					ImgRegions.objects[o].bboxTop,
					ImgRegions.objects[o].bboxRight,
					ImgRegions.objects[o].bboxBottom, false, currentColor);

			DrawLine(ImgRegions.objects[o].centroidX - SizeCross,
					ImgRegions.objects[o].centroidY,
					ImgRegions.objects[o].centroidX + SizeCross,
					ImgRegions.objects[o].centroidY, currentColor);
			DrawLine(ImgRegions.objects[o].centroidX,
					ImgRegions.objects[o].centroidY - SizeCross,
					ImgRegions.objects[o].centroidX,
					ImgRegions.objects[o].centroidY + SizeCross, currentColor);
		}
	}
}

void ChangeDetection() {
#define NumFgrCol 2

	uint8 FrgCol[NumFgrCol][2] = { { 128 - 12, 128 + 38 },
			{ 128 + 24, 128 - 17 } };
	int r, c, frg, p;

	memset(data.u8TempImage[INDEX0], 0, IMG_SIZE);
	memset(data.u8TempImage[INDEX1], 0, IMG_SIZE);
	memset(data.u8TempImage[BACKGROUND], 0, IMG_SIZE);
	memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);

//loop over the rows
	for (r = 0; r < nr * nc; r += nc) {
//loop over the columns
		for (c = 0; c < nc; c++) {
			//convert rgb to ycbcr
			//get rgb values (order is actually bgr!)
			float B_ = data.u8TempImage[SENSORIMG][(r + c) * NUM_COLORS + 0];
			float G_ = data.u8TempImage[SENSORIMG][(r + c) * NUM_COLORS + 1];
			float R_ = data.u8TempImage[SENSORIMG][(r + c) * NUM_COLORS + 2];
			uint8 Y_ = (uint8) (0 + 0.299 * R_ + 0.587 * G_ + 0.114 * B_);
			uint8 Cb_ = (uint8) (128 - 0.169 * R_ - 0.331 * G_ + 0.500 * B_);
			uint8 Cr_ = (uint8) (128 + 0.500 * R_ - 0.419 * G_ - 0.081 * B_);
			//we write result to ImIndex
			data.u8TempImage[THRESHOLD][(r + c) * NUM_COLORS + 0] = Y_;
			data.u8TempImage[THRESHOLD][(r + c) * NUM_COLORS + 1] = Cb_;
			data.u8TempImage[THRESHOLD][(r + c) * NUM_COLORS + 2] = Cr_;

			//loop over the different Frg colors and find smallest difference
			int MinDif = 1 << 30;
			int MinInd = 0;
			for (frg = 0; frg < NumFgrCol; frg++) {
				int Dif = 0;
				//loop over the color planes (Cb,Cr) and sum up the difference, save in threshold
				for (p = 0; p < NUM_CHROM; p++) {
					Dif += abs(
							(int) data.u8TempImage[THRESHOLD][(r + c)
									* NUM_COLORS + p + 1]
									- (int) FrgCol[frg][p]);
				}
				//see if the difference is smaller than the current minimum difference
				if (Dif < MinDif) {
					MinDif = Dif;
					MinInd = frg;
				}
			}
			//if the difference is smaller than threshold value
			if (MinDif < data.ipc.state.nThreshold) {
				//set pixel value to 255 in THRESHOLD image for further processing
				//(we use only the first third of the image buffer)
				data.u8TempImage[INDEX1][(r + c)] = 255;
				//set pixel value to Frg color in BACKGROUND image for visualization
				for (p = 0; p < NUM_CHROM; p++) {
					data.u8TempImage[BACKGROUND][(r + c) * NUM_COLORS + p] =
							FrgCol[MinInd][p];
				}
			}
		}
	}
}

