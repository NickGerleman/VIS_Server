#include "pch.h"
#include "Statistics.h"

using namespace std;

float calculateSD(vector<int> data) {
	float sum, mean, standardDeviation = 0.0;

	int dataSize = data.size();

	mean = calculateMean(data);

	for (int i = 0; i < dataSize; ++i) {
		standardDeviation += pow(data[i] - mean, 2);
	}

	return sqrt(standardDeviation / dataSize);
}


float calculateMean(vector<int> data) {
	float sum = 0;
	
	int dataSize = data.size();

	for (int i = 0; i < dataSize; ++i)
	{
		sum += data[i];
	}

	return sum / dataSize;
}

float findOuterFence(vector<int> data) {

	// sort data
	// find q1 and q3 outlier
	sort(data.begin(), data.end());

	int quartile1 = data.at(data.size() / 4);
	int quartile3 = data.at((data.size() / 4) * 3);

	int interQuartileRange = quartile3 - quartile1;

	return quartile3 + 3 * interQuartileRange;
}