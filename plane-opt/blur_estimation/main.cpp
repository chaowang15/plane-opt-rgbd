#include <iostream>
#include <string>
#include "blur_estimation.h"
#include "../common/tools.h"
#include <chrono>
#include <map>
#include <fstream>

using namespace std;

int main(int argc, char** argv)
{
    if (argc != 4 && argc != 7)
    {
        printInRed("Usage: image_blur image_path start_frame end_frame [filename_prefix filename_suffix index_digit_number]");
        cout << "Default image filename for a frame (such as frame 1) will be like 'frame-000001.color.jpg'." << endl;
        cout << "If with input prefix, suffix and frame index digit number, the filename for frame 1 will be 'filename_prefix' "
                "+ '0001' (digit number is 4 here) + 'filename_suffix'. If index digit number is 0, then there will be no zeros"
                " padded before frame index." << endl;
        return -1;
    }
    string image_path(argv[1]);
    if (image_path.back() != '/' && image_path.back() != '\\')
        image_path += "/";

    int start_fidx = atoi(argv[2]), end_fidx = atoi(argv[3]);
    string filename_prefix("frame-"), filename_suffix(".color.jpg");
    int digit_number = 6;
    if (argc == 7)
    {
        filename_prefix = string(argv[4]);
        filename_suffix = string(argv[5]);
        digit_number = atoi(argv[6]);
    }

    printInGreen("Computing image blurriness for frames ... ");
    float progress = 0.0;  // for printing a progress bar
    int frame_num = end_fidx - start_fidx + 1;
    const int kStep = (frame_num < 100) ? 1 : (frame_num / 100);
    auto start = std::chrono::steady_clock::now();
    map<int, float> blurriness;
    for (int fidx = start_fidx; fidx <= end_fidx; ++fidx)
    {
        int current_frame = fidx - start_fidx;
		if (current_frame % kStep == 0 || fidx == end_fidx)
		{
			progress = (fidx == end_fidx) ? 1.0f : static_cast<float>(current_frame) / frame_num;
			printProgressBar(progress);
		}

        string str_fidx = std::to_string(fidx);
        string str_fidx_padded = string(digit_number - str_fidx.length(), '0') + str_fidx;
        string filename = image_path + filename_prefix + str_fidx_padded + filename_suffix;
        cv::Mat img = cv::imread(filename);
        if (!img.data)
        {
            printInRed("ERROR: cannot read image file " + filename);
            return -1;
        }
        BlurEstimation blur_est(img);
        blurriness[fidx] = blur_est.estimate();
    }
    auto end = std::chrono::steady_clock::now();
    double delta = std::chrono::duration_cast<chrono::milliseconds>(end - start).count();
    printInRed("Time: " + std::to_string(delta));

    printInGreen("Save image blurriness data into 'blur.txt'");
    std::ofstream writeout("blur.txt", std::ios::trunc);
    for (auto it : blurriness)
    {
        writeout << it.first << " " << it.second << std::endl;
    }
    writeout.close();

    return 0;
}
