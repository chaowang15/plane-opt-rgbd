/*!
    This code is the implementation of this paper:

    "The Blur Effect: Perception and Estimation with a New No-Reference Perceptual Blur Metric", Crete, et al.

    This code refers to an online code repository:
    https://github.com/tianshanfz/blur-estimation

    The difference is that this code re-implement most functions using OpenCV matrix operations instead of
    loops in the original code. This is neater and faster. You can set 'flag_matrix_method = false' to switch
    back to the original code.
*/

#ifndef BLUR_ESTIMATION_H
#define BLUR_ESTIMATION_H

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class BlurEstimation
{
public:
    BlurEstimation(const cv::Mat &input);
    ~BlurEstimation() {}

    float estimate();  // return measure of  bluriness of input, 0<=ret<=1 , higher ret means more bluriness

private:
    void blur();
    void calDifferenceVer(const cv::Mat &origin, cv::Mat &d_ver);
    void calDifferenceHor(const cv::Mat &origin, cv::Mat &d_hor);
    void calV(const cv::Mat &m1, const cv::Mat &m2, cv::Mat &_Vver);
    float sumofCoefficient(cv::Mat &d_input);
    float estimationFinal(float s_Vver, float s_Vhor, float s_Fver, float s_Fhor);

private:
    cv::Mat _F;
    cv::Mat _Bver;
    cv::Mat _Bhor;

    bool flag_matrix_method = true; // faster and neater
};

#endif  // BLUR_ESTIMATION_H
