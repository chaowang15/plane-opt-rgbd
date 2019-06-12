#include "blur_estimation.h"

BlurEstimation::BlurEstimation(const cv::Mat &input)
{
    cv::Mat gray;
    cv::cvtColor(input, gray, CV_RGB2GRAY);
    gray.convertTo(_F, CV_32F);
    blur();  // F->Bver,Bhor
}

void BlurEstimation::blur()
{
    float k[9] = {1.f / 9, 1.f / 9, 1.f / 9, 1.f / 9, 1.f / 9, 1.f / 9, 1.f / 9, 1.f / 9, 1.f / 9};
    cv::Mat Km = cv::Mat(1, 9, CV_32F, k);
    cv::Mat B_ver, B_hor;

    cv::filter2D(_F, _Bver, _F.depth(), Km, cv::Point(-1, -1));      // apply filter per row
    cv::filter2D(_F, _Bhor, _F.depth(), Km.t(), cv::Point(-1, -1));  // apply filter per column
}

float BlurEstimation::estimate()
{
    cv::Mat d_Bver, d_Bhor, d_Fver, d_Fhor;
    cv::Mat Vver, Vhor;

    calDifferenceVer(_F, d_Fver);
    calDifferenceHor(_F, d_Fhor);
    calDifferenceVer(_Bver, d_Bver);
    calDifferenceHor(_Bhor, d_Bhor);

    calV(d_Fver, d_Bver, Vver);
    calV(d_Fhor, d_Bhor, Vhor);

    float s_Fver = sumofCoefficient(d_Fver);
    float s_Fhor = sumofCoefficient(d_Fhor);
    float s_Vver = sumofCoefficient(Vver);
    float s_Vhor = sumofCoefficient(Vhor);
    return estimationFinal(s_Vver, s_Vhor, s_Fver, s_Fhor);
}

float BlurEstimation::estimationFinal(float s_Vver, float s_Vhor, float s_Fver, float s_Fhor)
{
    float b_Fver = (s_Fver - s_Vver) / s_Fver;
    float b_Fhor = (s_Fhor - s_Vhor) / s_Fhor;
    return std::max(b_Fver, b_Fhor);
}

float BlurEstimation::sumofCoefficient(cv::Mat &d_input)
{
    int rows = d_input.rows;
    int cols = d_input.cols;
    if (flag_matrix_method)
    {
        // Matrix method.
        // NOTE: cv::sum() uses double instead of float by default.
        return cv::sum(d_input(cv::Rect(1, 1, cols - 1, rows - 1)))[0];
    }
    else
    {
        // Loop method. Feels like it's better to use a double variable instead of float to store the sum.
        double ret = 0;
        for (int row = 1; row < rows; row++)
            for (int col = 1; col < cols; col++)
                ret += double(d_input.at<float>(row, col));
        return float(ret);
    }
}

//!
void BlurEstimation::calV(const cv::Mat &m1, const cv::Mat &m2, cv::Mat &output)
{
    if (flag_matrix_method)
    {
        // Matrix method
        // NOTE: this code is not exactly the same as the loop method where the first row and column are
        // not updated. However, the first row and column is actually never used later, so this code still works.
        output = m1.clone();
        output = cv::max(m1 - m2, 0.0);
    }
    else
    {
        // Loop method
        output = m1.clone();
        int rows = output.rows;
        int cols = output.cols;
        for (int row = 1; row < rows; row++)
            for (int col = 1; col < cols; col++)
                output.at<float>(row, col) = std::max(0.0f, m1.at<float>(row, col) - m2.at<float>(row, col));
    }
}

//! Variation in horizontal (row) direction
void BlurEstimation::calDifferenceHor(const cv::Mat &input, cv::Mat &output)
{
    int rows = input.rows;
    int cols = input.cols;
    if (flag_matrix_method)
    {
        // Matrix method
        output = cv::Mat::zeros(input.size(), input.type());
        input(cv::Rect(0, 0, cols, rows - 1)).copyTo(output(cv::Rect(0, 1, cols, rows - 1)));
        cv::absdiff(input, output, output);
    }
    else
    {
        // Loop method
        output = input.clone();
        for (int col = 0; col < cols; col++)
            for (int row = 1; row < rows; row++)
                output.at<float>(row, col) = abs(input.at<float>(row, col) - input.at<float>(row - 1, col));
    }
}

//! Variation in vertical (column) direction
void BlurEstimation::calDifferenceVer(const cv::Mat &input, cv::Mat &output)
{
    int rows = input.rows;
    int cols = input.cols;
    if (flag_matrix_method)
    {
        // Matrix method
        output = cv::Mat::zeros(input.size(), input.type());
        input(cv::Rect(0, 0, cols - 1, rows)).copyTo(output(cv::Rect(1, 0, cols - 1, rows)));
        cv::absdiff(input, output, output);
    }
    else
    {
        // Loop method
        output = input.clone();
        for (int row = 0; row < rows; row++)
            for (int col = 1; col < cols; col++)
                output.at<float>(row, col) = abs(input.at<float>(row, col) - input.at<float>(row, col - 1));
    }
}
