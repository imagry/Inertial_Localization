#include "pybind11_utils.h"

cv::Mat numpy_to_cv(py::array_t<uint8_t> array) {
    // Get buffer information from the NumPy array
    py::buffer_info buf_info = array.request();

    // Create a cv::Mat using the buffer information
    // return cv::Mat(buf_info.shape[0], buf_info.shape[1], CV_8UC3,
    // buf_info.ptr);
    return cv::Mat(buf_info.shape[0], buf_info.shape[1], CV_8UC1, buf_info.ptr);
}

cv::Mat numpy_to_cv2(py::array_t<float> array) {
    // Get buffer information from the NumPy array
    py::buffer_info buf_info = array.request();

    // Create a cv::Mat using the buffer information
    return cv::Mat(buf_info.shape[0], buf_info.shape[1], CV_32F, buf_info.ptr);
}

py::array_t<uint8_t> cv_to_numpy(const cv::Mat& image) {
    py::array_t<uint8_t> result({image.rows, image.cols, image.channels()},
                                image.data);
    return result;
}

// Convert std::vector<cv::Point2f> to Python list
py::list vectorToPointList(const std::vector<cv::Point2f>& points) {
    py::list pointList;
    for (const auto& point : points) {
        py::list pointTuple;
        pointTuple.append(point.x);
        pointTuple.append(point.y);
        pointList.append(pointTuple);
    }
    return pointList;
}

// Convert Python list to std::vector<cv::Point2f>
std::vector<cv::Point2f> pointListToVector(const py::list& pointsList) {
    std::vector<cv::Point2f> pointsVector;

    for (const auto& pointItem : pointsList) {
        // Convert each element of the list to cv::Point2f
        auto pointTuple = py::cast<py::tuple>(pointItem);
        float x = py::cast<float>(pointTuple[0]);
        float y = py::cast<float>(pointTuple[1]);
        pointsVector.emplace_back(x, y);
    }

    return pointsVector;
}

// py::list doubleVectorToList(const std::vector<double>& vec) {
//     py::list doubleList;
//     for (const auto& val : vec) {
//         doubleList.append(val);
//     }
//     return doubleList;
// }

py::list floatVectorToList(const std::vector<float>& vec) {
    py::list doubleList;
    for (const auto& val : vec) {
        doubleList.append(val);
    }
    return doubleList;
}
