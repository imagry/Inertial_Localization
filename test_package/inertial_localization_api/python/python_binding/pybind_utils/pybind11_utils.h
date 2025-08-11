#include <opencv2/core/core.h>
#include <opencv2/imgproc/imgproc.h> // Include the imgproc header for image processing functions
#include <opencv2/opencv.h>
#include <opencv2/ximgproc.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

cv::Mat numpy_to_cv(py::array_t<uint8_t> array);
cv::Mat numpy_to_cv2(py::array_t<float> array);

py::array_t<uint8_t> cv_to_numpy(const cv::Mat &image);

py::list vectorToPointList(const std::vector<cv::Point2f> &points);

// py::list doubleVectorToList(const std::vector<double>& vec);
py::list floatVectorToList(const std::vector<float> &vec);

std::vector<cv::Point2f> pointListToVector(const py::list &pointsList);

// TEMPLATE CODE
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// TODO: Check if you can make the  template work.

// // Template function to convert NumPy array to cv::Mat
// template <typename T>
// cv::Mat numpy_to_cv(py::array_t<T> array) {
//     // Get buffer information from the NumPy array
//     py::buffer_info buf_info = array.request();

//     // Check if the data type matches the expected type
//     if (buf_info.itemsize != sizeof(T)) {
//         throw std::runtime_error("Unsupported data type.");
//     }

//     // Create a cv::Mat using the buffer information
//     return cv::Mat(buf_info.shape[0], buf_info.shape[1],
//     cv::DataDepth<T>::value, buf_info.ptr);
// }

// // // Explicit template instantiation for supported data types
// template cv::Mat numpy_to_cv<uint8_t>(py::array_t<uint8_t>);
// template cv::Mat numpy_to_cv<float>(py::array_t<float>);

// // Function to convert a cv::Mat to a NumPy array
// template <typename T>
// py::array_t<T> cv_to_numpy(const cv::Mat& image) {
//     // Determine the number of dimensions and shape
//     int ndim = image.dims;
//     std::vector<size_t> shape;
//     for (int i = 0; i < ndim; ++i) {
//         shape.push_back(static_cast<size_t>(image.size[i]));
//     }

//     // Determine the NumPy data type based on the OpenCV data type
//     py::dtype dtype = py::dtype::of<T>();

//     // Ensure the data types match
//     if (image.type() != cv::DataType<T>::type) {
//         throw std::runtime_error("Unsupported data type.");
//     }

//     // Create a buffer_info object
//     py::buffer_info buf_info(
//         image.data,                      // Pointer to the data
//         sizeof(T),                       // Size of a single item in bytes
//         py::format_descriptor<T>::format(), // Format descriptor
//         ndim,                            // Number of dimensions
//         shape,                           // Shape of the array
//         {sizeof(T) * image.cols, sizeof(T), sizeof(T)},  // Strides (assuming
//         contiguous data) false                            // Read-only flag
//     );

//     // Create a NumPy array from the buffer_info
//     return py::array_t<T>(buf_info);
// }

// // // Explicit template instantiation for supported data types
// template py::array_t<uint8_t> cv_to_numpy<uint8_t>(const cv::Mat& image);
// template py::array_t<float> cv_to_numpy<float>(const cv::Mat& image);

// ARCHIVE
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// // Template function to convert NumPy array to cv::Mat
// template <typename T>
// cv::Mat numpy_to_cv(py::array_t<T> array) {
//     // Get buffer information from the NumPy array
//     py::buffer_info buf_info = array.request();

//     // Check if the data type matches the expected type
//     if (buf_info.itemsize != sizeof(T)) {
//         throw std::runtime_error("Unsupported data type.");
//     }

//     // Create a cv::Mat using the buffer information
//     return cv::Mat(buf_info.shape[0], buf_info.shape[1],
//     cv::DataDepth<T>::value, buf_info.ptr);
// }

// // Explicit template instantiation for supported data types
// template cv::Mat numpy_to_cv<uint8_t>(py::array_t<uint8_t>);
// template cv::Mat numpy_to_cv<float>(py::array_t<float>);

// cv::Mat numpy_to_cv(py::array_t<uint8_t> array) {
//     // Get buffer information from the NumPy array
//     py::buffer_info buf_info = array.request();

//     // Create a cv::Mat using the buffer information
//     // return cv::Mat(buf_info.shape[0], buf_info.shape[1], CV_8UC3,
//     buf_info.ptr); return cv::Mat(buf_info.shape[0], buf_info.shape[1],
//     CV_8UC1, buf_info.ptr);
// }

// // Function to convert a cv::Mat to a NumPy array
// template <typename T>
// py::array_t<T> cv_to_numpy(const cv::Mat& image) {
//     // Determine the number of dimensions and shape
//     int ndim = image.dims;
//     std::vector<size_t> shape;
//     for (int i = 0; i < ndim; ++i) {
//         shape.push_back(static_cast<size_t>(image.size[i]));
//     }

//     // Determine the NumPy data type based on the OpenCV data type
//     py::dtype dtype = py::dtype::of<T>();

//     // Ensure the data types match
//     if (image.type() != cv::DataType<T>::type) {
//         throw std::runtime_error("Unsupported data type.");
//     }

//     // Create a buffer_info object
//     py::buffer_info buf_info(
//         image.data,                      // Pointer to the data
//         sizeof(T),                       // Size of a single item in bytes
//         py::format_descriptor<T>::format(), // Format descriptor
//         ndim,                            // Number of dimensions
//         shape,                           // Shape of the array
//         {sizeof(T) * image.cols, sizeof(T), sizeof(T)},  // Strides (assuming
//         contiguous data) false                            // Read-only flag
//     );

//     // Create a NumPy array from the buffer_info
//     return py::array_t<T>(buf_info);
// }

// // Explicit template instantiation for supported data types
// template py::array_t<uint8_t> cv_to_numpy<uint8_t>(const cv::Mat& image);
// template py::array_t<float> cv_to_numpy<float>(const cv::Mat& image);

// py::array_t<uint8_t> cv_to_numpy(const cv::Mat& image) {
//     py::array_t<uint8_t> result({image.rows, image.cols, image.channels()},
//     image.data); return result;
// }