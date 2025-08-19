#include <gtest/gtest.h>
#include "Utils/StaticDynamicTest.hpp"
#include "Utils/Sensors.hpp"
#include <nlohmann/json.hpp>
#include <vector>
#include <random>
#include <functional>

/**
 * @brief Generates a vector of random doubles from a normal distribution.
 *
 * This function is designed for testing and uses a fixed seed by default to ensure
 * that the sequence of generated numbers is the same every time the test is run.
 *
 * @param num_samples The number of samples to generate.
 * @param mean The desired mean of the signal.
 * @param std_dev The desired standard deviation of the signal.
 * @param seed A seed for the random number generator to ensure reproducibility.
 * @return std::vector<double> A vector of random numbers with the specified properties.
 */
std::vector<double> GenerateRandomSignal(size_t num_samples, double mean, double std_dev, unsigned int seed = 42) {
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(mean, std_dev);

    std::vector<double> signal(num_samples);
    for (size_t i = 0; i < num_samples; ++i) {
        signal[i] = distribution(generator);
    }

    return signal;
}

struct StaticDynamicTestF : public ::testing::Test {
    nlohmann::json config;
    int buffer_size = 10;

    void SetUp() override {
        config["SD_test_acc_std_th"] = 0.1;
        config["SD_test_gyro_std_th"] = 0.1;
        config["SD_test_carspeed_mean_th"] = 0.1;
        config["SD_test_carspeed_max_th"] = 0.2;
    }
};

TEST_F(StaticDynamicTestF, Initialization) {
    StaticDynamicTest sd_test(buffer_size, buffer_size, buffer_size, config);
    ASSERT_EQ(sd_test.GetState(), StaticDynamicTest::State::NOT_INITIALIZED);
}

TEST_F(StaticDynamicTestF, StateNotInitializedUntilBuffersAreFull) {
    StaticDynamicTest sd_test(buffer_size, buffer_size, buffer_size, config);

    for (int i = 0; i < buffer_size - 1; ++i) {
        sd_test.UpdateIMU(ImuSample{});
        sd_test.UpdateCarSpeed(0.0);
        ASSERT_EQ(sd_test.GetState(), StaticDynamicTest::State::NOT_INITIALIZED);
    }
}

TEST_F(StaticDynamicTestF, TransitionToStatic) {
    StaticDynamicTest sd_test(buffer_size, buffer_size, buffer_size, config);
    
    auto acc_data = GenerateRandomSignal(buffer_size, 9.81, 1e-4);
    auto gyro_data = GenerateRandomSignal(buffer_size, 0.0, 1e-4);
    
    for(int i = 0; i < buffer_size; ++i) {
        ImuSample sample;
        sample.acc_ = {acc_data[i], 0.0, 0.0};
        sample.gyro_ = {gyro_data[i], 0.0, 0.0};
        sd_test.UpdateIMU(sample);
        sd_test.UpdateCarSpeed(0.0);
    }

    ASSERT_EQ(sd_test.GetState(), StaticDynamicTest::State::STATIC);
}

TEST_F(StaticDynamicTestF, TransitionToDynamicDueToAcc) {
    StaticDynamicTest sd_test(buffer_size, buffer_size, buffer_size, config);
    
    auto acc_data = GenerateRandomSignal(buffer_size, 9.81, 0.5);
    auto gyro_data = GenerateRandomSignal(buffer_size, 0.0, 1e-4);

    for(int i = 0; i < buffer_size; ++i) {
        ImuSample sample;
        sample.acc_ = {acc_data[i], 0.0, 0.0};
        sample.gyro_ = {gyro_data[i], 0.0, 0.0};
        sd_test.UpdateIMU(sample);
        sd_test.UpdateCarSpeed(0.0);
    }
    
    ASSERT_EQ(sd_test.GetState(), StaticDynamicTest::State::DYNAMIC);
}

TEST_F(StaticDynamicTestF, TransitionToDynamicDueToGyro) {
    StaticDynamicTest sd_test(buffer_size, buffer_size, buffer_size, config);
    
    auto acc_data = GenerateRandomSignal(buffer_size, 9.81, 1e-4);
    auto gyro_data = GenerateRandomSignal(buffer_size, 0.0, 0.5);

    for(int i = 0; i < buffer_size; ++i) {
        ImuSample sample;
        sample.acc_ = {acc_data[i], 0.0, 0.0};
        sample.gyro_ = {gyro_data[i], 0.0, 0.0};
        sd_test.UpdateIMU(sample);
        sd_test.UpdateCarSpeed(0.0);
    }
    
    ASSERT_EQ(sd_test.GetState(), StaticDynamicTest::State::DYNAMIC);
}

TEST_F(StaticDynamicTestF, TransitionToDynamicDueToSpeedMean) {
    StaticDynamicTest sd_test(buffer_size, buffer_size, buffer_size, config);
    
    auto acc_data = GenerateRandomSignal(buffer_size, 9.81, 1e-4);
    auto gyro_data = GenerateRandomSignal(buffer_size, 0.0, 1e-4);

    for(int i = 0; i < buffer_size; ++i) {
        ImuSample sample;
        sample.acc_ = {acc_data[i], 0.0, 0.0};
        sample.gyro_ = {gyro_data[i], 0.0, 0.0};
        sd_test.UpdateIMU(sample);
        sd_test.UpdateCarSpeed(0.15);
    }
    
    ASSERT_EQ(sd_test.GetState(), StaticDynamicTest::State::DYNAMIC);
}

TEST_F(StaticDynamicTestF, TransitionToDynamicDueToSpeedMax) {
    StaticDynamicTest sd_test(buffer_size, buffer_size, buffer_size, config);
    
    auto acc_data = GenerateRandomSignal(buffer_size, 9.81, 1e-4);
    auto gyro_data = GenerateRandomSignal(buffer_size, 0.0, 1e-4);

    for(int i = 0; i < buffer_size; ++i) {
        ImuSample sample;
        sample.acc_ = {acc_data[i], 0.0, 0.0};
        sample.gyro_ = {gyro_data[i], 0.0, 0.0};
        sd_test.UpdateIMU(sample);
        sd_test.UpdateCarSpeed( (i == buffer_size - 1) ? 0.25 : 0.0);
    }
    
    ASSERT_EQ(sd_test.GetState(), StaticDynamicTest::State::DYNAMIC);
}

TEST_F(StaticDynamicTestF, StaticToDynamicTransition) {
    StaticDynamicTest sd_test(buffer_size, buffer_size, buffer_size, config);
    
    // Establish STATIC state
    for(int i = 0; i < buffer_size; ++i) {
        sd_test.UpdateIMU(ImuSample{});
        sd_test.UpdateCarSpeed(0.0);
    }
    ASSERT_EQ(sd_test.GetState(), StaticDynamicTest::State::STATIC);

    // Single dynamic event
    ImuSample dynamic_sample;
    dynamic_sample.acc_ = {15.0, 0.0, 0.0}; // Large acceleration
    sd_test.UpdateIMU(dynamic_sample);
    
    ASSERT_EQ(sd_test.GetState(), StaticDynamicTest::State::DYNAMIC);
}

TEST_F(StaticDynamicTestF, DynamicToStaticTransition) {
    StaticDynamicTest sd_test(buffer_size, buffer_size, buffer_size, config);
    
    // Establish DYNAMIC state
    for(int i = 0; i < buffer_size; ++i) {
        ImuSample dynamic_sample;
        dynamic_sample.acc_ = {15.0, 0.0, 0.0};
        sd_test.UpdateIMU(dynamic_sample);
        sd_test.UpdateCarSpeed(1.0);
    }
    ASSERT_EQ(sd_test.GetState(), StaticDynamicTest::State::DYNAMIC);

    // Flush with static data
    for(int i = 0; i < buffer_size; ++i) {
        sd_test.UpdateIMU(ImuSample{});
        sd_test.UpdateCarSpeed(0.0);
    }
    
    ASSERT_EQ(sd_test.GetState(), StaticDynamicTest::State::STATIC);
}