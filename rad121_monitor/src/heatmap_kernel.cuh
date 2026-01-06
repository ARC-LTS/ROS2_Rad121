#pragma once

#include <opencv2/core/cuda.hpp>

// A struct to hold a single radiation sample, optimized for GPU use.
// Using float is standard for GPU work.
struct RadiationSample
{
    float x;
    float y;
    float mrem_hr;
};

// A struct to pass all necessary parameters to the GPU kernel.
struct HeatmapParams
{
    float map_origin_x;
    float map_origin_y;
    float map_resolution;
    int map_width;
    int map_height;
    float gaussian_sigma;
    float distance_cutoff_sq;
    float background_threshold_mrem_hr;
};

/**
 * @brief Launches the CUDA kernel to compute the radiation heatmap.
 * @param d_heatmap_field Output GpuMat for the heatmap (CV_32FC1).
 * @param d_occupancy_map Input GpuMat for the occupancy grid (CV_8SC1).
 * @param d_samples Input GpuMat of radiation samples (CV_32FC3).
 * @param params Struct containing all scalar parameters.
 */
void launch_heatmap_kernel(
    cv::cuda::GpuMat& d_heatmap_field,
    const cv::cuda::GpuMat& d_occupancy_map,
    const cv::cuda::GpuMat& d_samples,
    const HeatmapParams& params);