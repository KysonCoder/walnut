#ifndef SERVO_SERVO_CSV_PIPELINE_HPP_
#define SERVO_SERVO_CSV_PIPELINE_HPP_

#include <cstdint>
#include <string>

#include "spline/path_spline.hpp"
#include "servo_step_data.hpp"
#include "robot_transformer.hpp"
#include "spline_interpolation.hpp"
#include "ring_buffer_ipc.hpp"
#include "ring_buffer_ipc_data.hpp"

namespace servo {

constexpr uint32_t kServoCsvPathQueueCapacity = 16;
constexpr uint32_t kServoCsvStepQueueCapacity = 8192;
constexpr int kServoCsvGenerateChunkSize = 64;

struct ServoCsvPipelineConfig {
    double csv_row_interval_s = 0.064;
    double output_sample_time_s = 0.001;
    robot_config::RobotSocUnitType unit_type =
        robot_config::RobotSocUnitType::RIGHT_ARM;
};

class ServoCsvPipeline {
 public:
    ServoCsvPipeline();
    ~ServoCsvPipeline();

    // Disallow copy and assignment.
    ServoCsvPipeline(const ServoCsvPipeline&) = delete;
    ServoCsvPipeline& operator=(const ServoCsvPipeline&) = delete;

    bool Initialize(const RobotTransformer* transformer,
                    const ServoCsvPipelineConfig& config);

    bool LoadPathSplinesFromCsv(const std::string& csv_path);

    bool GenerateServoSteps();

    util::RingBufferIpc<spline::PathSplineData, kServoCsvPathQueueCapacity>& path_queue();
    util::RingBufferIpc<ServoStepData, kServoCsvStepQueueCapacity>& step_queue();

 private:
    void Cleanup();

    bool initialized_;
    const RobotTransformer* transformer_;
    ServoCsvPipelineConfig config_;

    util::RingBufferIpcData<spline::PathSplineData, kServoCsvPathQueueCapacity>* path_queue_data_;
    util::RingBufferIpcData<ServoStepData, kServoCsvStepQueueCapacity>* step_queue_data_;
    util::RingBufferIpc<spline::PathSplineData, kServoCsvPathQueueCapacity>* path_queue_;
    util::RingBufferIpc<ServoStepData, kServoCsvStepQueueCapacity>* step_queue_;
};

}  // namespace servo

#endif  // SERVO_SERVO_CSV_PIPELINE_HPP_
