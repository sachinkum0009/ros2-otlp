// Copyright (c) 2018 Intel Corporation
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


#ifndef NAV2_AMCL__SENSORS__LASER__LASER_HPP_
#define NAV2_AMCL__SENSORS__LASER__LASER_HPP_

#include <string>
#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/pf/pf_pdf.hpp"
#include "nav2_amcl/map/map.hpp"

#include "opentelemetry/exporters/ostream/span_exporter_factory.h"
#include "opentelemetry/sdk/trace/exporter.h"
#include "opentelemetry/sdk/trace/processor.h"
#include "opentelemetry/sdk/trace/simple_processor_factory.h"
#include "opentelemetry/sdk/trace/tracer_provider_factory.h"
#include "opentelemetry/trace/provider.h"

#include "opentelemetry/exporters/otlp/otlp_http_exporter_factory.h"
#include "opentelemetry/exporters/otlp/otlp_http_exporter_options.h"
#include "opentelemetry/sdk/trace/processor.h"
#include "opentelemetry/sdk/trace/batch_span_processor_factory.h"
#include "opentelemetry/sdk/trace/batch_span_processor_options.h"
#include "opentelemetry/sdk/trace/tracer_provider_factory.h"
#include "opentelemetry/trace/provider.h"
#include "opentelemetry/sdk/trace/tracer_provider.h"

#include "opentelemetry/exporters/otlp/otlp_http_metric_exporter_factory.h"
#include "opentelemetry/exporters/otlp/otlp_http_metric_exporter_options.h"
#include "opentelemetry/metrics/provider.h"
#include "opentelemetry/sdk/metrics/aggregation/default_aggregation.h"
#include "opentelemetry/sdk/metrics/export/periodic_exporting_metric_reader.h"
#include "opentelemetry/sdk/metrics/export/periodic_exporting_metric_reader_factory.h"
#include "opentelemetry/sdk/metrics/meter_context_factory.h"
#include "opentelemetry/sdk/metrics/meter_provider.h"
#include "opentelemetry/sdk/metrics/meter_provider_factory.h"

#include "opentelemetry/exporters/otlp/otlp_http_log_record_exporter_factory.h"
#include "opentelemetry/exporters/otlp/otlp_http_log_record_exporter_options.h"
#include "opentelemetry/logs/provider.h"
#include "opentelemetry/sdk/logs/logger_provider_factory.h"
#include "opentelemetry/sdk/logs/processor.h"
#include "opentelemetry/sdk/logs/simple_log_record_processor_factory.h"

#include "opentelemetry/sdk/resource/resource.h"

using namespace std;

namespace trace_api = opentelemetry::trace;
namespace trace_sdk = opentelemetry::sdk::trace;

namespace metric_sdk = opentelemetry::sdk::metrics;
namespace metrics_api = opentelemetry::metrics;

namespace otlp = opentelemetry::exporter::otlp;
namespace resource  = opentelemetry::sdk::resource;

namespace logs_api = opentelemetry::logs;
namespace logs_sdk = opentelemetry::sdk::logs;

namespace opentelemetry {
  void InitTracer();

// void InitMetrics()
// {
//   otlp::OtlpHttpMetricExporterOptions opts;
//   opts.url = "http://localhost:4318/v1/metrics";
//   auto exporter = otlp::OtlpHttpMetricExporterFactory::Create(opts);
//   metric_sdk::PeriodicExportingMetricReaderOptions reader_options;
//   reader_options.export_interval_millis = std::chrono::milliseconds(1000);
//   reader_options.export_timeout_millis  = std::chrono::milliseconds(500);
//   auto reader = metric_sdk::PeriodicExportingMetricReaderFactory::Create(std::move(exporter), reader_options);
//   auto context = metric_sdk::MeterContextFactory::Create();
//   context->AddMetricReader(std::move(reader));
//   auto u_provider = metric_sdk::MeterProviderFactory::Create(std::move(context));
//   std::shared_ptr<metrics_api::MeterProvider> provider(std::move(u_provider));
//   metrics_api::Provider::SetMeterProvider(provider);
// }

// void InitLogger()
// {
//   otlp::OtlpHttpLogRecordExporterOptions opts;
//   opts.url = "http://localhost:4318/v1/logs";
//   auto exporter  = otlp::OtlpHttpLogRecordExporterFactory::Create(opts);
//   auto processor = logs_sdk::SimpleLogRecordProcessorFactory::Create(std::move(exporter));
//   std::shared_ptr<logs_api::LoggerProvider> provider =
//       logs_sdk::LoggerProviderFactory::Create(std::move(processor));
//   logs_api::Provider::SetLoggerProvider(provider);
// }
  void CleanupTracer();

}  // namespace


namespace nav2_amcl
{

// Forward declarations
class LaserData;

/*
 * @class Laser
 * @brief Base class for laser sensor models
 */
class Laser
{
public:
  /**
   * @brief A Laser constructor
   * @param max_beams number of beams to use
   * @param map Map pointer to use
   */
  Laser(size_t max_beams, map_t * map);

  /*
   * @brief Laser destructor
   */
  virtual ~Laser();

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  virtual bool sensorUpdate(pf_t * pf, LaserData * data) = 0;

  /*
   * @brief Set the laser pose from an update
   * @param laser_pose Pose of the laser
   */
  void SetLaserPose(pf_vector_t & laser_pose);

protected:
  double z_hit_;
  double z_rand_;
  double sigma_hit_;

  /*
   * @brief Reallocate weights
   * @param max_samples Max number of samples
   * @param max_obs number of observations
   */
  void reallocTempData(int max_samples, int max_obs);
  map_t * map_;
  pf_vector_t laser_pose_;
  int max_beams_;
  int max_samples_;
  int max_obs_;
  double ** temp_obs_;
};

/*
 * @class LaserData
 * @brief Class of laser data to process
 */
class LaserData
{
public:
  Laser * laser;

  /*
   * @brief LaserData constructor
   */
  LaserData() {ranges = NULL;}
  /*
   * @brief LaserData destructor
   */
  virtual ~LaserData() {delete[] ranges;}

public:
  int range_count;
  double range_max;
  double(*ranges)[2];
};

/*
 * @class BeamModel
 * @brief Beam model laser sensor
 */
class BeamModel : public Laser
{
public:
  /*
   * @brief BeamModel constructor
   */
  BeamModel(
    double z_hit, double z_short, double z_max, double z_rand, double sigma_hit,
    double lambda_short, double chi_outlier, size_t max_beams, map_t * map);

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  bool sensorUpdate(pf_t * pf, LaserData * data);

private:
  static double sensorFunction(LaserData * data, pf_sample_set_t * set);
  double z_short_;
  double z_max_;
  double lambda_short_;
  double chi_outlier_;
};

/*
 * @class LikelihoodFieldModel
 * @brief likelihood field model laser sensor
 */
class LikelihoodFieldModel : public Laser
{
public:
  /*
   * @brief BeamModel constructor
   */
  LikelihoodFieldModel(
    double z_hit, double z_rand, double sigma_hit, double max_occ_dist,
    size_t max_beams, map_t * map);

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  bool sensorUpdate(pf_t * pf, LaserData * data);

private:
  /*
   * @brief Perform the update function
   * @param data Laser data to use
   * @param pf Particle filter to use
   * @return if it was succesful
   */
  static double sensorFunction(LaserData * data, pf_sample_set_t * set);
};

/*
 * @class LikelihoodFieldModelProb
 * @brief likelihood prob model laser sensor
 */
class LikelihoodFieldModelProb : public Laser
{
public:
  /*
   * @brief BeamModel constructor
   */
  LikelihoodFieldModelProb(
    double z_hit, double z_rand, double sigma_hit, double max_occ_dist,
    bool do_beamskip, double beam_skip_distance,
    double beam_skip_threshold, double beam_skip_error_threshold,
    size_t max_beams, map_t * map);

    ~LikelihoodFieldModelProb();

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  bool sensorUpdate(pf_t * pf, LaserData * data);

private:
  /*
   * @brief Perform the update function
   * @param data Laser data to use
   * @param pf Particle filter to use
   * @return if it was succesful
   */
  static double sensorFunction(LaserData * data, pf_sample_set_t * set);
  bool do_beamskip_;
  double beam_skip_distance_;
  double beam_skip_threshold_;
  double beam_skip_error_threshold_;
};

}  // namespace nav2_amcl

#endif  // NAV2_AMCL__SENSORS__LASER__LASER_HPP_
