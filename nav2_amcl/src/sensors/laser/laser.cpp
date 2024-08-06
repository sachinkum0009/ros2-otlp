/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <sys/types.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>

#include "nav2_amcl/sensors/laser/laser.hpp"


namespace opentelemetry {
  void InitTracer()
{
  trace_sdk::BatchSpanProcessorOptions bspOpts{};
  otlp::OtlpHttpExporterOptions opts;
  resource::ResourceAttributes attributes = {{"service.name", "amcl_node"}};
  auto resource = resource::Resource::Create(attributes);
  opts.url = "http://localhost:4318/v1/traces";
  auto exporter  = otlp::OtlpHttpExporterFactory::Create(opts);
  auto processor = trace_sdk::BatchSpanProcessorFactory::Create(std::move(exporter), bspOpts);
  std::shared_ptr<trace_api::TracerProvider> provider = trace_sdk::TracerProviderFactory::Create(std::move(processor), resource);
  trace_api::Provider::SetTracerProvider(provider);
}

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
  void CleanupTracer() {
    std::shared_ptr<opentelemetry::trace::TracerProvider> none;
    trace_api::Provider::SetTracerProvider(none);
  }

}  // namespace

namespace nav2_amcl
{

Laser::Laser(size_t max_beams, map_t * map)
: max_samples_(0), max_obs_(0), temp_obs_(NULL)
{
  max_beams_ = max_beams;
  map_ = map;
  // opentelemetry::InitTracer();
  std::cout << "opentelemetry::InitTracer()" << std::endl;
}

Laser::~Laser()
{
  if (temp_obs_) {
    for (int k = 0; k < max_samples_; k++) {
      delete[] temp_obs_[k];
    }
    delete[] temp_obs_;
  }
  std::cout << "opentelemetry::CleanupTracer()" << std::endl;

  // opentelemetry::CleanupTracer();
}

void
Laser::reallocTempData(int new_max_samples, int new_max_obs)
{
  if (temp_obs_) {
    for (int k = 0; k < max_samples_; k++) {
      delete[] temp_obs_[k];
    }
    delete[] temp_obs_;
  }
  max_obs_ = new_max_obs;
  max_samples_ = fmax(max_samples_, new_max_samples);

  temp_obs_ = new double *[max_samples_]();
  for (int k = 0; k < max_samples_; k++) {
    temp_obs_[k] = new double[max_obs_]();
  }
}

void
Laser::SetLaserPose(pf_vector_t & laser_pose)
{
  laser_pose_ = laser_pose;
}

}  // namespace nav2_amcl
