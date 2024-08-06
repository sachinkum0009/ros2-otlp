#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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

#include <cstdlib>
#include <ctime>
#include <string>

using namespace std;

namespace trace_api = opentelemetry::trace;
namespace trace_sdk = opentelemetry::sdk::trace;

namespace metric_sdk = opentelemetry::sdk::metrics;
namespace metrics_api = opentelemetry::metrics;

namespace otlp = opentelemetry::exporter::otlp;
namespace resource  = opentelemetry::sdk::resource;

namespace logs_api = opentelemetry::logs;
namespace logs_sdk = opentelemetry::sdk::logs;

namespace {
  void InitTracer()
{
  trace_sdk::BatchSpanProcessorOptions bspOpts{};
  otlp::OtlpHttpExporterOptions opts;
  resource::ResourceAttributes attributes = {{"service.name", "trace_publisher"}};
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

class TracePublisher : public rclcpp::Node
{
public:
    TracePublisher() : Node("trace_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("hello_world", 10);

        message_.data = "Hello, world!";

        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TracePublisher::publishMessage, this));
        
        RCLCPP_INFO(this->get_logger(), "TracePublisher has been started.");
    }

private:
    void publishMessage()
    {
        auto tracer = opentelemetry::trace::Provider::GetTracerProvider()->GetTracer("publish-tracer");
        auto span = tracer->StartSpan("Publisher");
        publisher_->publish(message_);
        span->End();
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std_msgs::msg::String message_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    InitTracer();
    auto node = std::make_shared<TracePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    CleanupTracer();
    return 0;
}