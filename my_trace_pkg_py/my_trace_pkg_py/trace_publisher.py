import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# Import OpenTelemetry libraries
from opentelemetry import trace
from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter
from opentelemetry.sdk.resources import Resource
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import BatchSpanProcessor

resource = Resource(attributes={
    "service.name": "service"
})

trace.set_tracer_provider(TracerProvider(resource=resource))
tracer = trace.get_tracer(__name__)

otlp_exporter = OTLPSpanExporter(endpoint="http://localhost:4317", insecure=True)

span_processor = BatchSpanProcessor(otlp_exporter)

trace.get_tracer_provider().add_span_processor(span_processor)

class TracePublisher(Node):
    def __init__(self):
        super().__init__('trace_publisher')
        self.publisher_ = self.create_publisher(String, 'trace_topic', 10)
        self.timer_ = self.create_timer(1, self.publish_message)
        self.get_logger().info('Trace publisher node initialized')

    def publish_message(self):
        with tracer.start_as_current_span("publish_message"):
            msg = String()
            msg.data = 'hello world'
            with tracer.start_as_current_span("publish"):
                self.publisher_.publish(msg)
                self.get_logger().info('Published: %s' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    trace_publisher = TracePublisher()
    rclpy.spin(trace_publisher)
    trace_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()