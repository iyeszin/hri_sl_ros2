import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rh8d_msgs.srv import CustomService
import time
import threading
import csv

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener_node')
        self.subscription_ = self.create_subscription(
            String,
            'gesture_char',
            self.callback,
            10
        )
        self.subscription_
        self.get_logger().info('Listener node initialized')
        self.my_client_ = MyClient()        
        self.results = []  # Initialize an empty list to store the results

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        result = msg.data  # Obtain the result from the message
        # self.my_client_.send_request(result)  # Send the request using MyClient

        # modified to record latency time
        gesture_received_time = time.time()
        latency = self.my_client_.send_request(result, gesture_received_time)
        if latency is not None:
            self.get_logger().info(f'Recognized: {result}, Latency: {latency:.6f} seconds')
            self.results.append((result, latency))
        else:
            self.get_logger().info(f'Recognized: {result}, Latency: None')

        self.save_results('results.csv')
        self.my_client_.destroy()

    def save_results(self, filename):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Result', 'Latency'])
            for result, latency in self.results:
                writer.writerow([result, latency])



class MyClient:
    def __init__(self):
        self.node = rclpy.create_node('my_client')
        self.client = self.node.create_client(CustomService, 'custom_service')
        self.response_received = False
        self.client.wait_for_service(timeout_sec=1.0)  # Wait for the service to become available

    def send_request(self, result, gesture_received_time):
        attempts = 0
        while True:
            # Reset the flag to receive new requests
            self.response_received = False

            # Create a new Node object
            node = rclpy.create_node('my_client')

            # Create a new Client object
            client = node.create_client(CustomService, 'custom_service')
            client.wait_for_service(timeout_sec=1.0)  # Wait for the service to become available

            # Create the request and set the result
            request = CustomService.Request()
            request.request_data = result

            start_time = time.time()

            # Call the service
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future)

            end_time = time.time()

            # Process the response
            if future.result() is not None:
                response = future.result()
                self.node.get_logger().info('Received response: %s' % response.response_data)
                self.response_received = True
                # Calculate and store the latency
                latency = end_time - gesture_received_time
                return latency
                
            # Destroy the Node object
            node.destroy_node()

            # Exit the loop if maximum number of attempts reached
            attempts += 1
            if attempts >= 10:
                break

            # Sleep for some time before sending the next request
            time.sleep(0.1)

        # Return None if gesture recognition failed
        return None

    def destroy(self):
        self.node.destroy_node()


# class MyClient:
#     def __init__(self):
#         self.node = rclpy.create_node('my_client')
#         self.client = self.node.create_client(CustomService, 'custom_service')
#         self.response_received = False

#     def send_request(self, result, gesture_received_time):
#         while True:
#             # Wait for the service to become available
#             self.node.get_logger().info('Waiting for service...')
#             self.client.wait_for_service(timeout_sec=1.0)

#             # Create the request and set the result
#             request = CustomService.Request()
#             request.request_data = result

#             start_time = time.time()
            
#             # Call the service
#             future = self.client.call_async(request)
#             rclpy.spin_until_future_complete(self.node, future)

#             end_time = time.time()

#             # Process the response
#             if future.result() is not None:
#                 response = future.result()
#                 self.node.get_logger().info('Received response: %s' % response.response_data)
#                 self.response_received = True
#             else:
#                 self.node.get_logger().info('Service call failed')

#             # Break the loop if response received, else continue to wait
#             if self.response_received:
#                 break

#             # Sleep for some time before sending the next request
#             time.sleep(1)

#         self.response_received = False  # Reset the flag to receive new requests

#         # Calculate and return the latency
#         latency = end_time - gesture_received_time
#         return latency

#     def destroy(self):
#         self.node.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    listener_node = ListenerNode()

    rclpy.spin(listener_node)

    listener_node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
