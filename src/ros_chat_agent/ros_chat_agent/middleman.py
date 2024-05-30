'''
This code serves both purposes: checking latency and sending characters to the robot's hand. 
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rh8d_msgs.srv import CustomService
import time
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

        # Iterate over each character in the result
        for char in result:
            # Send each character to the robot's hand using MyClient
            response_time = self.my_client_.send_request(char)
            if response_time is not None:
                self.get_logger().info(f'Recognized: {char}, Response Time: {response_time:.6f} seconds')
                self.results.append((char, response_time))
            else:
                self.get_logger().info(f'Recognized: {char}, Response Time: None')

    def save_results(self, filename):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Character', 'Response Time'])
            for char, response_time in self.results:
                writer.writerow([char, response_time])

class MyClient:
    def __init__(self):
        self.node = rclpy.create_node('my_client')
        self.client = self.node.create_client(CustomService, 'custom_service')
        self.response_received = False
        self.client.wait_for_service(timeout_sec=1.0)  # Wait for the service to become available

    def send_request(self, char):
        attempts = 0
        gesture_received_time = time.time()
        while True:
            # Reset the flag to receive new requests
            self.response_received = False

            # Create a new Node object
            node = rclpy.create_node('my_client')

            # Create a new Client object
            client = node.create_client(CustomService, 'custom_service')
            client.wait_for_service(timeout_sec=1.0)  # Wait for the service to become available

            # Create the request and set the character
            request = CustomService.Request()
            request.request_data = char

            # Call the service
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future)

            end_time = time.time()

            # Process the response
            if future.result() is not None:
                response = future.result()
                self.node.get_logger().info('Received response: %s' % response.response_data)
                self.response_received = True
                # Calculate and store the response time
                response_time = end_time - gesture_received_time
                return response_time
                
            # Destroy the Node object
            node.destroy_node()

            # Exit the loop if maximum number of attempts reached
            attempts += 1
            if attempts >= 10:
                break

        # Return None if the request failed
        return None

    def destroy(self):
        self.node.destroy_node()

def main(args=None):
    rclpy.init(args=args)

    listener_node = ListenerNode()

    rclpy.spin(listener_node)

    listener_node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
