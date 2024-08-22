import rclpy
from rclpy.node import Node
from datetime import datetime, timedelta
import tkinter as tk
from threading import Thread, Event
from PIL import Image, ImageTk
import json
import argparse
import os

from geometry_msgs.msg import Pose
from mocap_msgs.msg import RigidBodies
from v2x.msg import EvcsnPdu, ItsEVCSNData, ItsChargingStationData, ItsChargingSpotDataElements, SpotAvailability

DISTANCE_THRESHOLD = 0.3
TIMEOUT_DURATION = timedelta(seconds=5)  

def load_parking_spots(folder_path, json_filename):
    json_file_path = os.path.join(folder_path, json_filename)
    with open(json_file_path, 'r') as file:
        parking_spots = json.load(file)

    return parking_spots

class ParkingNode(Node):
    def __init__(self, folder_path, map_filename, json_filename):
        super().__init__('parking_node')

        self.parking_spots = load_parking_spots(folder_path, json_filename)

        self.subscription = self.create_subscription(
            RigidBodies, '/pose_modelcars', self.rigidbody_callback, 10)

        self.publisher = self.create_publisher(
            EvcsnPdu, 'parking_status', 10)

        self.parking_spots_status = {spot["id"]: True for spot in self.parking_spots}
        self.last_message_time = datetime.now()

        self.spot_labels = {}

        self.gui_initialized = Event()

        self.gui_thread = Thread(target=self.start_gui, args=(folder_path, map_filename, self.parking_spots))
        self.gui_thread.start()

        self.create_timer(1.0, self.check_timeout)

    def rigidbody_callback(self, msg):
        self.last_message_time = datetime.now()
        current_time = datetime.now()
        current_status = {spot["id"]: True for spot in self.parking_spots}

        for rigidbody in msg.rigidbodies:
            car_position = rigidbody.pose.position
            for spot in self.parking_spots:
                distance = self.calculate_distance(car_position, spot)
                if distance < DISTANCE_THRESHOLD:
                    current_status[spot["id"]] = False
                    spot["occupied_by"] = rigidbody.rigid_body_name
                    spot["start_time"] = current_time if spot["start_time"] is None else spot["start_time"]
                elif spot["occupied_by"] == rigidbody.rigid_body_name:
                    spot["occupied_by"] = None

        for spot_id, status in current_status.items():
            if status != self.parking_spots_status[spot_id]:
                spot = self.parking_spots[int(spot_id)]
                if not status:
                    spot["start_time"] = current_time
                else:
                    end_time = current_time
                    start_time = spot["start_time"]
                    duration = (end_time - start_time).total_seconds() / 60 if start_time else 0
                    payment = f"€{duration * 10:.2f}"
                    self.publish_payment_info(spot_id, duration, payment, status, spot_id)
                    spot["start_time"] = None

        self.parking_spots_status.update(current_status)
        self.update_gui()

    def check_timeout(self):
        current_time = datetime.now()
        if current_time - self.last_message_time > TIMEOUT_DURATION:
            self.get_logger().info("Timeout reached, no rigidbody messages received.")
            self.reset_parking_status()

    def reset_parking_status(self):
        for spot in self.parking_spots:
            spot["occupied_by"] = None
            spot["start_time"] = None
        self.parking_spots_status = {spot["id"]: True for spot in self.parking_spots}
        self.update_gui()

    def calculate_distance(self, car_position, spot):
        dx = car_position.x - spot["x_coordinate"]
        dy = car_position.y - spot["y_coordinate"]
        return (dx**2 + dy**2)**0.5

    def publish_payment_info(self, spot_id, duration, payment, available, charging_station_id):
        message = EvcsnPdu()
        message.evcsn.evcsn_data = ItsEVCSNData()

        message.evcsn.evcsn_data.charging_stations_data = []

        station_data = ItsChargingStationData()
        station_data.payment = payment
        station_data.charging_station_id = int(charging_station_id)

        spot_element = ItsChargingSpotDataElements(
            ev_equipment_id=spot_id,
            parking_places_data=[SpotAvailability(
                max_waiting_time_minutes=int(duration * 60),
                blocking=not available
            )]
        )

        station_data.charging_spots_available.append(spot_element)
        message.evcsn.evcsn_data.charging_stations_data.append(station_data)

        self.publisher.publish(message)
        self.get_logger().info(f'Published payment and station ID {charging_station_id} for spot {spot_id}: {payment}, Available: {available}')

    def start_gui(self, folder_path, map_filename, parking_spots):
        self.root = tk.Tk()
        self.root.title("Parking Spot Status")

        map_image_path = os.path.join(folder_path, map_filename)
        map_image = Image.open(map_image_path)
        self.map_photo = ImageTk.PhotoImage(map_image)

        self.canvas = tk.Canvas(self.root, width=map_image.width, height=map_image.height)
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.map_photo)
        self.canvas.grid(row=0, column=0)

        self.spot_rectangles = {}
        self.spot_texts = {}

        parking_spot_coords = [
            (415, 445, 495, 485),
            (415, 485, 495, 530),
            (415, 530, 495, 570),
            (415, 570, 495, 608)
        ]

        for spot, (x1, y1, x2, y2) in zip(parking_spots, parking_spot_coords):
            rect = self.canvas.create_rectangle(x1, y1, x2, y2, fill="green", outline="black")
            self.spot_rectangles[spot["id"]] = rect

            text = self.canvas.create_text((x1 + x2) // 2, (y1 + y2) // 2, text=spot["id"], fill="white")
            self.spot_texts[spot["id"]] = text

        self.gui_initialized.set() 
        self.update_gui()
        self.root.mainloop()

    def update_gui(self):
        self.gui_initialized.wait()
        for spot_id, status in self.parking_spots_status.items():
            color = "green" if status else "red"
            self.canvas.itemconfig(self.spot_rectangles[spot_id], fill=color)
        self.root.after(1000, self.update_gui) 

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument('--data_folder', type=str, required=True, help='Path to the folder containing the map image and JSON files')
    parser.add_argument('--map_file', type=str, required=True, help='Name of the map image file')
    parser.add_argument('--json_file', type=str, required=True, help='Name of the JSON file containing parking spot data')
    args = parser.parse_args()

    folder_path = args.data_folder
    map_filename = args.map_file
    json_filename = args.json_file

    parking_node = ParkingNode(folder_path, map_filename, json_filename)
    rclpy.spin(parking_node)
    parking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

