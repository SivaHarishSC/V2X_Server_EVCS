
import unittest
from unittest.mock import Mock, patch
from int_pos.int_pos import ParkingNode, parking_spots
from datetime import datetime

class TestParkingNode(unittest.TestCase):
    def setUp(self):
        rclpy_patcher = patch('int_pos.rclpy')
        self.mock_rclpy = rclpy_patcher.start()
        self.addCleanup(rclpy_patcher.stop)
        self.node = ParkingNode()
        self.node.publisher = Mock()

    def test_calculate_distance(self):
        car_position = Mock(x=4.0, y=5.0)
        spot = {"x_coordinate": 4.5, "y_coordinate": 5.5}
        distance = self.node.calculate_distance(car_position, spot)
        self.assertAlmostEqual(distance, 0.707, places=3)

    def test_rigidbody_callback(self):
        msg = Mock()
        msg.rigidbodies = [Mock(rigid_body_name="Car1", pose=Mock(position=Mock(x=4.45, y=5.20)))]
        with patch('int_pos.datetime') as mock_datetime:
            mock_datetime.now.return_value = datetime(2021, 1, 1, 12, 0, 0)
            self.node.rigidbody_callback(msg)
        self.assertFalse(self.node.parking_spots_status["0"])
        self.assertIsNotNone(parking_spots[0]["start_time"])
        self.assertIsNone(parking_spots[1]["start_time"])

    def test_publish_payment_info(self):

        with patch('int_pos.datetime') as mock_datetime:
            mock_datetime.now.return_value = datetime(2021, 1, 1, 12, 10, 0)
            self.node.publish_payment_info("0", 10, "€100.00", False, "1")
            self.node.publisher.publish.assert_called()
            call_args = self.node.publisher.publish.call_args[0][0]
            self.assertEqual(call_args.evcsn_data.charging_stations_data[0].payment, "€100.00")

    def test_publish_all_parking_status(self):
        self.node.publish_all_parking_status()
        self.node.publisher.publish.assert_called()
        call_args = self.node.publisher.publish.call_args[0][0]
        self.assertEqual(len(call_args.evcsn_data.charging_stations_data), len(parking_spots))

if __name__ == '__main__':
    unittest.main()
