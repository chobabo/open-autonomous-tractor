#!/usr/bin/env python3

import rospy
import csv
import os
import math
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header

# PyQt5 imports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
    QWidget, QLabel, QDoubleSpinBox, QPushButton, QMessageBox, QDialog
)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal

# --- RoadBoundaryInputDialog Class ---
# 새로운 웨이포인트의 도로 경계 정보를 입력받는 팝업 다이얼로그
class RoadBoundaryInputDialog(QDialog):
    def __init__(self, parent=None, x=0.0, y=0.0, theta=0.0):
        super().__init__(parent)
        self.setWindowTitle("Enter Road Boundary Info")
        self.setModal(True) # 모달 다이얼로그로 설정하여 다른 창 클릭 불가
        
        self.result = {} # 입력받은 값을 저장할 딕셔너리
        
        # 다이얼로그 레이아웃
        layout = QVBoxLayout()

        # Pose 정보 표시
        layout.addWidget(QLabel(f"New Waypoint Pose:"))
        layout.addWidget(QLabel(f"X: {x:.2f}, Y: {y:.2f}, Theta: {theta:.2f}"))

        # Left Bound 입력 필드
        h_layout_left = QHBoxLayout()
        h_layout_left.addWidget(QLabel("Left Bound (d):"))
        self.left_bound_spinbox = QDoubleSpinBox()
        self.left_bound_spinbox.setRange(-100.0, 100.0) # 적절한 범위 설정
        self.left_bound_spinbox.setValue(1.75) # 기본값 설정 (예시)
        self.left_bound_spinbox.setDecimals(2)
        self.left_bound_spinbox.setSingleStep(0.1)
        h_layout_left.addWidget(self.left_bound_spinbox)
        layout.addLayout(h_layout_left)

        # Right Bound 입력 필드
        h_layout_right = QHBoxLayout()
        h_layout_right.addWidget(QLabel("Right Bound (d):"))
        self.right_bound_spinbox = QDoubleSpinBox()
        self.right_bound_spinbox.setRange(-100.0, 100.0) # 적절한 범위 설정
        self.right_bound_spinbox.setValue(1.75) # 기본값 설정 (예시)
        self.right_bound_spinbox.setDecimals(2)
        self.right_bound_spinbox.setSingleStep(0.1)
        h_layout_right.addWidget(self.right_bound_spinbox)
        layout.addLayout(h_layout_right)

        # 버튼
        button_layout = QHBoxLayout()
        self.save_button = QPushButton("Save")
        self.save_button.clicked.connect(self.accept_input)
        button_layout.addWidget(self.save_button)

        self.cancel_button = QPushButton("Cancel")
        self.cancel_button.clicked.connect(self.reject)
        button_layout.addWidget(self.cancel_button)
        layout.addLayout(button_layout)
        
        self.setLayout(layout)

    def accept_input(self):
        self.result['left_bound_d'] = self.left_bound_spinbox.value()
        self.result['right_bound_d'] = self.right_bound_spinbox.value()
        self.accept() # 다이얼로그 닫고 QDialog.Accepted 반환

    def get_values(self):
        return self.result

# --- ROS Thread (ROS 메시지 처리를 위한 별도 스레드) ---
# PyQt GUI는 메인 스레드에서 실행되어야 하므로, ROS 콜백은 별도 스레드에서 처리
class RosThread(QThread):
    new_goal_signal = pyqtSignal(float, float, float) # x, y, theta
    
    def __init__(self):
        super().__init__()
        # ROS 노드는 메인 스레드에서 이미 초기화되었으므로, 여기서는 구독만 처리
        # rospy.init_node('waypoint_recorder_gui_ros_thread', anonymous=True) # 중복 초기화 방지
        self.goal_sub = None

    def run(self):
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.spin() # ROS 스핀을 이 스레드에서 실행

    def goal_callback(self, goal_msg):
        x = goal_msg.pose.position.x
        y = goal_msg.pose.position.y
        
        qx = goal_msg.pose.orientation.x
        qy = goal_msg.pose.orientation.y
        qz = goal_msg.pose.orientation.z
        qw = goal_msg.pose.orientation.w
        
        t3 = 2.0 * (qw * qz + qx * qy)
        t4 = 1.0 - 2.0 * (qy * qy + qz * qz)
        theta = math.atan2(t3, t4)
        
        self.new_goal_signal.emit(x, y, theta) # 메인 GUI 스레드로 시그널 발행

# --- WaypointRecorderGUI Main Window Class ---
class WaypointRecorderGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Waypoint Recorder GUI")
        self.setGeometry(100, 100, 800, 600) # 창 크기 설정
        
        # Parameters
        self.save_dir = rospy.get_param('~save_dir', os.path.expanduser('~') + '/catkin_frei/src/frei_tractor/datasets/recorded_poses')
        self.csv_filename = rospy.get_param('~csv_filename', 'recorded_poses.csv')
        self.frame_id = rospy.get_param('~frame_id', 'map')
        
        # Create save directory if it doesn't exist
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        
        self.csv_path = os.path.join(self.save_dir, self.csv_filename)
        
        self.poses = []
        self.next_id = 0
        self.load_poses_from_csv() # 기존 포즈 로드

        # ROS Publisher for Markers
        self.marker_pub = rospy.Publisher('/recorded_poses/markers_gui', MarkerArray, queue_size=10)
        
        # ROS Thread for handling subscriptions
        self.ros_thread = RosThread()
        self.ros_thread.new_goal_signal.connect(self.handle_new_goal)
        self.ros_thread.start() # ROS 스레드 시작

        # Visualization timer (for continuous marker publishing)
        self.vis_timer = QTimer(self)
        self.vis_timer.timeout.connect(self.visualize_poses)
        self.vis_timer.start(100) # 100ms (10Hz)

        self.init_ui()
        rospy.loginfo(f"Waypoint Recorder GUI started. Poses will be saved to {self.csv_path}")
        rospy.loginfo("Click on 2D Nav Goal in RViz to record poses. A dialog will pop up.")

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Simple display for recorded poses (e.g., a text area or table)
        self.pose_display = QLabel("Recorded Poses will appear here...")
        self.pose_display.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        main_layout.addWidget(self.pose_display)
        self.update_pose_display() # 초기화 시 디스플레이 업데이트

    def load_poses_from_csv(self):
        """Loads existing poses from the CSV file."""
        if os.path.exists(self.csv_path):
            with open(self.csv_path, 'r', newline='') as f:
                reader = csv.reader(f)
                header = next(reader) # Read header
                
                # Check for new columns in header
                has_bounds = 'left_bound_d' in header and 'right_bound_d' in header
                
                self.poses = [] # Clear existing poses before loading
                for row in reader:
                    if len(row) >= 4: # Minimum columns (id, x, y, theta)
                        pose_data = {
                            'id': int(row[0]),
                            'x': float(row[1]),
                            'y': float(row[2]),
                            'theta': float(row[3])
                        }
                        if has_bounds and len(row) >= 6: # Check for all 6 columns
                            try:
                                pose_data['left_bound_d'] = float(row[4])
                                pose_data['right_bound_d'] = float(row[5])
                            except ValueError:
                                rospy.logwarn(f"Invalid boundary data in row: {row}. Setting to default 0.0.")
                                pose_data['left_bound_d'] = 0.0
                                pose_data['right_bound_d'] = 0.0
                        else:
                            pose_data['left_bound_d'] = 0.0
                            pose_data['right_bound_d'] = 0.0
                        
                        self.poses.append(pose_data)
                        if pose_data['id'] >= self.next_id:
                            self.next_id = pose_data['id'] + 1
        else:
            # Create new file with header if it doesn't exist
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['id', 'x', 'y', 'theta', 'left_bound_d', 'right_bound_d'])
            rospy.loginfo(f"New CSV file created at: {self.csv_path}")

    def save_poses_to_csv(self):
        """Saves all currently recorded poses to the CSV file, overwriting existing content."""
        try:
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['id', 'x', 'y', 'theta', 'left_bound_d', 'right_bound_d'])
                for pose in self.poses:
                    writer.writerow([
                        pose['id'], 
                        pose['x'], 
                        pose['y'], 
                        pose['theta'],
                        pose.get('left_bound_d', 0.0), # Use .get() for safety
                        pose.get('right_bound_d', 0.0)
                    ])
            rospy.loginfo(f"All poses saved to {self.csv_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save poses to CSV: {e}")

    def update_pose_display(self):
        """Updates the text display with current recorded poses."""
        display_text = "Recorded Poses:\n"
        for pose in self.poses:
            display_text += f"ID {pose['id']}: X={pose['x']:.2f}, Y={pose['y']:.2f}, Theta={pose['theta']:.2f}, L={pose.get('left_bound_d', 0.0):.2f}, R={pose.get('right_bound_d', 0.0):.2f}\n"
        self.pose_display.setText(display_text)

    def handle_new_goal(self, x, y, theta):
        """
        Slot connected to new_goal_signal from RosThread.
        Opens the dialog to get boundary info.
        """
        dialog = RoadBoundaryInputDialog(self, x, y, theta)
        if dialog.exec_() == QDialog.Accepted:
            boundary_info = dialog.get_values()
            
            new_pose = {
                'id': self.next_id,
                'x': x,
                'y': y,
                'theta': theta,
                'left_bound_d': boundary_info['left_bound_d'],
                'right_bound_d': boundary_info['right_bound_d']
            }
            self.poses.append(new_pose)
            self.next_id += 1
            self.save_poses_to_csv()
            self.update_pose_display()
            rospy.loginfo(f"Successfully recorded Waypoint {new_pose['id']} with boundary info.")
        else:
            rospy.loginfo("Waypoint recording cancelled by user.")

    def visualize_poses(self):
        """Visualizes recorded poses and their boundary markers in RViz."""
        marker_array = MarkerArray()
        
        for i, pose in enumerate(self.poses):
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = self.frame_id

            # Text marker for pose info
            text_marker = Marker(header=header)
            text_marker.ns = "gui_pose_texts"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = pose['x']
            text_marker.pose.position.y = pose['y']
            text_marker.pose.position.z = 0.5
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.3
            text_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0) # White
            text_marker.text = (f"ID: {pose['id']}\n"
                                f"X: {pose['x']:.2f}, Y: {pose['y']:.2f}\n"
                                f"Theta: {pose['theta']:.2f}\n"
                                f"L: {pose.get('left_bound_d', 0.0):.2f}, R: {pose.get('right_bound_d', 0.0):.2f}")
            marker_array.markers.append(text_marker)
            
            # Arrow marker for direction
            arrow_marker = Marker(header=header)
            arrow_marker.ns = "gui_pose_arrows"
            arrow_marker.id = i
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose.position.x = pose['x']
            arrow_marker.pose.position.y = pose['y']
            arrow_marker.pose.position.z = 0.1
            
            # Set arrow orientation from theta
            arrow_marker.pose.orientation.z = math.sin(pose['theta'] / 2.0)
            arrow_marker.pose.orientation.w = math.cos(pose['theta'] / 2.0)
            
            arrow_marker.scale.x = 0.8
            arrow_marker.scale.y = 0.1
            arrow_marker.scale.z = 0.1
            arrow_marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0) # Green
            marker_array.markers.append(arrow_marker)
            
            # Visualize the valid lane width
            if 'left_bound_d' in pose and 'right_bound_d' in pose:
                # Calculate points for the boundary line segment
                # Perpendicular vector to the pose's heading
                perp_x = -math.sin(pose['theta'])
                perp_y = math.cos(pose['theta'])

                left_point = Point()
                left_point.x = pose['x'] + pose['left_bound_d'] * perp_x
                left_point.y = pose['y'] + pose['left_bound_d'] * perp_y
                left_point.z = 0.0 # On the ground
                
                right_point = Point()
                right_point.x = pose['x'] - pose['right_bound_d'] * perp_x
                right_point.y = pose['y'] - pose['right_bound_d'] * perp_y
                right_point.z = 0.0

                boundary_line_marker = Marker(header=header)
                boundary_line_marker.ns = "gui_road_boundaries"
                boundary_line_marker.id = i
                boundary_line_marker.type = Marker.LINE_LIST
                boundary_line_marker.action = Marker.ADD
                boundary_line_marker.scale.x = 0.05 # Line width
                boundary_line_marker.color = ColorRGBA(1.0, 0.5, 0.0, 0.7) # Orange transparent
                boundary_line_marker.points.append(left_point)
                boundary_line_marker.points.append(right_point)
                
                marker_array.markers.append(boundary_line_marker)
        
        self.marker_pub.publish(marker_array)

    def closeEvent(self, event):
        """Handles the window close event."""
        rospy.loginfo("Shutting down Waypoint Recorder GUI...")
        self.ros_thread.quit() # ROS 스레드 종료 요청
        self.ros_thread.wait() # ROS 스레드가 종료될 때까지 대기
        event.accept()

# --- Main execution block ---
if __name__ == '__main__':
    # ROS 노드 초기화는 QApplication이 시작되기 전에 이루어져야 합니다.
    # ROS 노드는 한 번만 초기화되어야 하며, 메인 스레드에서 시작됩니다.
    rospy.init_node('waypoint_recorder_gui', anonymous=True)

    app = QApplication([])
    main_window = WaypointRecorderGUI()
    main_window.show()
    app.exec_()