#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from rh8d_msgs.msg import LoneJoint, AllJoints, LoneMainBoard, AllMainBoards, ClearHWError, JointListSetStiffness, JointListSetSpeedPos, SetShutdownCond

from .dynamixel_sdk import GroupSyncWrite, PacketHandler, PortHandler, COMM_SUCCESS, GroupSyncRead, DXL_LOBYTE, DXL_LOWORD, DXL_HIBYTE, DXL_HIWORD

import os
import time
import math

# Control table address and data lengths

ADDR_MODEL_NR                   = 0
LEN_MODEL_NR                    = 2
ADDR_FIRMWARE_VERSION           = 2
ADDR_BUS_ID                     = 3
ADDR_D                          = 26
ADDR_I                          = 27
ADDR_P                          = 28
ADDR_OL_FILTER_THRESHOLD        = 66
ADDR_GEARTRAIN_MODEL            = 102
ADDR_ELECTRONICS_MODEL          = 103
ADDR_PERMISSION                 = 23
LEN_PERMISSION                  = 1
PERMISSION_ENABLE               = 0
ADDR_TORQUE_ENABLE              = 24
ADDR_TARGET_POSITION            = 30
LEN_TARGET_POSITION             = 2
ADDR_TARGET_SPEED               = 32
LEN_TARGET_SPEED                = 2
ADDR_TORQUE_LIMIT               = 34
LEN_TORQUE_LIMIT                = 2
ADDR_PRESENT_POSITION           = 36
LEN_PRESENT_POSITION            = 2
ADDR_PRESENT_SPEED              = 38
LEN_PRESENT_SPEED               = 2
ADDR_TEMPERATURE                = 43
LEN_TEMPERATURE                 = 1
ADDR_MOVING                     = 46
LEN_MOVING                      = 1
ADDR_HW_ERROR_COND              = 47
LEN_HW_ERROR_COND               = 1
ADDR_OL_FILTER_VALUE            = 67
ADDR_SHUTDOWN_COND              = 18
LEN_SHUTDOWN_COND               = 1
DEFAULT_SHUTDOWN_COND           = 36
DISABLE_TEMP_COND               = 32
DISABLE_OL_COND                 = 4
DISABLE_OL_AND_TEMP             = 0
LEN_OL_FILTER_VALUE             = 1
ADDR_CURRENT                    = 68
LEN_CURRENT                     = 2
ADDR_CURRENT_LIMIT              = 71

# Main Board addresses
ADDR_PALM_IR_SENSOR             = 94
LEN_PALM_IR_SENSOR              = 2
ADDR_CAPACITIVE_SENSOR_1        = 98
ADDR_CAPACITIVE_SENSOR_2        = 100
LEN_CAPACITIVE_SENSOR           = 2

ADDR_START_SYNC_READ            = 30
LEN_SYNC_READ                   = 40

ADDR_START_SYNC_WRITE           = 30
LEN_SYNC_WRITE                  = 4

ADDR_SYNC_WRITE_STIFFNESS       = 26
LEN_SYNC_WRITE_STIFFNESS        = 3

ADDR_START_SYNC_READ_MAIN_BOARD = 94
LEN_SYNC_READ_MAIN_BOARD        = 8

# Protocol version
PROTOCOL_VERSION                = 1.0  # See which protocol version is used in the Dynamixel

# Dynamixel Constants
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4095            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold


class RH8DHWNode(Node):
    def __init__(self):
        self.name = 'rh8d_hand'
        super().__init__(self.name + '_node')

        # Declare Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('light_mode', False),
                ('hand_polarity', 'L_'),
                ('baudrate', 1000000),
                ('port', "/dev/ttyUSB0"),
                ('frequency', 50),
                ('main_board', rclpy.Parameter.Type.INTEGER),
                ('w_rotation', rclpy.Parameter.Type.INTEGER),
                ('w_adduction', rclpy.Parameter.Type.INTEGER),
                ('w_flexion', rclpy.Parameter.Type.INTEGER),
                ('th_adduction', rclpy.Parameter.Type.INTEGER),
                ('th_flexion', rclpy.Parameter.Type.INTEGER),
                ('ix_flexion', rclpy.Parameter.Type.INTEGER),
                ('middle_flexion', rclpy.Parameter.Type.INTEGER),
                ('ring_ltl_flexion', rclpy.Parameter.Type.INTEGER),
            ])

        # Get the Parameters from Config files
        try:
            self.light_mode = self.get_parameter('light_mode').value
            self.get_logger().info("Light mode is: %s" % self.light_mode)
            if self.light_mode:
                global LEN_SYNC_READ
                LEN_SYNC_READ = 10
        except KeyError:
            self.get_logger().info("Could not find light mode parameter, setting to default (false)")
            pass

        try:
            self.polarity = self.get_parameter('hand_polarity').value
            self.get_logger().info("Hand polarity is : %s" % self.polarity)
        except KeyError:
            self.get_logger().info("Could not find hand polarity parameter, setting to default (L_)")
            pass

        try:
            self.baudrate = self.get_parameter('baudrate').value
            self.get_logger().info("Baudrate is : %s" % self.baudrate)
        except KeyError:
            self.get_logger().info("Could not find baudrate parameter, setting to default (1000000)")
            pass

        try:
            self.port = self.get_parameter('port').value
            self.get_logger().info("Port name is: %s" % self.port)
        except KeyError:
            self.get_logger().info("Could not find port parameter, setting to default port (/dev/ttyUSB1)")
            pass

        try:
            self.frequency = self.get_parameter('frequency').value
            self.get_logger().info("Frequency is: %s" % self.frequency)
        except KeyError:
            self.get_logger().info("Could not find frequency parameter, setting to default port (50)")
            pass

        try:
            self.main_board_mapping = self.get_parameter('main_board').value
            self.w_rotation_mapping = self.get_parameter('w_rotation').value
            self.w_adduction_mapping = self.get_parameter('w_adduction').value
            self.w_flexion_mapping = self.get_parameter('w_flexion').value
            self.th_adduction_mapping = self.get_parameter('th_adduction').value
            self.th_flexion_mapping = self.get_parameter('th_flexion').value
            self.ix_flexion_mapping = self.get_parameter('ix_flexion').value
            self.middle_flexion_mapping = self.get_parameter('middle_flexion').value
            self.ring_ltl_flexion_mapping = self.get_parameter('ring_ltl_flexion').value
            self.get_logger().info("Joint mapping imported, Mainboard mapping is: %s" % self.main_board_mapping)
        except KeyError:
            self.get_logger().error("Could not find joint mapping dictionary : package will only work with joints IDs")
            pass

        # initialize attributes
        # Main board attributes
        self.id = None
        self.name = None
        self.palm_IR_sensor = 0
        self.capacitive_sensor_1 = 0
        self.capacitive_sensor_2 = 0

        # Flags
        self.WRITE_SPEED_POS = False
        self.WRITE_CLEAR_ERROR = False
        self.WRITE_STIFFNESS = False
        self.FORBIDDEN_STIFFNESS = False
        self.WRITE_SHUTDOWN_COND = False

        # initialize storage lists
        # id
        self.id_list = [self.main_board_mapping,
                        self.w_rotation_mapping,
                        self.w_adduction_mapping,
                        self.w_flexion_mapping,
                        self.th_adduction_mapping,
                        self.th_flexion_mapping,
                        self.ix_flexion_mapping,
                        self.middle_flexion_mapping,
                        self.ring_ltl_flexion_mapping]

        self.id_dict = {self.polarity.lower() + 'main_board': self.main_board_mapping,
                        self.polarity.lower() + 'w_rotation': self.w_rotation_mapping,
                        self.polarity.lower() + 'w_adduction': self.w_adduction_mapping,
                        self.polarity.lower() + 'w_flexion': self.w_flexion_mapping,
                        self.polarity.lower() + 'th_adduction': self.th_adduction_mapping,
                        self.polarity.lower() + 'th_flexion': self.th_flexion_mapping,
                        self.polarity.lower() + 'ix_flexion': self.ix_flexion_mapping,
                        self.polarity.lower() + 'middle_flexion': self.middle_flexion_mapping,
                        self.polarity.lower() + 'ring_ltl_flexion': self.ring_ltl_flexion_mapping}

        # reading IDs and model number
        self.dxl_read_ID_list = []
        self.dxl_model_number_list = []

        # main board ids
        self.main_board_ID_list = []

        # write target position and speed ids and values
        self.dxl_write_ID_pos_speed = []
        self.dxl_write_val_pos_speed = []

        # write stiffness id, vals, permissions
        self.dxl_write_ID_stiffness = []
        self.dxl_write_val_stiffness = []
        self.dxl_stiffnes_perm_ID = []
        self.dxl_stiffnes_perm_val = []

        # Storage lists for Joints, Mainboards for ROS Msg
        self.joints = []
        self.main_boards = []

        # storage for security class
        self.security_list = []

        # timing lists
        self.timing_write_list = []
        self.exceeding_timing_list = []
        self.timing_list = []

        # cycle variables
        self.cycle_start = 0
        self.cycle_end = 0

        ################################################################################################################
        # setup Serial Interface to Hand
        self.portHandler = PortHandler(self.port)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # initialize  GroupSyncRead instances
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_START_SYNC_READ, LEN_SYNC_READ)
        self.groupSyncReadMainBoard = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_START_SYNC_READ_MAIN_BOARD, LEN_SYNC_READ_MAIN_BOARD)

        # initialize GroupSyncWrite instances
        self.groupSyncWriteSpeedPos = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_START_SYNC_WRITE, LEN_SYNC_WRITE)
        self.groupSyncWriteClearError = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_HW_ERROR_COND, LEN_HW_ERROR_COND)
        self.groupSyncWriteStiffness = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_SYNC_WRITE_STIFFNESS, LEN_SYNC_WRITE_STIFFNESS)
        self.groupSyncWritePermission = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_PERMISSION, LEN_PERMISSION)
        self.groupSyncWriteSetShutdownCond = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_SHUTDOWN_COND, LEN_SHUTDOWN_COND)

        ################################################################################################################
        # Open Connection to Hand
        try:
            self.portHandler.openPort()
            self.get_logger().info('Succeeded to open the port!')
        except:
            self.get_logger().info("Failed to open the port!")
            self.get_logger().info("Execution stops in 3 secs!")
            time.sleep(3)
            quit()

        try:
            self.portHandler.setBaudRate(self.baudrate)
            self.get_logger().info('Succeeded to change the baudrate!')
        except:
            self.get_logger().info("Failed to change the baudrate!")
            self.get_logger().info("Execution stops in 3 secs!")
            time.sleep(3)
            quit()

        # Scan the buses and check if all addresses are found
        time.sleep(1)
        self.pingScan()
        all_IDs_list_ = self.dxl_read_ID_list + self.main_board_ID_list

        if set(all_IDs_list_) != set(self.id_list):
            self.get_logger().warn("Bus scan did not find every joint declared in the YAML dictionary")
        else:
            self.get_logger().info("Bus scan went well, every joint in YAML dictionary was found")

        # initialize Joints & Mainboards
        self.initJoints()
        # for joint in self.joints:
        #     joint.printJointInfo()

        self.initMainBoards()
        # for main_board in self.main_boards:
        #     main_board.printMainBoardInfo()


        # set up all publishing messages
        self.lone_joint_list = [LoneJoint() for _ in range(len(self.dxl_read_ID_list))]
        self.all_joints_msg = AllJoints()

        self.lone_main_boards_list = [LoneMainBoard() for _ in range(len(self.main_board_ID_list))]
        self.all_main_boards_msg = AllMainBoards()

        # define callback groups
        self.callback_group1 = MutuallyExclusiveCallbackGroup()
        self.callback_group2 = MutuallyExclusiveCallbackGroup()

        # ROS2 subscriber and publisher set upt
        self.clearError_subscriber = self.create_subscription(ClearHWError, self.polarity + 'clearHWError', self.clearErrorCallback, 10, callback_group=self.callback_group1)
        self.clearError_subscriber

        self.setShutdownCond_subscriber = self.create_subscription(SetShutdownCond, self.polarity + 'setShutdownCond', self.setShutDownCondCallback, 10, callback_group=self.callback_group1)
        self.setShutdownCond_subscriber

        self.setStiffness_subscriber = self.create_subscription(JointListSetStiffness, self.polarity + 'setStiffnes', self.setStiffnessCallback, 10, callback_group=self.callback_group1)
        self.setStiffness_subscriber

        self.setSpeedPos_subscriber = self.create_subscription(JointListSetSpeedPos, self.polarity + 'setSpeedPos', self.setSpeedPosCallback, 10, callback_group=self.callback_group1)
        self.setSpeedPos_subscriber

        self.joint_publisher_ = self.create_publisher(AllJoints, self.polarity + 'Joints', 10)
        self.main_board_publisher_ = self.create_publisher(AllMainBoards, self.polarity + 'MainBoards', 10)

        # main loop callback
        self.timer_period = 1 / self.frequency
        self.timer = self.create_timer(self.timer_period, self.mainLoopCallback, callback_group=self.callback_group2)

    def pingScan(self):
        self.get_logger().info("Start ping scan!")
        for id_ in range(253):
            # try to ping the Dynamixel
            # Get Dynamixel model number and store them in a list
            dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, id_)
            # time.sleep(0.0)
            # self.get_logger().info("Ping # %03d: COMM_SUCCESS: %s, ERROR: %d" % (id_, self.packetHandler.getTxRxResult(dxl_comm_result), dxl_error))
            if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
                self.get_logger().info("[ID: %03d] ping succeeded! Dynamixel model number: %d" % (id_, dxl_model_number))
                if dxl_model_number != 405:
                    self.dxl_read_ID_list.append(id_)
                    self.dxl_model_number_list.append(dxl_model_number)
                else:
                    self.main_board_ID_list.append(id_)

    def initJoints(self):
        for index, id_ in enumerate(self.dxl_read_ID_list):
            temp_model_number = self.dxl_model_number_list[index]
            temp_id = id_
            temp_name = [name for name, i in self.id_dict.items() if i == id_]
            temp_joint = Joint(self, temp_id, temp_name, temp_model_number)
            self.joints.append(temp_joint)

        for joint in self.joints:
            joint.firmware = self.readInfo(joint.id, ADDR_FIRMWARE_VERSION)
            joint.bus_id = self.readInfo(joint.id, ADDR_BUS_ID)
            joint.geartrain_model = self.readInfo(joint.id, ADDR_GEARTRAIN_MODEL)
            joint.electronics_model = self.readInfo(joint.id, ADDR_ELECTRONICS_MODEL)

            joint.kp = self.readInfo(joint.id, ADDR_P)
            joint.ki = self.readInfo(joint.id, ADDR_I)
            joint.kd = self.readInfo(joint.id, ADDR_D)
            joint.overload_filter_threshold = self.readInfo(joint.id, ADDR_OL_FILTER_THRESHOLD)

            temp_check_PID_change = self.readInfo(joint.id, ADDR_PERMISSION)
            if temp_check_PID_change == 0:
                self.FORBIDDEN_STIFFNESS = True

            temp_current_limit, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, joint.id ,ADDR_CURRENT_LIMIT)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                self.get_logger().info("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                joint.current_limit = temp_current_limit

            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, joint.id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                self.get_logger().info("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                self.get_logger().info("Dynamixel #%d has been successfully connected" % joint.id)

            dxl_addparam_result = self.groupSyncRead.addParam(joint.id)
            if not dxl_addparam_result:
                self.get_logger().error("[ID:%03d] groupSyncRead addparam failed" % joint.id)
                quit()
        if self.WRITE_STIFFNESS:
            self.get_logger().info("ROS Package started after stiffness change. Stiffness commands will be ignored."
                                   " Power cycle Unit to restore stiffness change.")

    def initMainBoards(self):
        for i in self.main_board_ID_list:
            try:
                name_ = [name for name, j in self.id_dict.items() if j == i]
                temp_name = self.polarity + name_[0]
            except IndexError:
                temp_name = 'None'
                self.get_logger().warn("Main board with ID %d has not met its mapping, name set to 'None'" % i)

            temp_main_board = MainBoard(self, i, temp_name)
            self.main_boards.append(temp_main_board)
            dxl_addparam_result = self.groupSyncReadMainBoard.addParam(i)
            if not dxl_addparam_result:
                self.get_logger().error("[ID:%03d] groupSyncReadMainBoard addparam failed" % i)
                quit()



    def readInfo(self, id_, address):
        info, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, id_, address)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.get_logger().info("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            return info

    def getIDfromName(self, name):
        if name.isnumeric():
            id_ = int(name)
            return id_

        else:
            try:
                id_ = self.id_dict[name]
            except KeyError:
                id_ = 'None'
                self.get_logger().warn("KeyError raised while trying to match joint %s with its ID. Check the dictionary declaration in the YAML file. Ignore this if you're using 2 hands on 2 different ports." % name)
            return id_

    def getPIDStiffness(self, stiffness, _id):
        joint = [j for j in self.joints if j.bus_id == _id]
        if not joint:
            self.get_logger().warn("[ID: %d] is not used! Could not get stiffness!" % _id)
            return
        else:
            # Lowest stiffness: default P/2
            if stiffness == 1:
                kp = math.ceil(joint[0].kp / 2)
            elif stiffness == 8:
                kp = joint[0].kp
            elif 1 < stiffness < 8:
                kp = math.ceil(1.42 * stiffness + 8.5)
            elif stiffness == 9:
                kp = math.ceil(25 * stiffness - 180)
            else:
                kp = math.ceil(joint[0].kp / 2)
                self.get_logger().info("Stiffness not in range of [1,9] set to default!")

            ki = joint[0].ki
            kd = joint[0].kd
            return kp, ki, kd

    # ROS2 Message fill methods
    def fillJointMsg(self):
        for index, lone_joint in enumerate(self.lone_joint_list):
            # Go through all the IDs and fill ROS message field one by one
            lone_joint.name = self.joints[index].name[0]
            lone_joint.bus_id = self.joints[index].bus_id
            lone_joint.stiffness = self.joints[index].stiffness
            lone_joint.target_position = self.joints[index].target_pos
            lone_joint.target_speed = self.joints[index].target_speed
            lone_joint.torque_limit = self.joints[index].torque_limit
            lone_joint.present_position = self.joints[index].pres_pos
            lone_joint.temperature = self.joints[index].temperature
            lone_joint.hw_error_condition = self.joints[index].hw_err_cond
            lone_joint.present_speed = self.joints[index].pres_speed
            if not self.light_mode:
                lone_joint.stress_level = self.joints[index].stress_level
                lone_joint.moving = self.joints[index].moving
                lone_joint.current = self.joints[index].pres_current

        # Once every lone_joint_list element is filled, fill the alljoints_msg that will be sent
        self.all_joints_msg.header.stamp = self.get_clock().now().to_msg()
        self.all_joints_msg.length = len(self.dxl_read_ID_list)
        self.all_joints_msg.joints = self.lone_joint_list

    def fillMainBoardMessages(self):
        for index, lone_main_board in enumerate(self.lone_main_boards_list):
            lone_main_board.name = str(self.main_boards[index].name)
            lone_main_board.id = self.main_boards[index].id
            # lone_main_board.palm_IR_sensor = self.main_boards[index].palm_IR_sensor
            lone_main_board.capacitive_sensor_1 = self.main_boards[index].capacitive_sensor_1
            lone_main_board.capacitive_sensor_2 = self.main_boards[index].capacitive_sensor_2

        self.all_main_boards_msg.header.stamp = self.get_clock().now().to_msg()
        self.all_main_boards_msg.length = len(self.main_board_ID_list)
        self.all_main_boards_msg.boards = self.lone_main_boards_list

    # Callback functions
    def clearErrorCallback(self, msg):
        # Clearing previous parameters for syncWrite, clearing list variables used to add these parameters
        self.groupSyncWriteClearError.clearParam()

        id_ = self.getIDfromName(msg.name)
        if id_ != 'None':
            # Check security timing:
            for i in self.security_list:
                if id_ == i.id:
                    if i.time_last_clear is not None:
                        timing_check = time.time_ns()/1000000000 - i.time_last_clear
                        if timing_check < 30:
                            self.get_logger().warn("Trying to clear error on joint %s too frequently : command discarded" % msg.name)
                            return
                    i.time_last_clear = time.time_ns()/1000000000
            param = [DXL_LOBYTE(DXL_LOWORD(0))]
            dxl_add_param_result = self.groupSyncWriteClearError.addParam(id_, param)
            if not dxl_add_param_result:
                self.get_logger().error("groupSyncWriteClearError addparam failed on joint %s" % msg.name)
                return
        else:
            self.get_logger().warn("Cannot clear error for joint %s, abort writing. Ignore this if you're using 2 hands on 2 different ports." % msg.name)
            return
        self.WRITE_CLEAR_ERROR = True

    def setShutDownCondCallback(self, msg):
        self.groupSyncWriteSetShutdownCond.clearParam()
        id_ = self.getIDfromName(msg.name)
        if id_ != 'None':
            if msg.temperature is True and msg.overload is True:
                param = [DXL_LOBYTE(DXL_LOWORD(DEFAULT_SHUTDOWN_COND))]
            elif msg.temperature is False and msg.overload is True:
                param = [DXL_LOBYTE(DXL_LOWORD(DISABLE_TEMP_COND))]
            elif msg.temperature is True and msg.overload is False:
                param = [DXL_LOBYTE(DXL_LOWORD(DISABLE_OL_COND))]
            elif msg.temperature is False and msg.overload is False:
                param = [DXL_LOBYTE(DXL_LOWORD(DISABLE_OL_AND_TEMP))]
            else:
                return

            dxl_add_param_result = self.groupSyncWriteSetShutdownCond.addParam(id_, param)
            if dxl_add_param_result != True:
                self.get_logger().error("groupSyncWriteSetShutdownCond add param failed on joint %s" % msg.name)
                return
        else:
            self.get_logger().warn("Cannot set shutdown condition for joint %s, abort writing. Ignore this if you're using 2 hands on 2 different ports." % msg.name)
            return

        self.WRITE_SHUTDOWN_COND = True

    def setStiffnessCallback(self, msg):
        if self.WRITE_STIFFNESS:
            self.get_logger().warn("ROS Package started after stiffness change. Stiffness commands discarded. Power cycle unit to restore stiffness change.")
            return

        self.groupSyncWriteStiffness.clearParam()
        self.dxl_write_ID_stiffness.clear()
        self.dxl_write_val_stiffness.clear()
        self.dxl_stiffnes_perm_ID.clear()
        self.dxl_stiffnes_perm_val.clear()

        for joint in msg.joints:
            id_ = self.getIDfromName(joint.name)
            if id_ != 'None':
                # Check security timing
                for i in self.security_list:
                    if id_ == i.id:
                        if i.time_last_set_stiffness is not None:
                            timing_check = time.time_ns() / 1000000000 - i.time_last_set_stiffness
                            if timing_check < 30:
                                self.get_logger().warn("Trying to set stiffness on joint %s too frequently : command discarded" % joint.name)
                                return
                        i.time_last_set_stiffness = time.time_ns() / 1000000000

                self.dxl_write_ID_stiffness.append(id_)
                self.dxl_stiffnes_perm_ID.append(id_)
                stiffness = joint.stiffness
                if 1 <= stiffness <= 9:
                    kp, ki, kd = self.getPIDStiffness(joint.stiffness, id_)
                    param = [DXL_LOBYTE(DXL_LOWORD(kd)), DXL_LOBYTE(DXL_LOWORD(ki)), DXL_LOBYTE(DXL_LOWORD(kp))]
                    self.dxl_write_val_stiffness.append(param)
                    perm_param = [DXL_LOBYTE(DXL_LOWORD(PERMISSION_ENABLE))]
                    self.dxl_stiffnes_perm_val.append(perm_param)

                    joint_pos = [index for (index, item) in enumerate(self.joints) if item.name == joint.name]
                    index = joint_pos[0]
                    self.joints[index].stiffness = joint.stiffness

                else:
                    self.get_logger().warn("Stiffness value out of range, try with a value between 1 and 9")
                    return
            else:
                self.get_logger().warn("Cannot set stiffness for joint %s, abort writing. Ignore this if you're using 2 hands on 2 different ports." % joint.name)
                return

        # publish new stiffness to the hardware
        self.get_logger().info("%s" % self.dxl_write_ID_stiffness)
        for index, id_ in enumerate(self.dxl_write_ID_stiffness):
            dxl_add_param_result = self.groupSyncWriteStiffness.addParam(id_, self.dxl_write_val_stiffness[index])
            if not dxl_add_param_result:
                self.get_logger().error("[ID:%d] groupSyncWriteStiffness addparam failed, abort wrtiting" % id_)
                return

        # set permissions to the hardware
        for index, id_ in enumerate(self.dxl_stiffnes_perm_ID):
            dxl_add_param_result = self.groupSyncWritePermission.addParam(id_, self.dxl_permission_param[index])
            if not dxl_add_param_result:
                self.get_logger().error("[ID:%d] groupSyncWritePermission addparam failed, abort wrtiting" % id_)

        self.WRITE_STIFFNESS = True

    def setSpeedPosCallback(self, msg):
        self.groupSyncWriteSpeedPos.clearParam()
        self.dxl_write_ID_pos_speed.clear()
        self.dxl_write_val_pos_speed.clear()

        for joint in msg.joints:
            id_ = self.getIDfromName(joint.name)
            if id_ != 'None':
                self.dxl_write_ID_pos_speed.append(id_)
                target_pos = joint.target_pos
                if joint.target_speed < 0:
                    for i in self.lone_joint_list:
                        if i.bus_id == id:
                            target_speed = i.target_speed
                            break
                else:
                    target_speed = joint.target_speed

                param = [DXL_LOBYTE(DXL_LOWORD(target_pos)), DXL_HIBYTE(DXL_LOWORD(target_pos)),
                         DXL_LOBYTE(DXL_LOWORD(target_speed)), DXL_HIBYTE(DXL_LOWORD(target_speed))]
                self.dxl_write_val_pos_speed.append(param)

            else:
                self.get_logger().warn("Cannot set target speed and position for joint %s, abort writing.  Ignore this if you're using 2 hands on 2 different ports." % joint.name)
                return

        for index, id_ in enumerate(self.dxl_write_ID_pos_speed):
            dxl_add_param_result = self.groupSyncWriteSpeedPos.addParam(id_, self.dxl_write_val_pos_speed[index])
            if not dxl_add_param_result:
                self.get_logger().error("[ID:%d] groupSyncWriteSpeedPos add param failed, abort wrtiting" % id_)
                return
        self.WRITE_SPEED_POS = True

    def mainLoopCallback(self):
        # Defining Cycle start and cycle end for each iteration to have clearance about the timing
        self.cycle_start = time.time_ns()/1000000
        self.cycle_end = self.cycle_start + self.timer_period * 1000
        # timer_ms1 = time.time_ns()/1000000 - self.cycle_start
        # self.get_logger().warn("First timer %f" % timer_ms1)

        dxl_comm_result = self.groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().warn("Communication not successfull in SyncRead : %s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # timer_ms2 = time.time_ns() / 1000000 - self.cycle_start
        # self.get_logger().warn("First read %f" % timer_ms2)

        for joint, id_ in zip(self.joints, self.dxl_read_ID_list):
            # Check if groupsyncread data of each id is available
            dxl_get_data_result = self.groupSyncRead.isAvailable(id_, ADDR_START_SYNC_READ, LEN_SYNC_READ)
            if not dxl_get_data_result:
                self.get_logger().error("[ID:%d] groupSyncRead getdata failed" % id_)

            joint.target_pos = self.groupSyncRead.getData(id_, ADDR_TARGET_POSITION, LEN_TARGET_POSITION)
            joint.target_speed = self.groupSyncRead.getData(id_, ADDR_TARGET_SPEED, LEN_TARGET_SPEED)
            joint.torque_limit = self.groupSyncRead.getData(id_, ADDR_TORQUE_LIMIT, LEN_TORQUE_LIMIT)
            joint.pres_pos = self.groupSyncRead.getData(id_, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            joint.pres_speed = self.groupSyncRead.getData(id_, ADDR_PRESENT_SPEED, LEN_PRESENT_SPEED)
            if not self.light_mode:
                joint.temperature = self.groupSyncRead.getData(id_, ADDR_TEMPERATURE, LEN_TEMPERATURE)
                joint.moving = self.groupSyncRead.getData(id_, ADDR_MOVING, LEN_MOVING)
                joint.hw_error_cond = self.groupSyncRead.getData(id_, ADDR_HW_ERROR_COND, LEN_HW_ERROR_COND)
                if joint.hw_error_cond != 0:
                    self.get_logger().warn("Joint %s currently has hardware error %d" % (joint.name, joint.hw_error_cond))
                joint.overload_filter_value = self.groupSyncRead.getData(id_, ADDR_OL_FILTER_VALUE, LEN_OL_FILTER_VALUE)
                joint.pres_current = self.groupSyncRead.getData(id_, ADDR_CURRENT, LEN_CURRENT)
                joint.setStressLevel()

        # timer_ms3 = time.time_ns() / 1000000 - self.cycle_start
        # self.get_logger().warn("loop read %f" % timer_ms3)

        self.fillJointMsg()

        # timer_ms4 = time.time_ns() / 1000000 - self.cycle_start
        # self.get_logger().warn("Fill msg %f" % timer_ms4)

        self.joint_publisher_.publish(self.all_joints_msg)

        if self.light_mode:
            if time.time_ns()/1000000 > self.cycle_end:
                exceeding_time_ms = time.time_ns()/1000000 - self.cycle_end
                self.get_logger().warn("TIME PERIOD EXCEEDED. Time exceeded by %d ms" % exceeding_time_ms)

        # Check the Flags to know if something is pending to sent
        if not self.WRITE_CLEAR_ERROR:                  # Highest Priority : Clear Error
            if not self.WRITE_SHUTDOWN_COND:            # Medium High Priority : Set Shutdown Conditions
                if not self.WRITE_STIFFNESS:            # Medium Priority : Set Stiffness
                    if self.WRITE_SPEED_POS:        # Lowest Priority : Set target speed and position
                        dxl_comm_result = self.groupSyncWriteSpeedPos.txPacket()
                        if dxl_comm_result != COMM_SUCCESS:
                            self.get_logger().warn("Communication problem in groupSyncWriteSpeedPos : %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                        self.WRITE_SPEED_POS = False
                else:
                    # Enable PID write permission on every joint needed
                    dxl_comm_result = self.groupSyncWritePermission.txPacket()
                    if dxl_comm_result != COMM_SUCCESS:
                        self.get_logger().warn("Communication error while syncWriting Stiffness Permission : %s" % self.packetHandler.getTxRxResult(dxl_comm_result))

                    # Sync write Stiffness
                    dxl_comm_result = self.groupSyncWriteStiffness.txPacket()
                    if dxl_comm_result != COMM_SUCCESS:
                        self.get_logger().warn("Communication error while syncWriting Stiffness : %s" % self.packetHandler.getTxRxResult(dxl_comm_result))

                    self.WRITE_STIFFNESS = False
                    self.get_logger().info("######################## SETTING STIFFNESS #########################")
            else:
                dxl_comm_result = self.groupSyncWriteClearError.txPacket()
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().warn("Communication problem in groupSyncWriteClearError : %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                self.get_logger().info("######################### CLEARING ERROR ##########################")
                self.WRITE_CLEAR_ERROR = False

        if time.time_ns() / 1000000 > self.cycle_end:
            if not self.light_mode:
                exceeding_time_ms = time.time_ns() / 1000000 - self.cycle_end
                self.get_logger().info("Time needed: %f" % (time.time_ns()/1000000 - self.cycle_start))
                self.get_logger().warn("TIME PERIOD EXCEEDED, You may want to try Light Mode. Time exceeded by %d ms" % exceeding_time_ms)

        else:
            if not self.light_mode:
                dxl_comm_result = self.groupSyncReadMainBoard.txRxPacket()
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().warn("Communication not successfull in groupSyncReadMainBoard : %s" % self.packetHandler.getTxRxResult(dxl_comm_result))

                for main_board, id_ in zip(self.main_boards, self.main_board_ID_list):
                    # Check if groupsyncread data of each id is available
                    dxl_get_data_result = self.groupSyncReadMainBoard.isAvailable(id_, ADDR_START_SYNC_READ_MAIN_BOARD, LEN_SYNC_READ_MAIN_BOARD)
                    if not dxl_get_data_result:
                        self.get_logger().error("[ID:%d] groupSyncReadMainBoard getdata failed" % id_)

                    main_board.palm_ir_sensor = self.groupSyncReadMainBoard.getData(id_, ADDR_PALM_IR_SENSOR, LEN_PALM_IR_SENSOR)
                    main_board.capacitive_sensor_1 = self.groupSyncReadMainBoard.getData(id_, ADDR_CAPACITIVE_SENSOR_1, LEN_CAPACITIVE_SENSOR)
                    main_board.capacitive_sensor_2 = self.groupSyncReadMainBoard.getData(id_, ADDR_CAPACITIVE_SENSOR_2, LEN_CAPACITIVE_SENSOR)

                self.fillMainBoardMessages()

                self.main_board_publisher_.publish(self.all_main_boards_msg)

                if time.time_ns() / 1000000 > self.cycle_end:
                    exceeding_time_ms = time.time_ns() / 1000000 - self.cycle_end
                    self.get_logger().info("Time needed: %d" % (self.cycle_end-self.cycle_end))
                    #self.get_logger().warn("TIME PERIOD EXCEEDED, You may want to try Light Mode. Time exceeded by %d ms" % exceeding_time_ms)

                else:
                    sleep_time_ms = self.cycle_end - time.time_ns()/1000000
                    sleep_time = (math.floor(sleep_time_ms)) * 0.001
                    if sleep_time > 0:
                        time.sleep(sleep_time)
            else:
                if time.time_ns() / 1000000 < self.cycle_end:
                    sleep_time_ms = self.cycle_end - time.time_ns() / 1000000  # msec
                    sleep_time = (math.floor(sleep_time_ms)) * 0.001  # sec
                    if sleep_time > 0:
                        time.sleep(sleep_time)


class Joint:
    def __init__(self, node, id_=-1, name=None, model_number=0):

        self.node = node
        self.id = id_
        self.name = name
        self.model_number = model_number
        self.firmware = 0
        self.bus_id = -1
        self.geartrain_model = None
        self.electronics_model = None

        self.kp = None
        self.ki = None
        self.kd = None
        self.overload_filter_threshold = None
        self.current_limit = None

        self.stiffness = 8
        self.stress_level = 0
        self.target_pos = 0
        self.target_speed = 0
        self.torque_limit = 0
        self.pres_pos = 0
        self.pres_speed = 0
        self.temperature = 0
        self.moving = 0
        self.hw_err_cond = 0
        self.overload_filter_value = 0
        self.pres_current = 0

    def printJointInfo(self):
        self.node.get_logger().info("Information about joint:" + str(self.name))
        self.node.get_logger().info("Model number %d" % self.model_number)
        self.node.get_logger().info("Firmware version %d" % self.firmware)
        self.node.get_logger().info("Bus ID %d" % self.bus_id)
        self.node.get_logger().info("Geartrain model %d" % self.geartrain_model)
        self.node.get_logger().info("Electronics model %d" % self.electronics_model)

    def setStressLevel(self):
        temp_stress_ = self.overload_filter_value / (self.overload_filter_threshold * 0.8) +\
                       self.pres_current / (self.current_limit * 0.9)
        self.stress_level = int(temp_stress_ / 2)

class MainBoard:
    def __init__(self, node, bus_id=-1, name='None', model_number=405):
        self.node = node
        self.id = bus_id
        self.name = name
        self.model_number = model_number
        self.palm_IR_sensor = 0
        self.capcitive_sensor_1 = 0
        self.capcitive_sensor_2 = 0

    def printMainBoardInfo(self):
        self.node.get_logger().info("Information about main board:" + str(self.name))
        self.node.get_logger().info("Model number %d" % self.model_number)
        self.node.get_logger().info("Bus ID %d" % self.id)


class Security:
    def __init__(self, id_=0):
        self.id = id_
        self.time_last_clear = None
        self.time_last_set_stiffness = None

def main(args=None):
    rclpy.init(args=args)

    hw_node = RH8DHWNode()

    rclpy.spin(hw_node)

    hw_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
