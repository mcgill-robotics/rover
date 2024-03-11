import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import rospy
from bms_feedback.msg import BMSData
import dalybms

class BMSNode():

    def __init__(self, device = "/dev/ttyACM0") -> None:
        

        rospy.init_node("bms_feedback", anonymous=False)

        self.bms_data_publisher = rospy.Publisher("bmsData", BMSData, queue_size=10)

        self.device = device
        self.bms_obj = dalybms.DalyBMS()
        self.bms_obj.connect(self.device)
        self.bmsData = BMSData()

        self.bms_errors = []
        self.connected = self.bms_obj.serial.is_open

        self.rate = rospy.Rate(1)

        self.run()

    def disconnect(self) -> None:
        self.bms_obj.disconnect()
        self.connected = False

    def connect(self) -> bool:
        self.bms_obj.connect(self.device)
        self.connected = self.bms_obj.serial.is_open
        return self.connected
    
    def isConnected(self) -> bool:
        return self.connected
    
    def run(self) -> None:
        while not rospy.is_shutdown():

            self.getBMSData()

            self.bms_data_publisher.publish(self.bmsData)

            self.rate.sleep()


    def getBMSData(self):
        cell_voltages = self.bms_obj.get_cell_voltages()
        soc_data = self.bms_obj.get_soc()
        mosfet_status = self.bms_obj.get_mosfet_status()
        self.bmsData.error_strings = self.bms_obj.get_errors()

        if cell_voltages:
            self.bmsData.cell_voltages[0] = cell_voltages["1"]
            self.bmsData.cell_voltages[1] = cell_voltages["2"]
            self.bmsData.cell_voltages[2] = cell_voltages["3"]
            self.bmsData.cell_voltages[3] = cell_voltages["4"]
            self.bmsData.cell_voltages[4] = cell_voltages["5"]
            self.bmsData.cell_voltages[5] = cell_voltages["6"]
            self.bmsData.cell_voltages[6] = cell_voltages["7"]
            self.bmsData.cell_voltages[7] = cell_voltages["8"]

        if soc_data:
            self.bmsData.discharging_mosfet = int(mosfet_status["discharging_mosfet"])
            self.bmsData.charging_mosfet = int(mosfet_status["charging_mosfet"])
            self.bmsData.capacity_ah = mosfet_status["capacity_ah"]

        if mosfet_status:
            self.bmsData.total_voltage = soc_data["total_voltage"]
            self.bmsData.current = soc_data["current"]
            self.bmsData.soc_percent = soc_data["soc_percent"]

        
if __name__ == "__main__":
    driver = BMSNode()
    rospy.spin()