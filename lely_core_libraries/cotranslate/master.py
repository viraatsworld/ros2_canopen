from yaml import Node, load
from ament_index_python import get_package_share_path
from pathlib import Path
from pprint import pprint


class Master:
    def __init__(self, root: Node, lifecycle: bool = False, master_id: int = 1):
        self.lifecycle = lifecycle
        self.id = master_id
        if "sync" in root:
            if "interval_ms" in root["sync"]:
                self.sync_period = root["sync"]["interval_ms"]
            if "overflow" in root["sync"]:
                self.sync_overflow = root["sync"]["overflow"]
        if "heartbeat" in root:
            if "rate" in root["heartbeat"]:
                self.heartbeat_period = int(1 / float(root["heartbeat"]["rate"]) * 1000)

    def get_master_dict(self):
        master = {}
        if self.sync_period is not None:
            master["sync_period"] = self.sync_period
        if self.sync_overflow is not None:
            master["sync_overflow"] = self.sync_overflow
        if self.heartbeat_period is not None:
            master["heartbeat_producer"] = self.heartbeat_period
        if self.lifecycle:
            master["driver"] = "ros2_canopen::LifecycleMasterDriver"
            master["package"] = "canopen_master_driver"
        master["node_id"] = self.id
        return master


class Slave:
    def __init__(self, root: Node, lifecycle: bool = False) -> None:
        self.id: int = None
        self.dcf_path: str = None
        self.dcf: str = None
        self.sdo: list = None
        if "id" in root:
            self.id = root["id"]
        self.dcf_path = ""
        if "eds_package" in root:
            print("The eds needs to be stored in the config package, 'eds_package' is ignored.")
        if "eds_file" in root:
            self.dcf = Path(root["eds_file"]).name
        self.sdo = []
        if "dcf_overlay" in root:
            for index, item in root["dcf_overlay"].items():
                sdo = {}
                indexes = index.split("sub")
                sdo["index"] = indexes[0]
                sdo["sub_index"] = 0
                if len(indexes) == 2:
                    sdo["sub_index"] = indexes[1]
                sdo["value"] = item
                self.sdo.append(sdo)

        if "pos_to_device" in root:
            print("pos_to_device not supported, currently only factor.")
        if "pos_from_device" in root:
            print("pos_from_device not supported, currently only factor.")
        if "vel_to_device" in root:
            print("vel_to_device not supported, currently only factor.")
        if "vel_from_device" in root:
            print("vel_from_device not supported, currently only factor.")
        if "eff_to_device" in root:
            print("eff_to_device not supported, currently only factor.")
        if "eff_from_device" in root:
            print("eff_from_device not supported, currently only factor.")

    def get_slave_dict(self):
        slave = {}
        if self.id is not None:
            slave["node_id"] = self.id
        if self.dcf is not None:
            slave["dcf"] = self.dcf
        if len(self.sdo) > 0:
            slave["sdo"] = self.sdo
        slave["driver"] = "ros2_canopen::Cia402Driver"
        slave["package"] = "canopen_402_driver"
        slave["dcf_path"] = "@BUS_CONFIG_PATH@"
        return slave


if __name__ == "__main__":
    test = """
sync:
  interval_ms: 20
  overflow: 0
heartbeat:
  rate: 20
  msg: "77f#05"

defaults:
  eds_pkg: cob_hardware_config
  eds_file: "robots/common/Elmo.dcf"
  dcf_overlay:
    "6083": "1000000" # profile acceleration
    "6084": "1000000" # profile deceleration
    "60C5": "1000000" # maximum acceleration
    "60C6": "1000000" # maximum deceleration
    "6099sub1": "100000" # find switch (coarse)
    "6099sub2": "10000" # find home (fine)
    "1016sub1" : "0x7F0064" # heartbeat timeout of 100 ms for master at 127
  eff_from_device: "obj6078/1000.0*9.76" #CL[1]=9.76

drive_wheel: &drive_wheel
  dcf_overlay: # "ObjectID" : "ParameterValue" (both as strings)
    "6098": "35" # homing method
  vel_from_device: "p1 != p1 || obj2041 == t1 ? 0 : deg2rad(1000*norm(obj6064-p1,-36000000,36000000)/norm(obj2041-t1,0,2^32)), p1=p0, p0 = obj6064, t1 = t0, t0 = obj2041"
  eff_from_device: "obj6078/1000.0*14.14" #CL[1]=14.14

nodes:
  ##Wheel 1
  fl_caster_rotation_joint:
    id: 1
    dcf_overlay: # "ObjectID" : "ParameterValue" (both as strings)
      "6098": "19" # homing method
  fl_caster_r_wheel_joint:
    <<: *drive_wheel
    id: 2

  ##Wheel 2
  b_caster_rotation_joint:
    id: 3
    dcf_overlay: # "ObjectID" : "ParameterValue" (both as strings)
      "6098": "19" # homing method
  b_caster_r_wheel_joint:
    <<: *drive_wheel
    id: 4

  ##Wheel 3
  fr_caster_rotation_joint:
    id: 5
    dcf_overlay: # "ObjectID" : "ParameterValue" (both as strings)
      "6098": "19" # homing method
  fr_caster_r_wheel_joint:
    <<: *drive_wheel
    id: 6
    """

    node = load(test)

    master = Master(node, False, 127)
    try:
        defaults = Slave(node["defaults"], False)
    except:
        pass

    slaves = {}
    for name, item in node["nodes"].items():
        slaves[name] = Slave(item, False).get_slave_dict()

    config = {}
    config["master"] = master.get_master_dict()
    config["defaults"] = defaults.get_slave_dict()
    config["nodes"] = slaves
    pprint(config)
