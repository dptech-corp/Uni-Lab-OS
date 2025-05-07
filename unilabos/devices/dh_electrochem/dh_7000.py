import clr
import os
import time
import pandas as pd
import json
import shutil
import sys
import time
from System import Environment


# define class DH7000
# this class is used to control the DH7000 electrochemical workstation
class DH7000:
    def __init__(self, dll_path: str = r'D:\DH_release64'):
        self.dll_path = dll_path
        self.machine = None
        self._load_dll()
        self._initialize_machine()
        self._status = "PowerOff"   # otherwise it will be "Running", "Overload", "Idle"
        self._machine_id = None

    @property
    def status(self) -> str:
        return self._status

    @property
    def machine_id(self) -> int:
        return self._machine_id

    def _load_dll(self):
        # Check if ECCore.dll exists
        eccore_dll_path = os.path.join(self.dll_path, 'ECCore.dll')
        if not os.path.exists(eccore_dll_path):
            raise FileNotFoundError(f"ECCore.dll不存在于路径 {eccore_dll_path}")
        # Prepend DLL path to system PATH
        os.environ["PATH"] = self.dll_path + os.pathsep + os.environ["PATH"]

        # Add reference
        try:
            clr.AddReference(eccore_dll_path)
            from ECCore import ElecMachines
            self.machine = ElecMachines()
            print("成功添加ECCore.dll引用，导入ElecMachines类，并创建实例")
        except Exception as e:
            raise RuntimeError(f"添加DLL引用或导入ElecMachines类时出错: {e}")

    def _initialize_machine(self):
        interface_dir = os.path.join(self.dll_path, 'DHInterface')
        if not os.path.exists(interface_dir):
            raise FileNotFoundError(f"DHInterface文件夹不存在于路径 {interface_dir}")

        init_success = self.machine.Init(interface_dir)
        if not init_success:
            raise RuntimeError("工作站初始化失败")
        print("工作站初始化成功")
        self.check_status()

    def check_status(self):
        Flag_isexp = self.machine.IsExperimenting()
        if Flag_isexp:
            Flag_isover = self.machine.IsOverLoad()
            if Flag_isover:
                self._status = "Overload"
            else:
                self._status = "Running"
        else:
            self._status = "Idle"

    def set_current_machineid(self, machine_id: int):
        machineid_list = list(self.machine.GetMachineId())
        if machine_id not in machineid_list:
            raise ValueError(f"工作站ID {machine_id} 不在可用ID列表中: {machineid_list}")
        self.machine.SetCurrentMachineId(machine_id)
        self._machine_id = machine_id
        print(f"选择的工作站ID: {self._machine_id}")

    def start_eis(
        self,
        isVolEIS: bool = True,
        StartFreq: float = 1E5,
        EndFreq: float = 1E-1,
        Amplitude: float = 10,
        IntervalType: int = 1,
        PointCount: int = 10,
        Voltage: float = 0.0,
        VoltageVSType: float = 1.0,
        isVoltageRandAuto: int = 1,
        VoltageRand: str = "10",
        isCurrentRandAuto: int = 1,
        CurrentRand: str = "10",
        file_name: str = 'test_eis',
        save_root: str = r"D:\UniLab\results"
    ):
        """
        EIS test
        """
        self.machine.Start_EIS(
            isVolEIS,
            StartFreq,
            EndFreq,
            Amplitude,
            IntervalType,
            PointCount,
            Voltage,
            VoltageVSType,
            isVoltageRandAuto,
            VoltageRand,
            isCurrentRandAuto,
            CurrentRand
        )
        self._status = "Running"

        # Wait for experiment to finish
        while self._status == "Running":
            time.sleep(5)
            self.check_status()
            print(f"当前状态: {self._status}")

        # Retrieve data
        datatype = self.machine.GetResultDataType()
        data_result = self.machine.GetData(datatype)
        splitCount = 6

        timeData = list(self.machine.SplitData(data_result, splitCount, 0))
        ZreData = list(self.machine.SplitData(data_result, splitCount, 1))
        ZimData = list(self.machine.SplitData(data_result, splitCount, 2))
        ZData = list(self.machine.SplitData(data_result, splitCount, 3))
        FreqData = list(self.machine.SplitData(data_result, splitCount, 4))
        PhaseData = list(self.machine.SplitData(data_result, splitCount, 5))

        data_to_dataframe = {
            "TimeData": timeData,
            "ZreData": ZreData,
            "ZimData": ZimData,
            "ZData": ZData,
            "FreqData": FreqData,
            "PhaseData": PhaseData,
        }

        # Convert to DataFrame
        df = pd.DataFrame(data_to_dataframe)
        self.save_data(df, file_name, save_root)
        self.check_status()

    def start_lsv(
        self,
        InitialPotential: float = 0.0,
        InitialPotentialVSType: int = 1,
        FinallyPotential: float = 1.0,
        FinallyPotentialVSType: int = 1,
        ScanRate: float = 0.05,
        isVoltageRandAuto: int = 1,
        VoltageRand: str = "10",
        isCurrentRandAuto: int = 1,
        CurrentRand: str = "10",
        file_name: str = 'lsv_test',
        save_root: str = r"D:\UniLab\results"
    ):
        """
        LSV test
        """
        self.machine.Start_Linear_Scan_Voltammetry(
            InitialPotential,
            InitialPotentialVSType,
            FinallyPotential,
            FinallyPotentialVSType,
            ScanRate,
            isVoltageRandAuto,
            VoltageRand,
            isCurrentRandAuto,
            CurrentRand
        )
        self._status = "Running"

        # Wait for experiment to finish
        while self._status == "Running":
            time.sleep(5)
            self.check_status()
            print(f"当前状态: {self._status}")

        # Retrieve data
        datatype = self.machine.GetResultDataType()
        data_result = self.machine.GetData(datatype)
        splitCount = 3

        timeData = list(self.machine.SplitData(data_result, splitCount, 0))
        VolData = list(self.machine.SplitData(data_result, splitCount, 1))
        CurData = list(self.machine.SplitData(data_result, splitCount, 2))

        data_to_dataframe = {
            "TimeData": timeData,
            "VolData": VolData,
            "CurData": CurData,
        }
        # Convert to DataFrame
        df = pd.DataFrame(data_to_dataframe)
        self.save_data(df, file_name, save_root)
        self.check_status()

    def start_cv_single(
        self,
        InitialPotential: float = 0.0,
        InitialPotentialVSType: int = 0,
        TopPotential: float = 1.0,
        TopPotentialVSType: int = 0,
        FinallyPotential: float = 0.0,
        FinallyPotentialVSType: int = 0,
        ScanRate: float = 0.1,
        isVoltageRandAuto: int = 1,
        VoltageRand: str = "10",
        isCurrentRandAuto: int = 1,
        CurrentRand: str = "10",
        isVoltageFilterAuto: int = 1,
        VoltageFilter: str = "",
        isCurrentFilterAuto: int = 1,
        currentFilter: str = "",
        machineId: int = 0,
        delayTime: float = 0.0,
        file_name: str = 'cv_single_test',
        save_root: str = r"D:\UniLab\results"
    ):
        """
        Single cyclic voltammetry (Single CV)
        """

        # Call underlying DLL function
        self.machine.Start_Circle_Voltammetry_Single(
            InitialPotential,
            InitialPotentialVSType,
            TopPotential,
            TopPotentialVSType,
            FinallyPotential,
            FinallyPotentialVSType,
            ScanRate,
            isVoltageRandAuto,
            VoltageRand,
            isCurrentRandAuto,
            CurrentRand,
            isVoltageFilterAuto,
            VoltageFilter,
            isCurrentFilterAuto,
            currentFilter,
            machineId,
            delayTime
        )

        # Wait for experiment to finish
        self._status = "Running"
        while self._status == "Running":
            time.sleep(5)
            self.check_status()
            print(f"当前状态: {self._status}")

        # Retrieve and parse data
        datatype = self.machine.GetResultDataType()
        data_result = self.machine.GetData(datatype)
        # Same as LSV, usually three columns: Time, Voltage, Current
        splitCount = 3

        timeData = list(self.machine.SplitData(data_result, splitCount, 0))
        volData = list(self.machine.SplitData(data_result, splitCount, 1))
        curData = list(self.machine.SplitData(data_result, splitCount, 2))

        data_to_dataframe = {
            "TimeData": timeData,
            "VolData": volData,
            "CurData": curData,
        }
        df = pd.DataFrame(data_to_dataframe)

        # Save data
        self.save_data(df, file_name, save_root)
        self.check_status()    

    def start_cv_multi(
        self,
        IsUseInitialPotential: bool = True,
        InitialPotential: float = 0.0,
        InitialPotentialVSType: int = 0,
        TopPotential1: float = 1.0,
        TopPotential1VSType: int = 0,
        TopPotential2: float = -1.0,
        TopPotential2VSType: int = 0,
        IsUseFinallyPotential: bool = True,
        FinallyPotential: float = 0.0,
        FinallyPotentialVSType: int = 0,
        ScanRate: float = 0.1,
        cycleCount: int = 3,
        isVoltageRandAuto: int = 1,
        VoltageRand: str = "10",
        isCurrentRandAuto: int = 1,
        CurrentRand: str = "10",
        isVoltageFilterAuto: int = 1,
        VoltageFilter: str = "",
        isCurrentFilterAuto: int = 1,
        currentFilter: str = "",
        machineId: int = 0,
        delayTime: float = 0.0,
        file_name: str = "cv_multi_test",
        save_root: str = r"D:\UniLab\results",
    ):
        """
        Multiple cyclic voltammetry (Multiple CV).
        """
        # Call underlying DLL
        self.machine.Start_Circle_Voltammetry_Multi(
            IsUseInitialPotential,
            InitialPotential,
            InitialPotentialVSType,
            TopPotential1,
            TopPotential1VSType,
            TopPotential2,
            TopPotential2VSType,
            IsUseFinallyPotential,
            FinallyPotential,
            FinallyPotentialVSType,
            ScanRate,
            cycleCount,
            isVoltageRandAuto,
            VoltageRand,
            isCurrentRandAuto,
            CurrentRand,
            isVoltageFilterAuto,
            VoltageFilter,
            isCurrentFilterAuto,
            currentFilter,
            machineId,
            delayTime,
        )

        # Wait for experiment to finish
        self._status = "Running"
        while self._status == "Running":
            time.sleep(5)
            self.check_status()
            print(f"当前状态: {self._status}")

        # Fetch data and split
        datatype = self.machine.GetResultDataType()
        data_result = self.machine.GetData(datatype)

        splitCount = 3          # Assume still three columns: Time/Voltage/Current
        timeData = list(self.machine.SplitData(data_result, splitCount, 0))
        volData  = list(self.machine.SplitData(data_result, splitCount, 1))
        curData  = list(self.machine.SplitData(data_result, splitCount, 2))

        df = pd.DataFrame(
            {
                "TimeData": timeData,
                "VolData": volData,
                "CurData": curData,
            }
        )

        # Save
        self.save_data(df, file_name, save_root)
        self.check_status()

    def start_ca(
        self,
        timePerPoint: float = 0.1,
        continueTime: float = 100.0,
        InitialPotential: float = 0.0,
        InitialPotentialVSType: int = 0,
        isVoltageRandAuto: int = 1,
        VoltageRand: str = "10",
        isCurrentRandAuto: int = 1,
        CurrentRand: str = "10",
        isVoltageFilterAuto: int = 1,
        VoltageFilter: str = "",
        isCurrentFilterAuto: int = 1,
        currentFilter: str = "",
        machineId: int = 0,
        file_name: str = "ca_test",
        save_root: str = r"D:\UniLab\results",
    ):
        """
        Chrono‑Amperometry (CA) — constant potential
        """
        # Call DLL
        self.machine.Start_ChronoamperonetryParam(
            timePerPoint,
            continueTime,
            InitialPotential,
            InitialPotentialVSType,
            isVoltageRandAuto,
            VoltageRand,
            isCurrentRandAuto,
            CurrentRand,
            isVoltageFilterAuto,
            VoltageFilter,
            isCurrentFilterAuto,
            currentFilter,
            machineId,
        )

        # Wait for experiment to finish
        self._status = "Running"
        while self._status == "Running":
            time.sleep(5)
            self.check_status()
            print(f"当前状态: {self._status}")

        # Fetch and split data (commonly three columns: Time / Voltage / Current)
        datatype = self.machine.GetResultDataType()
        data_result = self.machine.GetData(datatype)

        splitCount = 3
        timeData = list(self.machine.SplitData(data_result, splitCount, 0))
        volData  = list(self.machine.SplitData(data_result, splitCount, 1))
        curData  = list(self.machine.SplitData(data_result, splitCount, 2))

        df = pd.DataFrame(
            {
                "TimeData": timeData,
                "VolData": volData,
                "CurData": curData,
            }
        )

        # Save
        self.save_data(df, file_name, save_root)
        self.check_status()

    def start_cp(
        self,
        timePerPoint: float = 0.1,
        continueTime: float = 100.0,
        current: float = 1.0,
        VoltageRand: str = "100",
        isCurrentRandAuto: int = 1,
        CurrentRand: str = "10",
        isVoltageFilterAuto: int = 1,
        VoltageFilter: str = "",
        isCurrentFilterAuto: int = 1,
        currentFilter: str = "",
        machineId: int = 0,
        file_name: str = "cp_test",
        save_root: str = r"D:\UniLab\results",
    ):
        """
        Chrono‑Potentiometry (CP) — constant current
        """
        # Call DLL
        self.machine.Start_ChronopotentiometryParam(
            timePerPoint,
            continueTime,
            current,
            VoltageRand,
            isCurrentRandAuto,
            CurrentRand,
            isVoltageFilterAuto,
            VoltageFilter,
            isCurrentFilterAuto,
            currentFilter,
            machineId,
        )

        # Wait for experiment to finish
        self._status = "Running"
        while self._status == "Running":
            time.sleep(5)
            self.check_status()
            print(f"当前状态: {self._status}")

        # Fetch and split data (Time / Voltage / Current)
        datatype = self.machine.GetResultDataType()
        data_result = self.machine.GetData(datatype)

        splitCount = 3
        timeData = list(self.machine.SplitData(data_result, splitCount, 0))
        volData  = list(self.machine.SplitData(data_result, splitCount, 1))
        curData  = list(self.machine.SplitData(data_result, splitCount, 2))

        df = pd.DataFrame(
            {
                "TimeData": timeData,
                "VolData": volData,
                "CurData": curData,
            }
        )

        # Save
        self.save_data(df, file_name, save_root)
        self.check_status()


    def stop_experiment(self):
        self.machine.StopExperiment()
        self.check_status()
        print("实验已停止")

    def save_data(self, data: pd.DataFrame, file_name: str, save_root: str):
        if not os.path.exists(save_root):
            os.makedirs(save_root)
        data.to_csv(os.path.join(save_root, f"{file_name}.csv"), index=False)
        print(f"数据已保存到 {save_root}")

    # === Core: a unified dh_cmd method that calls start_eis or start_lsv according to methods ===
    def dh_cmd(self, command: str):
        """
        Unified handler for different commands such as EIS / LSV / CV / CA.
        In the incoming command JSON, use the key "methods" to specify the measurement type: "eis", "lsv", etc.
        """
        self.success = False
        print(f"接收到命令: {command}")

        # Replace "!=!" with quotes and fix True/False and path separators
        command = command.replace("!=!", "\"")
        command = command.replace('\\', '\\\\')
        command = command.replace("True", "true").replace("False", "false")

        try:
            cmd_dict = json.loads(command)
            print(f"命令参数: {cmd_dict}")

            # Extract measurement method, default to "EIS" if not specified
            method = cmd_dict.pop("methods", "eis").lower()
            # Extract file save path (may not be provided)
            save_root = cmd_dict.get("save_root", r"D:\UniLab\results\test")

            if method in ("cv_single", "cvs"):
                # CV (cyclic voltammetry, single cycle)
                print("执行 EIS 测试...")
                self.start_cv_single(**cmd_dict)
            elif method in ("cv_multi", "cvm"):
                # CV (cyclic voltammetry, multiple cycles)  
                print("执行多循环伏安测试...")
                self.start_cv_multi(**cmd_dict)
            elif method == "lsv":
                # LSV
                print("执行 LSV 测试...")
                self.start_lsv(**cmd_dict)
            elif method == "ca":
                # CA
                self.start_ca(**cmd_dict)
            elif method == "cp":
                # CP
                self.start_cp(**cmd_dict)
            else:
                # Default to EIS                        
                print("执行 EIS 测试...")
                self.start_eis(**cmd_dict)
            

            print(f"实验完成，数据已保存到 {save_root}")
            self.success = True

        except Exception as e:
            print(f"命令执行失败: {e}")
            raise RuntimeError(f"error: {e}")