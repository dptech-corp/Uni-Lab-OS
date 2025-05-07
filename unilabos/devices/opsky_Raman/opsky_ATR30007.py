import clr 
import os
import time
import pandas as pd
import json
import shutil
import sys
import time
from System import Environment

# define class ATR30007 
# this class is used to control the ATR30007 Raman workstation

class ATR30007:
    def __init__(self, dll_path: str = r'D:\Raman_RS'):
        self.dll_path = dll_path
        self.machine = None
        self._load_dll()

    def _load_dll(self):
        # 检查ECCore.dll是否存在
        eccore_dll_path = os.path.join(self.dll_path, 'ATRWrapper.dll')
        if not os.path.exists(eccore_dll_path):
            raise FileNotFoundError(f"ATRWrapper.dll不存在于路径 {eccore_dll_path}")

        # 将DLL路径添加到系统PATH的最前面
        os.environ["PATH"] = self.dll_path + os.pathsep + os.environ["PATH"]

        # 添加引用
        try:
            clr.AddReference(eccore_dll_path)
            from Optosky.Wrapper import ATRWrapper
            self.machine = ATRWrapper()
            print("成功添加ATRWrapper.dll引用，导入ATRWrapper类，并创建实例")
        except Exception as e:
            raise RuntimeError(f"添加DLL引用或导入ATRWrapper类时出错: {e}")
        

    def get_machineSn(self):
        machineSn = self.machine.GetSn()
        print(f"选择的工作站ID: {machineSn}")

    def start_Raman(self, IntegTime: int = 5000, LdPower: int = 200,
                    Ldwave: int = 1, CCDTemp: int = -5, 
                    file_name: str = 'test_raman', save_root: str = None):
        #打开仪器
        On_flag = self.machine.OpenDevice()
        print(f"On_flag: {On_flag}")
        #获取仪器SN
        wrapper_Sn = self.machine.GetSn()
        print(f"wrapper_Sn: {wrapper_Sn}")
        #设置当前设备的积分时间， 单位为毫秒
        Integ_flag = self.machine.SetIntegrationTime(IntegTime)
        print(f"Integ_flag:{Integ_flag}")
        #设置激光功率， 单位mW
        LdP_flag = self.machine.SetLdPower(LdPower,Ldwave)
        print(f"LdP_flag:{LdP_flag}")
        #设置 CCD 制冷温度
        SetC_flag = self.machine.SetCool(CCDTemp)
        print(f"SetC_flag:{SetC_flag}")
        #开始采集光谱
        Spect = self.machine.AcquireSpectrum()
        #开始采集光谱谱图数据转换
        Spect_data =  list(Spect.get_Data())
        Spect_suss_flag =  Spect.get_Success()
        print(f"Spect_suss_flag:{Spect_suss_flag}")
        #获取波数
        WaveNum = list(self.machine.GetWaveNum())
        #光谱数据基线校正
        Spect_bLC = list(self.machine.BaseLineCorrect(Spect_data))
        #对数据进行boxcar 平滑
        Spect_StB = list(self.machine.SmoothBoxcar(Spect_bLC, 10))
        #关闭仪器
        OFF_flag = wrapper.CloseDevice()
        print(f"OFF_flag: {OFF_flag}")

        data_to_dataframe = {
            "WaveNum": WaveNum,
            "Spect_data": Spect_data,
            "Spect_bLC": Spect_bLC,
            "Spect_StB": Spect_StB
        }
        # 将数据转换为DataFrame格式
        df = pd.DataFrame(data_to_dataframe)
        self.save_data(df, file_name, save_root)
        self.check_status()


    def save_data(self, data: pd.DataFrame, file_name: str, save_root: str):
        if not os.path.exists(save_root):
            os.makedirs(save_root)
        data.to_csv(os.path.join(save_root, f"{file_name}.csv"), index=False)
        print(f"数据已保存到 {save_root}")

    def opsky_cmd(self, command: str):
        print(f"接收到命令: {command}")
        # replace !=! to "
        command = command.replace("!=!", "\"")
        command = command.replace('\\', '\\\\')
        command = command.replace("True", "true").replace("False", "false")
        try:
            cmd_dict = json.loads(command)
            print(f"命令参数: {cmd_dict}")
            # 解析命令参数
            # file_name = cmd_dict.get("file_name", "test")
            save_root = cmd_dict.get("save_root", r"D:\UniLab\results\250414")
            # FIXME: use EIS for test. Add parameter for other tests
            
            self.start_Raman(**cmd_dict) # , file_name=file_name, save_root=save_root
            print(f"实验完成，数据已保存到 {save_root}")
        except Exception as e:
            print(f"命令执行失败: {e}")
            raise f"error: {e}" 
        