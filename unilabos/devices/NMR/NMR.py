import os
import time
import pandas as pd
import json
import shutil
import sys
import time

# define class ATR30007 
# this class is used to control the ATR30007 Raman workstation

class NMR_400:
    def __init__(self):
        self.machine = None
        
    def start_NMR(self, user: str = "admin", holder: int = 1,
                    name: str = "Samplename1", expno: int = 1, 
                    solvent: str = "D2O", experiment: str = "PROTON16",
                    title: str = "test_1H", save_root: str = None,):
        #打开仪器
        content = f"""USER {user}
HOLDER {holder}
NAME {name}
EXPNO {expno}
SOLVENT {solvent}
EXPERIMENT {experiment}
TITLE {title}
END
"""
        # 生成文件名（当前日期）
        filename = datetime.now().strftime("%Y%m%d%H%M") + ".txt"
        
        # 创建输出目录（如果不存在）
        os.makedirs(save_root, exist_ok=True)
        
        # 写入文件
        filepath = os.path.join(save_root, filename)
        with open(filepath, "w") as f:
            f.write(content)
        
        print(f'文件{filename}保存到{save_root}')

    def NMR_cmd(self, command: str):
        print(f"接收到命令: {command}")
        # replace !=! to "
        command = command.replace("!=!", "\"")
        command = command.replace('\\', '\\\\')
        command = command.replace("True", "true").replace("False", "false")
        try:
            cmd_dict = json.loads(command)
            print(f"命令参数: {cmd_dict}")
            # 解析命令参数
            save_root = cmd_dict.get("save_root", r"D:\UniLab\results\250414")
            # FIXME: use EIS for test. Add parameter for other tests
            
            self.start_NMR(**cmd_dict) # , file_name=file_name, save_root=save_root
            print(f"实验完成，数据已保存到 {save_root}")
        except Exception as e:
            print(f"命令执行失败: {e}")
            raise f"error: {e}" 
        