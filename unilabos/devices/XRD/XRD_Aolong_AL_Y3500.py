
### test Ao long AL-Y3500 XRD device #####
'''
注意： 复制前删除同名文件
      (1) 首先要在D盘创建Aolong Save文件夹。
	      在D盘Aolong Save文件夹里创建Hardware.txt（参数控制文件）,  即D:\Aolong Save\Hardware.txt。
        
      (2) 除了停止和开始命令，其它参数要在命令符后面输入一位空格字符（同一行不能输入多位空格）。
        所有命令符英文都要大写。
        在D盘Aolong Save文件夹里创建Jade文件夹，测量结果自动保存在D:\Aolong Save\Jade里面。

      (3) `远程控制.exe`的主界面要勾选远程控制，才能实现远程操作功能。

      
输入参数`Hardware.txt`解读：

命令符   参数
 
USER    用户名
SAMPLE  样品编号
TIME    采集时间
STEP  	步进角度
START  	起始角度
C_END   结束角度         
RUN     开始命令
STOP    停止命令
END


示例：
建议采用(1)的写法，XRD机器保持开启状态（参数RUN），启动`远程控制.exe`, 保持界面中的高压电源（30 kV）开启状态，

(1). 测量Aolong 513 样品。在20-50度范围内，以0.2秒速度采集0.02步进角度的样品数据！ 
  	
USER   Aolong
SAMPLE 513
TIME   0.2
STEP   0.02
START  20
C_END  50 
RUN
END


(2). 停止测量

USER   Aolong
SAMPLE 513
STOP
END

'''

class XRD_Aolong_AL_Y3500:

    def __init__(self, name:str, time:float, step_angle:float, start_angle:float, end_angle:float, status:str):
        self.name = name
        self.time = time
        self.step_angle = step_angle
        self.start_angle = start_angle
        self.end_angle = end_angle
        self.status = status
     

    def setup_run_XRD(self, name:str, time:float, step_angle:float, start_angle:float, end_angle:float, status:str):
        '''
        setup input parameter of `aolong XRD AL-Y3500`
        '''

        ### default input parameters ##
        parameter_path = "D:/Aolong Save/Hardware.txt"

        ret = "USER   Aolong"
        ret += "\n"

        ret += f"SAMPLE {name}"
        ret += "\n"

        ret += f"TIME {time}"
        ret += "\n"

        ret += f"STEP {step_angle}"
        ret += "\n"

        ret += f"START {start_angle}"
        ret += "\n"

        ret += f"C_END {end_angle}"
        ret += "\n"

        ret += f"{status}"
        ret += "\n"

        ret += "END"
        ret += "\n"

        fp = open(parameter_path, "w")
        fp.write(ret)
        fp.close()

        ## 文件`parameter_path = "D:/Aolong Save/Hardware.txt"` 一旦发生改动，仪器开启自动开始XRD测试
        ## 并将测试结果自动保存到 `default save path` 中 
        save_path = "D:/Aolong Save/Jade"


#if __name__ == "__main__":
#    setup_run_XRD("smc-20250515-test", 0.2, 0.02, 20, 40, "RUN" )


