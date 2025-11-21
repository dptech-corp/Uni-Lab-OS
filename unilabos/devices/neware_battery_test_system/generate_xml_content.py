def xml_811_Li_002(act_mass, Cap_mAh):
    """
    生成XML内容
    
    参数:
    act_mass: 正极质量(mg)
    Cap_mAh: 正极载量(mAh)
    devid: 设备号
    subdevid: 排号
    chlid: 通道号
    """
    xml_data = f"""<?xml version="1.0" encoding="utf-8"?>
<root>
  <config type="Step File" version="18" client_version="BTS Client 8.0.1.492(2025.01.23)(R3)" date="20250913110432" Guid="f92e53da-a6a6-41aa-a5b2-093701ffa657">
    <Head_Info>
      <Operate Value="66" />
      <Scale Value="1" />
      <Start_Step Value="1" Hide_Ctrl_Step="0" />
      <RateType Value="105" />
      <SCQ Value="{act_mass*1000}" />
      <SCQ_F Value="{act_mass/100000}" />
      <MultCap Value="{Cap_mAh*3600}" />
    </Head_Info>
    <Whole_Prt>
      <Protect>
        <Main>
          <Volt>
            <Upper Value="50000" />
            <Lower Value="0" />
          </Volt>
        </Main>
      </Protect>
      <Record>
        <Main>
          <Time Value="30000" />
        </Main>
      </Record>
    </Whole_Prt>
    <Step_Info Num="7">
      <Step1 Step_ID="1" Step_Type="4">
        <Limit>
          <Main>
            <Time Value="180000" />
          </Main>
        </Limit>
      </Step1>
      <Step2 Step_ID="2" Step_Type="7">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.02}" />
            <Rate Value="0.02" />
            <Volt Value="42000" />
            <Stop_Curr Value="{Cap_mAh*0.01}" />
            <Stop_Rate Value="0.01" />
          </Main>
        </Limit>
      </Step2>
      <Step3 Step_ID="3" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.02}" />
            <Rate Value="0.02" />
            <Stop_Volt Value="30000" />
          </Main>
        </Limit>
      </Step3>
      <Step4 Step_ID="4" Step_Type="7">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.5}" />
            <Rate Value="0.5" />
            <Volt Value="42000" />
            <Stop_Curr Value="{Cap_mAh*0.02}" />
            <Stop_Rate Value="0.02" />
          </Main>
        </Limit>
      </Step4>
      <Step5 Step_ID="5" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.5}" />
            <Rate Value="0.5" />
            <Stop_Volt Value="30000" />
          </Main>
        </Limit>
      </Step5>
      <Step6 Step_ID="6" Step_Type="5">
        <Limit>
          <Other>
            <Start_Step Value="4" />
            <Cycle_Count Value="3" />
          </Other>
        </Limit>
      </Step6>
      <Step7 Step_ID="7" Step_Type="6">
      </Step7>
    </Step_Info>
    <SMBUS>
      <SMBUS_Info Num="0" AdjacentInterval="0" />
    </SMBUS>
  </config>
</root>
    """
    return xml_data

def xml_811_Li_005(act_mass, Cap_mAh):
    """
    生成XML内容
    
    参数:
    act_mass: 正极质量(mg)
    Cap_mAh: 正极载量(mAh)
    devid: 设备号
    subdevid: 排号
    chlid: 通道号
    """
    xml_data = f"""<?xml version="1.0" encoding="utf-8"?>
<root>
  <config type="Step File" version="18" client_version="BTS Client 8.0.1.492(2025.01.23)(R3)" date="20250913110432" Guid="f92e53da-a6a6-41aa-a5b2-093701ffa657">
    <Head_Info>
      <Operate Value="66" />
      <Scale Value="1" />
      <Start_Step Value="1" Hide_Ctrl_Step="0" />
      <RateType Value="105" />
      <SCQ Value="{act_mass*1000}" />
      <SCQ_F Value="{act_mass/100000}" />
      <MultCap Value="{Cap_mAh*3600}" />
    </Head_Info>
    <Whole_Prt>
      <Protect>
        <Main>
          <Volt>
            <Upper Value="50000" />
            <Lower Value="0" />
          </Volt>
        </Main>
      </Protect>
      <Record>
        <Main>
          <Time Value="30000" />
        </Main>
      </Record>
    </Whole_Prt>
    <Step_Info Num="7">
      <Step1 Step_ID="1" Step_Type="4">
        <Limit>
          <Main>
            <Time Value="180000" />
          </Main>
        </Limit>
      </Step1>
      <Step2 Step_ID="2" Step_Type="7">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.05}" />
            <Rate Value="0.05" />
            <Volt Value="42000" />
            <Stop_Curr Value="{Cap_mAh*0.02}" />
            <Stop_Rate Value="0.02" />
          </Main>
        </Limit>
      </Step2>
      <Step3 Step_ID="3" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.05}" />
            <Rate Value="0.05" />
            <Stop_Volt Value="30000" />
          </Main>
        </Limit>
      </Step3>
      <Step4 Step_ID="4" Step_Type="7">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.5}" />
            <Rate Value="0.5" />
            <Volt Value="42000" />
            <Stop_Curr Value="{Cap_mAh*0.02}" />
            <Stop_Rate Value="0.02" />
          </Main>
        </Limit>
      </Step4>
      <Step5 Step_ID="5" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.5}" />
            <Rate Value="0.5" />
            <Stop_Volt Value="30000" />
          </Main>
        </Limit>
      </Step5>
      <Step6 Step_ID="6" Step_Type="5">
        <Limit>
          <Other>
            <Start_Step Value="4" />
            <Cycle_Count Value="3" />
          </Other>
        </Limit>
      </Step6>
      <Step7 Step_ID="7" Step_Type="6">
      </Step7>
    </Step_Info>
    <SMBUS>
      <SMBUS_Info Num="0" AdjacentInterval="0" />
    </SMBUS>
  </config>
</root>
    """
    return xml_data

def xml_LFP_Li(act_mass, Cap_mAh):
    """
    生成XML内容
    
    参数:
    act_mass: 正极质量(mg)
    Cap_mAh: 正极载量(mAh)
    devid: 设备号
    subdevid: 排号
    chlid: 通道号
    """
    xml_data = f"""<?xml version="1.0" encoding="utf-8"?>
<root>
  <config type="Step File" version="18" client_version="BTS Client 8.0.1.492(2025.01.23)(R3)" date="20250913110344" Guid="2d63c443-2216-4b66-84de-f42e8824605a">
    <Head_Info>
      <Operate Value="66" />
      <Scale Value="1" />
      <Start_Step Value="1" Hide_Ctrl_Step="0" />
      <RateType Value="105" />
      <SCQ Value="{act_mass*1000}" />
      <SCQ_F Value="{act_mass/100000}" />
      <MultCap Value="{Cap_mAh*3600}" />
    </Head_Info>
    <Whole_Prt>
      <Protect>
        <Main>
          <Volt>
            <Upper Value="50000" />
            <Lower Value="0" />
          </Volt>
        </Main>
      </Protect>
      <Record>
        <Main>
          <Time Value="30000" />
        </Main>
      </Record>
    </Whole_Prt>
    <Step_Info Num="7">
      <Step1 Step_ID="1" Step_Type="4">
        <Limit>
          <Main>
            <Time Value="180000" />
          </Main>
        </Limit>
      </Step1>
      <Step2 Step_ID="2" Step_Type="7">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.05}" />
            <Rate Value="0.05" />
            <Volt Value="40000" />
            <Stop_Curr Value="{Cap_mAh*0.02}" />
            <Stop_Rate Value="0.02" />
          </Main>
        </Limit>
      </Step2>
      <Step3 Step_ID="3" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.05}" />
            <Rate Value="0.05" />
            <Stop_Volt Value="28000" />
          </Main>
        </Limit>
      </Step3>
      <Step4 Step_ID="4" Step_Type="7">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.5}" />
            <Rate Value="0.5" />
            <Volt Value="40000" />
            <Stop_Curr Value="{Cap_mAh*0.02}" />
            <Stop_Rate Value="0.02" />
          </Main>
        </Limit>
      </Step4>
      <Step5 Step_ID="5" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.5}" />
            <Rate Value="0.5" />
            <Stop_Volt Value="28000" />
          </Main>
        </Limit>
      </Step5>
      <Step6 Step_ID="6" Step_Type="5">
        <Limit>
          <Other>
            <Start_Step Value="4" />
            <Cycle_Count Value="3" />
          </Other>
        </Limit>
      </Step6>
      <Step7 Step_ID="7" Step_Type="6">
      </Step7>
    </Step_Info>
    <SMBUS>
      <SMBUS_Info Num="0" AdjacentInterval="0" />
    </SMBUS>
  </config>
</root>
    """
    return xml_data

def xml_LFP_Gr(act_mass, Cap_mAh):
    """
    生成XML内容
    
    参数:
    act_mass: 正极质量(mg)
    Cap_mAh: 正极载量(mAh)
    devid: 设备号
    subdevid: 排号
    chlid: 通道号
    """
    xml_data = f"""<?xml version="1.0" encoding="utf-8"?>
<root>
  <config type="Step File" version="18" client_version="BTS Client 8.0.1.492(2025.01.23)(R3)" date="20250913110344" Guid="2d63c443-2216-4b66-84de-f42e8824605a">
    <Head_Info>
      <Operate Value="66" />
      <Scale Value="1" />
      <Start_Step Value="1" Hide_Ctrl_Step="0" />
      <RateType Value="105" />
      <SCQ Value="{act_mass*1000}" />
      <SCQ_F Value="{act_mass/100000}" />
      <MultCap Value="{Cap_mAh*3600}" />
    </Head_Info>
    <Whole_Prt>
      <Protect>
        <Main>
          <Volt>
            <Upper Value="50000" />
            <Lower Value="0" />
          </Volt>
        </Main>
      </Protect>
      <Record>
        <Main>
          <Time Value="30000" />
        </Main>
      </Record>
    </Whole_Prt>
    <Step_Info Num="7">
      <Step1 Step_ID="1" Step_Type="4">
        <Limit>
          <Main>
            <Time Value="180000" />
          </Main>
        </Limit>
      </Step1>
      <Step2 Step_ID="2" Step_Type="7">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.05}" />
            <Rate Value="0.05" />
            <Volt Value="36000" />
            <Stop_Curr Value="{Cap_mAh*0.02}" />
            <Stop_Rate Value="0.02" />
          </Main>
        </Limit>
      </Step2>
      <Step3 Step_ID="3" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.05}" />
            <Rate Value="0.05" />
            <Stop_Volt Value="28000" />
          </Main>
        </Limit>
      </Step3>
      <Step4 Step_ID="4" Step_Type="7">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.5}" />
            <Rate Value="0.5" />
            <Volt Value="36000" />
            <Stop_Curr Value="{Cap_mAh*0.02}" />
            <Stop_Rate Value="0.02" />
          </Main>
        </Limit>
      </Step4>
      <Step5 Step_ID="5" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.5}" />
            <Rate Value="0.5" />
            <Stop_Volt Value="28000" />
          </Main>
        </Limit>
      </Step5>
      <Step6 Step_ID="6" Step_Type="5">
        <Limit>
          <Other>
            <Start_Step Value="4" />
            <Cycle_Count Value="3" />
          </Other>
        </Limit>
      </Step6>
      <Step7 Step_ID="7" Step_Type="6">
      </Step7>
    </Step_Info>
    <SMBUS>
      <SMBUS_Info Num="0" AdjacentInterval="0" />
    </SMBUS>
  </config>
</root>
    """
    return xml_data

def xml_Gr_Li(act_mass, Cap_mAh):
    """
    生成XML内容
    
    参数:
    act_mass: 正极质量(mg)
    Cap_mAh: 正极载量(mAh)
    devid: 设备号
    subdevid: 排号
    chlid: 通道号
    """
    xml_data = f"""<?xml version="1.0" encoding="utf-8"?>
<root>
  <config type="Step File" version="18" client_version="BTS Client 8.0.1.492(2025.01.23)(R3)" date="20250917205515" Guid="547058a3-855f-4a13-bcf3-e59e489ca1e5">
    <Head_Info>
      <Operate Value="66" />
      <Scale Value="1" />
      <Start_Step Value="1" Hide_Ctrl_Step="0" />
      <RateType Value="105" />
      <SCQ Value="{act_mass*1000}" />
      <SCQ_F Value="{act_mass/100000}" />
      <MultCap Value="{Cap_mAh*3600}" />
    </Head_Info>
    <Whole_Prt>
      <Protect>
        <Main>
          <Volt>
            <Upper Value="50000" />
            <Lower Value="0" />
          </Volt>
        </Main>
      </Protect>
      <Record>
        <Main>
          <Time Value="30000" />
        </Main>
      </Record>
    </Whole_Prt>
    <Step_Info Num="7">
      <Step1 Step_ID="1" Step_Type="4">
        <Limit>
          <Main>
            <Time Value="120000" />
          </Main>
        </Limit>
      </Step1>
      <Step2 Step_ID="2" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.05}" />
            <Rate Value="0.05" />
            <Stop_Volt Value="500" />
          </Main>
        </Limit>
      </Step2>
      <Step3 Step_ID="3" Step_Type="7">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.05}" />
            <Rate Value="0.05" />
            <Volt Value="18000" />
            <Stop_Curr Value="{Cap_mAh*0.01}" />
            <Stop_Rate Value="0.01" />
          </Main>
        </Limit>
      </Step3>
      <Step4 Step_ID="4" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.2}" />
            <Rate Value="0.2" />
            <Stop_Volt Value="500" />
          </Main>
        </Limit>
      </Step4>
      <Step5 Step_ID="5" Step_Type="7">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.2}" />
            <Rate Value="0.2" />
            <Volt Value="18000" />
            <Stop_Curr Value="0.01" />
            <Stop_Rate Value="0.01" />
          </Main>
        </Limit>
      </Step5>
      <Step6 Step_ID="6" Step_Type="5">
        <Limit>
          <Other>
            <Start_Step Value="4" />
            <Cycle_Count Value="3" />
          </Other>
        </Limit>
      </Step6>
      <Step7 Step_ID="7" Step_Type="6">
      </Step7>
    </Step_Info>
    <SMBUS>
      <SMBUS_Info Num="0" AdjacentInterval="0" />
    </SMBUS>
  </config>
</root>
    """
    return xml_data

def xml_LB6(act_mass, Cap_mAh):
    """
    生成XML内容
    
    参数:
    act_mass: 正极质量(mg)
    Cap_mAh: 正极载量(mAh)
    devid: 设备号
    subdevid: 排号
    chlid: 通道号
    """
    xml_data = f"""<?xml version="1.0" encoding="utf-8"?>
<root>
  <config type="Step File" version="18" client_version="BTS Client 8.0.1.492(2025.01.23)(R3)" date="20250917190650" Guid="5bf80fd1-f2a6-47b2-b919-9b75726daae4">
    <Head_Info>
      <Operate Value="66" />
      <Scale Value="1" />
      <Start_Step Value="1" Hide_Ctrl_Step="0" />
      <RateType Value="105" />
      <SCQ Value="{act_mass*1000}" />
      <SCQ_F Value="{act_mass/100000}" />
      <MultCap Value="{Cap_mAh*3600}" />
    </Head_Info>
    <Whole_Prt>
      <Record>
        <Main>
          <Time Value="30000" />
        </Main>
      </Record>
    </Whole_Prt>
    <Step_Info Num="5">
      <Step1 Step_ID="1" Step_Type="4">
        <Record>
          <Main>
            <Time Value="30000" />
          </Main>
        </Record>
        <Limit>
          <Main>
            <Time Value="18000000" />
          </Main>
        </Limit>
        <Protect>
          <Main>
            <Volt>
              <Upper Value="50000" />
              <Lower Value="-50000" />
            </Volt>
          </Main>
        </Protect>
      </Step1>
      <Step2 Step_ID="2" Step_Type="1">
        <Record>
          <Main>
            <Time Value="30000" />
          </Main>
        </Record>
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.1}" />
            <Rate Value="0.1" />
            <Stop_Volt Value="44000" />
          </Main>
        </Limit>
        <Protect>
          <Main>
            <Volt>
              <Upper Value="50000" />
              <Lower Value="-50000" />
            </Volt>
          </Main>
        </Protect>
      </Step2>
      <Step3 Step_ID="3" Step_Type="3">
        <Record>
          <Main>
            <Time Value="30000" />
          </Main>
        </Record>
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.1}" />
            <Rate Value="0.1" />
            <Volt Value="44000" />
            <Time Value="3600000" />
            <Stop_Curr Value="{Cap_mAh*0.01}" />
            <Stop_Rate Value="0.01" />
          </Main>
        </Limit>
        <Protect>
          <Main>
            <Volt>
              <Upper Value="50000" />
              <Lower Value="-50000" />
            </Volt>
          </Main>
        </Protect>
      </Step3>
      <Step4 Step_ID="4" Step_Type="2">
        <Record>
          <Main>
            <Time Value="30000" />
          </Main>
        </Record>
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.1}" />
            <Rate Value="0.1" />
            <Stop_Volt Value="28000" />
          </Main>
        </Limit>
        <Protect>
          <Main>
            <Volt>
              <Upper Value="50000" />
              <Lower Value="-50000" />
            </Volt>
          </Main>
        </Protect>
      </Step4>
      <Step5 Step_ID="5" Step_Type="6">
      </Step5>
    </Step_Info>
    <SMBUS>
      <SMBUS_Info Num="0" AdjacentInterval="0" />
    </SMBUS>
  </config>
</root>
    """
    return xml_data


def xml_SiGr_Li_Step(act_mass, Cap_mAh):
    """
    生成XML内容
    
    参数:
    act_mass: 正极质量(mg)
    Cap_mAh: 正极载量(mAh)
    devid: 设备号
    subdevid: 排号
    chlid: 通道号
    """
    xml_data= f"""<?xml version="1.0" encoding="utf-8"?>
<root>
  <config type="Step File" version="18" client_version="BTS Client 8.0.1.492(2025.01.23)(R3)" date="20251025115844" Guid="67dfeafb-67f8-4354-8c02-351e66d927a7">
    <Head_Info>
      <Operate Value="66" />
      <Scale Value="1" />
      <Start_Step Value="1" Hide_Ctrl_Step="0" />
      <RateType Value="105" />
      <SCQ Value="{act_mass*1000}" />
      <SCQ_F Value="{act_mass/100000}" />
      <MultCap Value="{Cap_mAh*3600}" />
    </Head_Info>
    <Whole_Prt>
      <Protect>
        <Main>
          <Volt>
            <Upper Value="50000" />
            <Lower Value="0" />
          </Volt>
        </Main>
      </Protect>
      <Record>
        <Main>
          <Time Value="30000" />
        </Main>
      </Record>
    </Whole_Prt>
    <Step_Info Num="18">
      <Step1 Step_ID="1" Step_Type="4">
        <Limit>
          <Main>
            <Time Value="21600000" />
          </Main>
        </Limit>
      </Step1>
      <Step2 Step_ID="2" Step_Type="2">
        <Limit>
          <Main>
           <Curr Value="{Cap_mAh*0.1}" />
            <Rate Value="0.1" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step2>
      <Step3 Step_ID="3" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.09}" />
            <Rate Value="0.09" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step3>
      <Step4 Step_ID="4" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.08}" />
            <Rate Value="0.08" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step4>
      <Step5 Step_ID="5" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.07}" />
            <Rate Value="0.07" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step5>
      <Step6 Step_ID="6" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.06}" />
            <Rate Value="0.06" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step6>
      <Step7 Step_ID="7" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.05}" />
            <Rate Value="0.05" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step7>
      <Step8 Step_ID="8" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.04}" />
            <Rate Value="0.04" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step8>
      <Step9 Step_ID="9" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.03}" />
            <Rate Value="0.03" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step9>
      <Step10 Step_ID="10" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.02}" />
            <Rate Value="0.02" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step10>
      <Step11 Step_ID="11" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.01}" />
            <Rate Value="0.01" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step11>
      <Step12 Step_ID="12" Step_Type="4">
        <Limit>
          <Main>
            <Time Value="180000" />
          </Main>
        </Limit>
      </Step12>
      <Step13 Step_ID="13" Step_Type="1">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.1}" />
            <Rate Value="0.1" />
            <Stop_Volt Value="20000" />
          </Main>
        </Limit>
      </Step13>
      <Step14 Step_ID="14" Step_Type="4">
        <Limit>
          <Main>
            <Time Value="180000" />
          </Main>
        </Limit>
      </Step14>
      <Step15 Step_ID="15" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*1}" />
            <Rate Value="1" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step15>
      <Step16 Step_ID="16" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.9}" />
            <Rate Value="0.9" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step16>
      <Step17 Step_ID="17" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.8}" />
            <Rate Value="0.8" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step17>
      <Step18 Step_ID="18" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.7}" />
            <Rate Value="0.7" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step18>
      <Step19 Step_ID="19" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.6}" />
            <Rate Value="0.6" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step19>
      <Step20 Step_ID="20" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.5}" />
            <Rate Value="0.5" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step20>
      <Step21 Step_ID="21" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.4}" />
            <Rate Value="0.4" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step21>
      <Step22 Step_ID="22" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.3}" />
            <Rate Value="0.3" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step22>
      <Step23 Step_ID="23" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.2}" />
            <Rate Value="0.2" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step23>
      <Step24 Step_ID="24" Step_Type="2">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.1}" />
            <Rate Value="0.1" />
            <Stop_Volt Value="50" />
          </Main>
        </Limit>
      </Step24>
      <Step25 Step_ID="25" Step_Type="4">
        <Limit>
          <Main>
            <Time Value="180000" />
          </Main>
        </Limit>
      </Step25>
      <Step26 Step_ID="26" Step_Type="1">
        <Limit>
          <Main>
            <Curr Value="{Cap_mAh*0.5}" />
            <Rate Value="0.5" />
            <Stop_Volt Value="20000" />
          </Main>
        </Limit>
      </Step26>
      <Step27 Step_ID="27" Step_Type="4">
        <Limit>
          <Main>
            <Time Value="180000" />
          </Main>
        </Limit>
      </Step27>
      <Step28 Step_ID="28" Step_Type="5">
        <Limit>
          <Other>
            <Start_Step Value="15" />
            <Cycle_Count Value="3" />
          </Other>
        </Limit>
      </Step28>
      <Step29 Step_ID="29" Step_Type="6">
      </Step29>
    </Step_Info>
    <SMBUS>
      <SMBUS_Info Num="0" AdjacentInterval="0" />
    </SMBUS>
  </config>
</root>
  """
    return xml_data

def xml_811_SiGr(act_mass, Cap_mAh):
    """
    生成XML内容
    
    参数:
    act_mass: 正极质量(mg)
    Cap_mAh: 正极载量(mAh)
    devid: 设备号
    subdevid: 排号
    chlid: 通道号
    """
    xml_data= f"""<?xml version="1.0" encoding="utf-8"?>
<root>
  <config type="Step File" version="18" client_version="BTS Client 8.0.1.492(2025.01.23)(R3)" date="20251029205249" Guid="515dd9bb-0264-4ae2-9486-01d319537c6b">
    <Head_Info>
      <Operate Value="66" />
      <Scale Value="1" />
      <Start_Step Value="1" Hide_Ctrl_Step="0" />
      <RateType Value="105" />
      <SCQ Value="{act_mass*1000}" />
      <SCQ_F Value="{act_mass/100000}" />
      <MultCap Value="{Cap_mAh*3600}" />
    </Head_Info>
    <Whole_Prt>
      <Protect>
        <Main>
          <Volt>
            <Upper Value="50000" />
            <Lower Value="-20000" />
          </Volt>
        </Main>
      </Protect>
      <Record>
        <Main>
          <Time Value="30000" />
        </Main>
      </Record>
    </Whole_Prt>
    <Step_Info Num="13">
      <Step1 Step_ID="1" Step_Type="4">
        <Limit>
          <Main>
            <Time Value="18000000" />
          </Main>
        </Limit>
      </Step1>
      <Step2 Step_ID="2" Step_Type="1">
        <Limit>
          <Main>
           <Curr Value="{Cap_mAh*0.05}" />
            <Rate Value="0.05" />
            <Time Value="7200000" />
          </Main>
        </Limit>
      </Step2>
      <Step3 Step_ID="3" Step_Type="1">
        <Limit>
          <Main>
           <Curr Value="{Cap_mAh*0.1}" />
            <Rate Value="0.1" />
            <Time Value="3600000" />
          </Main>
        </Limit>
      </Step3>
      <Step4 Step_ID="4" Step_Type="1">
        <Limit>
          <Main>
           <Curr Value="{Cap_mAh*0.2}" />
            <Rate Value="0.2" />
            <Time Value="9000000" />
            <Stop_Volt Value="42000" />
          </Main>
        </Limit>
      </Step4>
      <Step5 Step_ID="5" Step_Type="2">
        <Limit>
          <Main>
           <Curr Value="{Cap_mAh*0.05}" />
            <Rate Value="0.05" />
            <Time Value="7200000" />
          </Main>
        </Limit>
      </Step5>
      <Step6 Step_ID="6" Step_Type="2">
        <Limit>
          <Main>
           <Curr Value="{Cap_mAh*0.1}" />
            <Rate Value="0.1" />
            <Time Value="3600000" />
          </Main>
        </Limit>
      </Step6>
      <Step7 Step_ID="7" Step_Type="2">
        <Limit>
          <Main>
           <Curr Value="{Cap_mAh*0.2}" />
            <Rate Value="0.2" />
            <Time Value="9000000" />
            <Stop_Volt Value="25000" />
          </Main>
        </Limit>
      </Step7>
      <Step8 Step_ID="8" Step_Type="1">
        <Limit>
          <Main>
           <Curr Value="{Cap_mAh*0.33}" />
            <Rate Value="0.33" />
            <Stop_Volt Value="42000" />
          </Main>
        </Limit>
      </Step8>
      <Step9 Step_ID="9" Step_Type="4">
        <Limit>
          <Main>
            <Time Value="1800000" />
          </Main>
        </Limit>
      </Step9>
      <Step10 Step_ID="10" Step_Type="2">
        <Limit>
          <Main>
           <Curr Value="{Cap_mAh*0.33}" />
            <Rate Value="0.33" />
            <Stop_Volt Value="25000" />
          </Main>
        </Limit>
      </Step10>
      <Step11 Step_ID="11" Step_Type="4">
        <Limit>
          <Main>
            <Time Value="1800000" />
          </Main>
        </Limit>
      </Step11>
      <Step12 Step_ID="12" Step_Type="5">
        <Limit>
          <Other>
            <Start_Step Value="8" />
            <Cycle_Count Value="20" />
          </Other>
        </Limit>
      </Step12>
      <Step13 Step_ID="13" Step_Type="6">
      </Step13>
    </Step_Info>
    <SMBUS>
      <SMBUS_Info Num="0" AdjacentInterval="0" />
    </SMBUS>
  </config>
</root>
  """
    return xml_data