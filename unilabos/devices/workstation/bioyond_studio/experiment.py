"""
实验流程主程序
"""

import json
from unilabos.devices.workstation.bioyond_studio.reaction_station import BioyondReactionStation
from unilabos.devices.workstation.bioyond_studio.config import API_CONFIG, WORKFLOW_MAPPINGS, DECK_CONFIG, MATERIAL_TYPE_MAPPINGS


def run_experiment():
    """运行实验流程"""

    # 初始化Bioyond客户端
    config = {
        **API_CONFIG,
        "workflow_mappings": WORKFLOW_MAPPINGS,
        "material_type_mappings": MATERIAL_TYPE_MAPPINGS
    }

    # 创建BioyondReactionStation实例，传入deck配置
    Bioyond = BioyondReactionStation(
        config=config,
        deck=DECK_CONFIG
    )

    print("\n============= 多工作流参数测试（简化接口+材料缓存）=============")

    # 显示可用的材料名称（前20个）
    available_materials = Bioyond.hardware_interface.get_available_materials()
    print(f"可用材料名称（前20个）: {available_materials[:20]}")
    print(f"总共有 {len(available_materials)} 个材料可用\n")

    # 1. 反应器放入
    print("1. 添加反应器放入工作流，带参数...")
    Bioyond.reactor_taken_in(
        assign_material_name="BTDA-DD",
        cutoff="10000",
        temperature="-10"
    )

    # 2. 液体投料-烧杯 (第一个)
    print("2. 添加液体投料-烧杯，带参数...")
    Bioyond.liquid_feeding_beaker(
        volume="34768.7",
        assign_material_name="ODA",
        time="0",
        torque_variation="1",
        titration_type="1",
        temperature=-10
    )

    # 3. 液体投料-烧杯 (第二个)
    print("3. 添加液体投料-烧杯，带参数...")
    Bioyond.liquid_feeding_beaker(
        volume="34080.9",
        assign_material_name="MPDA",
        time="5",
        torque_variation="2",
        titration_type="1",
        temperature=0
    )

    # 4. 液体投料-小瓶非滴定
    print("4. 添加液体投料-小瓶非滴定，带参数...")
    Bioyond.liquid_feeding_vials_non_titration(
        volume_formula="639.5",
        assign_material_name="SIDA",
        titration_type="1",
        time="0",
        torque_variation="1",
        temperature=-10
    )

    # 5. 液体投料溶剂
    print("5. 添加液体投料溶剂，带参数...")
    Bioyond.liquid_feeding_solvents(
        assign_material_name="NMP",
        volume="19000",
        titration_type="1",
        time="5",
        torque_variation="2",
        temperature=-10
    )

    # 6-8. 固体进料小瓶 (三个)
    print("6. 添加固体进料小瓶，带参数...")
    Bioyond.solid_feeding_vials(
        material_id="3",
        time="180",
        torque_variation="2",
        assign_material_name="BTDA1",
        temperature=-10.00
    )

    print("7. 添加固体进料小瓶，带参数...")
    Bioyond.solid_feeding_vials(
        material_id="3",
        time="180",
        torque_variation="2",
        assign_material_name="BTDA2",
        temperature=25.00
    )

    print("8. 添加固体进料小瓶，带参数...")
    Bioyond.solid_feeding_vials(
        material_id="3",
        time="480",
        torque_variation="2",
        assign_material_name="BTDA3",
        temperature=25.00
    )

    # 液体投料滴定（第一个）
    print("9. 添加液体投料滴定，带参数...")  # ODPA
    Bioyond.liquid_feeding_titration(
        volume_formula="{{6-0-5}}+{{7-0-5}}+{{8-0-5}}",
        assign_material_name="BTDA-DD",
        titration_type="1",
        time="360",
        torque_variation="2",
        temperature="25.00"
    )

    # 液体投料滴定（第二个）
    print("10. 添加液体投料滴定，带参数...")  # ODPA
    Bioyond.liquid_feeding_titration(
        volume_formula="500",
        assign_material_name="BTDA-DD",
        titration_type="1",
        time="360",
        torque_variation="2",
        temperature="25.00"
    )

    # 液体投料滴定（第三个）
    print("11. 添加液体投料滴定，带参数...")  # ODPA
    Bioyond.liquid_feeding_titration(
        volume_formula="500",
        assign_material_name="BTDA-DD",
        titration_type="1",
        time="360",
        torque_variation="2",
        temperature="25.00"
    )

    print("12. 添加液体投料滴定，带参数...")  # ODPA
    Bioyond.liquid_feeding_titration(
        volume_formula="500",
        assign_material_name="BTDA-DD",
        titration_type="1",
        time="360",
        torque_variation="2",
        temperature="25.00"
    )

    print("13. 添加液体投料滴定，带参数...")  # ODPA
    Bioyond.liquid_feeding_titration(
        volume_formula="500",
        assign_material_name="BTDA-DD",
        titration_type="1",
        time="360",
        torque_variation="2",
        temperature="25.00"
    )

    print("14. 添加液体投料滴定，带参数...")  # ODPA
    Bioyond.liquid_feeding_titration(
        volume_formula="500",
        assign_material_name="BTDA-DD",
        titration_type="1",
        time="360",
        torque_variation="2",
        temperature="25.00"
    )

    print("15. 添加液体投料溶剂，带参数...")
    Bioyond.liquid_feeding_solvents(
        assign_material_name="PGME",
        volume="16894.6",
        titration_type="1",
        time="360",
        torque_variation="2",
        temperature=25.00
    )

    # 16. 反应器取出
    print("16. 添加反应器取出工作流...")
    Bioyond.reactor_taken_out()

    # 显示当前工作流序列
    sequence = Bioyond.get_workflow_sequence()
    print("\n当前工作流执行顺序:")
    print(sequence)

    # 执行process_and_execute_workflow，合并工作流并创建任务
    print("\n4. 执行process_and_execute_workflow...")

    result = Bioyond.process_and_execute_workflow(
        workflow_name="test3",
        task_name="实验3"
    )

    # 显示执行结果
    print("\n5. 执行结果:")
    if isinstance(result, str):
        try:
            result_dict = json.loads(result)
            if result_dict.get("success"):
                print("任务创建成功!")
                # print(f"- 工作流: {result_dict.get('workflow', {}).get('name')}")
                # print(f"- 工作流ID: {result_dict.get('workflow', {}).get('id')}")
                # print(f"- 任务结果: {result_dict.get('task')}")
            else:
                print(f"任务创建失败: {result_dict.get('error')}")
        except:
            print(f"结果解析失败: {result}")
    else:
        if result.get("success"):
            print("任务创建成功!")
            print(f"- 工作流: {result.get('workflow', {}).get('name')}")
            print(f"- 工作流ID: {result.get('workflow', {}).get('id')}")
            print(f"- 任务结果: {result.get('task')}")
        else:
            print(f"任务创建失败: {result.get('error')}")

    # 可选：启动调度器
    # Bioyond.scheduler_start()

    return Bioyond


# def prepare_materials(bioyond):
#     """准备实验材料（可选）"""

#     # 样品板材料数据定义
#     material_data_yp_1 = {
#         "typeId": "3a142339-80de-8f25-6093-1b1b1b6c322e",
#         "name": "样品板-1",
#         "unit": "个",
#         "quantity": 1,
#         "details": [
#             {
#                 "typeId": "3a14233a-84a3-088d-6676-7cb4acd57c64",
#                 "name": "BPDA-DD-1",
#                 "quantity": 1,
#                 "x": 1,
#                 "y": 1,
#                 "Parameters": "{\"molecular\": 1}"
#             },
#             {
#                 "typeId": "3a14233a-84a3-088d-6676-7cb4acd57c64",
#                 "name": "PEPA",
#                 "quantity": 1,
#                 "x": 1,
#                 "y": 2,
#                 "Parameters": "{\"molecular\": 1}"
#             },
#             {
#                 "typeId": "3a14233a-84a3-088d-6676-7cb4acd57c64",
#                 "name": "BPDA-DD-2",
#                 "quantity": 1,
#                 "x": 1,
#                 "y": 3,
#                 "Parameters": "{\"molecular\": 1}"
#             },
#             {
#                 "typeId": "3a14233a-84a3-088d-6676-7cb4acd57c64",
#                 "name": "BPDA-1",
#                 "quantity": 1,
#                 "x": 2,
#                 "y": 1,
#                 "Parameters": "{\"molecular\": 1}"
#             },
#             {
#                 "typeId": "3a14233a-84a3-088d-6676-7cb4acd57c64",
#                 "name": "PMDA",
#                 "quantity": 1,
#                 "x": 2,
#                 "y": 2,
#                 "Parameters": "{\"molecular\": 1}"
#             },
#             {
#                 "typeId": "3a14233a-84a3-088d-6676-7cb4acd57c64",
#                 "name": "BPDA-2",
#                 "quantity": 1,
#                 "x": 2,
#                 "y": 3,
#                 "Parameters": "{\"molecular\": 1}"
#             }
#         ],
#         "Parameters": "{}"
#     }

#     material_data_yp_2 = {
#         "typeId": "3a142339-80de-8f25-6093-1b1b1b6c322e",
#         "name": "样品板-2",
#         "unit": "个",
#         "quantity": 1,
#         "details": [
#             {
#                 "typeId": "3a14233a-84a3-088d-6676-7cb4acd57c64",
#                 "name": "BPDA-DD",
#                 "quantity": 1,
#                 "x": 1,
#                 "y": 1,
#                 "Parameters": "{\"molecular\": 1}"
#             },
#             {
#                 "typeId": "3a14233a-84a3-088d-6676-7cb4acd57c64",
#                 "name": "SIDA",
#                 "quantity": 1,
#                 "x": 1,
#                 "y": 2,
#                 "Parameters": "{\"molecular\": 1}"
#             },
#             {
#                 "typeId": "3a14233a-84a3-088d-6676-7cb4acd57c64",
#                 "name": "BTDA-1",
#                 "quantity": 1,
#                 "x": 2,
#                 "y": 1,
#                 "Parameters": "{\"molecular\": 1}"
#             },
#             {
#                 "typeId": "3a14233a-84a3-088d-6676-7cb4acd57c64",
#                 "name": "BTDA-2",
#                 "quantity": 1,
#                 "x": 2,
#                 "y": 2,
#                 "Parameters": "{\"molecular\": 1}"
#             },
#             {
#                 "typeId": "3a14233a-84a3-088d-6676-7cb4acd57c64",
#                 "name": "BTDA-3",
#                 "quantity": 1,
#                 "x": 2,
#                 "y": 3,
#                 "Parameters": "{\"molecular\": 1}"
#             }
#         ],
#         "Parameters": "{}"
#     }

#     # 烧杯材料数据定义
#     beaker_materials = [
#         {
#             "typeId": "3a14233b-f0a9-ba84-eaa9-0d4718b361b6",
#             "name": "PDA-1",
#             "unit": "微升",
#             "quantity": 1,
#             "parameters": "{\"DeviceMaterialType\":\"NMP\"}"
#         },
#         {
#             "typeId": "3a14233b-f0a9-ba84-eaa9-0d4718b361b6",
#             "name": "TFDB",
#             "unit": "微升",
#             "quantity": 1,
#             "parameters": "{\"DeviceMaterialType\":\"NMP\"}"
#         },
#         {
#             "typeId": "3a14233b-f0a9-ba84-eaa9-0d4718b361b6",
#             "name": "ODA",
#             "unit": "微升",
#             "quantity": 1,
#             "parameters": "{\"DeviceMaterialType\":\"NMP\"}"
#         },
#         {
#             "typeId": "3a14233b-f0a9-ba84-eaa9-0d4718b361b6",
#             "name": "MPDA",
#             "unit": "微升",
#             "quantity": 1,
#             "parameters": "{\"DeviceMaterialType\":\"NMP\"}"
#         },
#         {
#             "typeId": "3a14233b-f0a9-ba84-eaa9-0d4718b361b6",
#             "name": "PDA-2",
#             "unit": "微升",
#             "quantity": 1,
#             "parameters": "{\"DeviceMaterialType\":\"NMP\"}"
#         }
#     ]

#     # 如果需要，可以在这里调用add_material方法添加材料
#     # 例如:
#     # result = bioyond.add_material(json.dumps(material_data_yp_1))
#     # print(f"添加材料结果: {result}")

#     return {
#         "sample_plates": [material_data_yp_1, material_data_yp_2],
#         "beakers": beaker_materials
#     }


if __name__ == "__main__":
    # 运行主实验流程
    bioyond_client = run_experiment()

    # 可选：准备材料数据
    # materials = prepare_materials(bioyond_client)
    # print(f"\n准备的材料数据: {materials}")
