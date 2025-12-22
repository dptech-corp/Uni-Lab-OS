from ast import If
import pytest
import json
import os

from pylabrobot.resources import Resource as ResourcePLR
from unilabos.resources.graphio import resource_bioyond_to_plr
from unilabos.ros.nodes.resource_tracker import ResourceTreeSet
from unilabos.registry.registry import lab_registry

from unilabos.resources.bioyond.decks import BIOYOND_PolymerReactionStation_Deck
from unilabos.resources.bioyond.decks import YB_Deck

lab_registry.setup()


type_mapping = {
    "加样头(大)": ("YB_jia_yang_tou_da", "3a190ca0-b2f6-9aeb-8067-547e72c11469"),
    "液": ("YB_1BottleCarrier", "3a190ca1-2add-2b23-f8e1-bbd348b7f790"),
    "配液瓶(小)板": ("YB_peiyepingxiaoban", "3a190c8b-3284-af78-d29f-9a69463ad047"),
    "配液瓶(小)": ("YB_pei_ye_xiao_Bottler", "3a190c8c-fe8f-bf48-0dc3-97afc7f508eb"),
}


@pytest.fixture
def bioyond_materials_reaction() -> list[dict]:
    print("加载 BioYond 物料数据...")
    print(os.getcwd())
    with open("bioyond_materials_reaction.json", "r", encoding="utf-8") as f:
        data = json.load(f)
    print(f"加载了 {len(data)} 条物料数据")
    return data


@pytest.fixture
def bioyond_materials_liquidhandling_1() -> list[dict]:
    print("加载 BioYond 物料数据...")
    print(os.getcwd())
    with open("bioyond_materials_liquidhandling_1.json", "r", encoding="utf-8") as f:
        data = json.load(f)
    print(f"加载了 {len(data)} 条物料数据")
    return data


@pytest.fixture
def bioyond_materials_liquidhandling_2() -> list[dict]:
    print("加载 BioYond 物料数据...")
    print(os.getcwd())
    with open("bioyond_materials_liquidhandling_2.json", "r", encoding="utf-8") as f:
        data = json.load(f)
    print(f"加载了 {len(data)} 条物料数据")
    return data


@pytest.mark.parametrize("materials_fixture", [
    "bioyond_materials_reaction",
    "bioyond_materials_liquidhandling_1",
])
def test_resourcetreeset_from_plr() -> list[dict]:
    # 直接加载 bioyond_materials_reaction.json 文件
    current_dir = os.path.dirname(os.path.abspath(__file__))
    json_path = os.path.join(current_dir, "test.json")
    with open(json_path, "r", encoding="utf-8") as f:
        materials = json.load(f)
    deck = YB_Deck("test_deck")
    output = resource_bioyond_to_plr(materials, type_mapping=type_mapping, deck=deck)
    print(output)
    # print(deck.summary())

    r = ResourceTreeSet.from_plr_resources([deck])
    print(r.dump())
    # json.dump(deck.serialize(), open("test.json", "w", encoding="utf-8"), indent=4)

if __name__ == "__main__":
    test_resourcetreeset_from_plr()
