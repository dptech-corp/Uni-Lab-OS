import pytest
import json
import os

from unilabos.resources.graphio import resource_bioyond_to_plr
from unilabos.registry.registry import lab_registry

from unilabos.resources.bioyond.decks import BIOYOND_PolymerReactionStation_Deck

lab_registry.setup()


type_mapping = {
    "烧杯": "BIOYOND_PolymerStation_1FlaskCarrier",
    "试剂瓶": "BIOYOND_PolymerStation_1BottleCarrier",
    "样品板": "BIOYOND_PolymerStation_6VialCarrier",
}

@pytest.fixture
def bioyond_materials() -> list[dict]:
    print("加载 BioYond 物料数据...")
    print(os.getcwd())
    with open("bioyond_materials.json", "r", encoding="utf-8") as f:
        data = json.load(f)["data"]
    print(f"加载了 {len(data)} 条物料数据")
    return data


def test_bioyond_to_plr(bioyond_materials) -> list[dict]:
    deck = BIOYOND_PolymerReactionStation_Deck("test_deck")
    print("将 BioYond 物料数据转换为 PLR 格式...")
    output = resource_bioyond_to_plr(bioyond_materials, type_mapping=type_mapping, deck=deck)
    print(deck.summary())
    print([resource.serialize() for resource in output])
    print([resource.serialize_all_state() for resource in output])
