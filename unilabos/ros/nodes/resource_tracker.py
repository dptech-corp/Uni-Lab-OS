from typing import List, Tuple, Any

from unilabos.utils.log import logger


class DeviceNodeResourceTracker(object):

    def __init__(self):
        self.resources = []
        self.resource2parent_resource = {}
        pass

    def parent_resource(self, resource):
        if id(resource) in self.resource2parent_resource:
            return self.resource2parent_resource[id(resource)]
        else:
            return resource

    def add_resource(self, resource):
        for r in self.resources:
            if id(r) == id(resource):
                return
        self.resources.append(resource)

    def clear_resource(self):
        self.resources = []

    def figure_resource(self, query_resource, try_mode=False):
        if isinstance(query_resource, list):
            return [self.figure_resource(r) for r in query_resource]
        res_id = query_resource.id if hasattr(query_resource, "id") else (query_resource.get("id") if isinstance(query_resource, dict) else None)
        res_name = query_resource.name if hasattr(query_resource, "name") else (query_resource.get("name") if isinstance(query_resource, dict) else None)
        res_identifier = res_id if res_id else res_name
        identifier_key = "id" if res_id else "name"
        resource_cls_type = type(query_resource)
        if res_identifier is None:
            logger.warning(f"resource {query_resource} 没有id或name，暂不能对应figure")
        res_list = []
        for r in self.resources:
            if isinstance(query_resource, dict):
                res_list.extend(
                    self.loop_find_resource(r, resource_cls_type, identifier_key, query_resource[identifier_key])
                )
            else:
                res_list.extend(
                    self.loop_find_resource(r, resource_cls_type, identifier_key, getattr(query_resource, identifier_key))
                )
        if not try_mode:
            assert len(res_list) > 0, f"没有找到资源 {query_resource}，请检查资源是否存在"
            assert len(res_list) == 1, f"{query_resource} 找到多个资源，请检查资源是否唯一: {res_list}"
        else:
            return [i[1] for i in res_list]
        # 后续加入其他对比方式
        self.resource2parent_resource[id(query_resource)] = res_list[0][0]
        self.resource2parent_resource[id(res_list[0][1])] = res_list[0][0]
        return res_list[0][1]

    def loop_find_resource(self, resource, target_resource_cls_type, identifier_key, compare_value, parent_res=None) -> List[Tuple[Any, Any]]:
        res_list = []
        # print(resource, target_resource_cls_type, identifier_key, compare_value)
        children = getattr(resource, "children", [])
        for child in children:
            res_list.extend(self.loop_find_resource(child, target_resource_cls_type, identifier_key, compare_value, resource))
        if target_resource_cls_type == type(resource):
            if target_resource_cls_type == dict:
                if identifier_key in resource:
                    if resource[identifier_key] == compare_value:
                        res_list.append((parent_res, resource))
            elif hasattr(resource, identifier_key):
                if getattr(resource, identifier_key) == compare_value:
                    res_list.append((parent_res, resource))
        return res_list

    def filter_find_list(self, res_list, compare_std_dict):
        new_list = []
        for res in res_list:
            for k, v in compare_std_dict.items():
                if hasattr(res, k):
                    if getattr(res, k) == v:
                        new_list.append(res)
        return new_list
