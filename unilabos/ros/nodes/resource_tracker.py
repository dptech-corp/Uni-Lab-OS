from unilabos.utils.log import logger


class DeviceNodeResourceTracker:

    def __init__(self):
        self.resources = []
        self.root_resource2resource = {}
        pass

    def root_resource(self, resource):
        if id(resource) in self.root_resource2resource:
            return self.root_resource2resource[id(resource)]
        else:
            return resource

    def add_resource(self, resource):
        # 使用内存地址跟踪是否为同一个resource
        for r in self.resources:
            if id(r) == id(resource):
                return
        # 添加资源到跟踪器
        self.resources.append(resource)

    def clear_resource(self):
        self.resources = []

    def figure_resource(self, resource):
        # 使用内存地址跟踪是否为同一个resource
        if isinstance(resource, list):
            return [self.figure_resource(r) for r in resource]
        res_id = resource.id if hasattr(resource, "id") else None
        res_name = resource.name if hasattr(resource, "name") else None
        res_identifier = res_id if res_id else res_name
        identifier_key = "id" if res_id else "name"
        resource_cls_type = type(resource)
        if res_identifier is None:
            logger.warning(f"resource {resource} 没有id或name，暂不能对应figure")
        res_list = []
        for r in self.resources:
            res_list.extend(
                self.loop_find_resource(r, resource_cls_type, identifier_key, getattr(resource, identifier_key))
            )
        assert len(res_list) == 1, f"找到多个资源，请检查资源是否唯一: {res_list}"
        self.root_resource2resource[id(resource)] = res_list[0]
        # 后续加入其他对比方式
        return res_list[0]

    def loop_find_resource(self, resource, resource_cls_type, identifier_key, compare_value):
        res_list = []
        print(resource, resource_cls_type, identifier_key, compare_value)
        children = getattr(resource, "children", [])
        for child in children:
            res_list.extend(self.loop_find_resource(child, resource_cls_type, identifier_key, compare_value))
        if resource_cls_type == type(resource):
            if hasattr(resource, identifier_key):
                if getattr(resource, identifier_key) == compare_value:
                    res_list.append(resource)
        return res_list

    def filter_find_list(self, res_list, compare_std_dict):
        new_list = []
        for res in res_list:
            for k, v in compare_std_dict.items():
                if hasattr(res, k):
                    if getattr(res, k) == v:
                        new_list.append(res)
        return new_list
