import bpy
import json
import re

# 获取激活对象
obj = bpy.context.active_object
if not obj or obj.type != 'MESH':
    print("请激活一个Mesh对象!")
elif not obj.data.shape_keys:
    print("该Mesh没有形态键!")
else:
    sk = obj.data.shape_keys
    drivers_data = {}
    
    if sk.animation_data and sk.animation_data.drivers:
        for fcurve in sk.animation_data.drivers:
            data_path = fcurve.data_path
            # 匹配 key_blocks["KeyName"].value
            match = re.match(r'key_blocks\["([^"]+)"\]\.value', data_path)
            if match:
                key_name = match.group(1)
                driver = fcurve.driver
                
                var_list = []
                for var in driver.variables:
                    target = var.targets[0] if var.targets else None
                    var_dict = {
                        "name": var.name,
                        #"type": var.type,
                        #"target_id": target.id.name if target and target.id else None,
                        "data_path": target.data_path if target else None
                    }
                    var_list.append(var_dict)
                
                driver_info = {
                    "Mesh_name": obj.name,
                    #"type": driver.type,
                    "expression": driver.expression,
                    "variables": var_list
                }
                
                drivers_data[key_name] = driver_info
    
    # 导出到JSON
    if drivers_data:
        with open("D:\\Desktop\\四元数分解\\shape_drivers.json", "w", encoding="utf-8") as f:  # 修改路径
            json.dump(drivers_data, f, indent=4, ensure_ascii=False)  # ensure_ascii=False 让中文直接显示
        print("导出成功! 包含形态键:", list(drivers_data.keys()))
    else:
        print("没有找到形态键驱动器!")