import bpy
import json

obj = bpy.context.active_object
if not obj or obj.type != 'MESH':
    print("请激活一个Mesh对象!")
elif not obj.data.shape_keys:
    print("该Mesh没有形态键!")
else:
    sk = obj.data.shape_keys
    json_path = ""  # <--- 请修改为JSON路径
    
    try:
        with open(json_path, "r", encoding="utf-8") as f:
            drivers_data = json.load(f)
    except Exception as e:
        print(f"JSON加载失败: {e}")
        drivers_data = {}
    
    applied_count = 0
    skipped_count = 0
    
    for key_name, driver_info in drivers_data.items():
        kb = sk.key_blocks.get(key_name)
        if kb:
            # 只对存在的形态键进行操作（移除旧驱动器 + 添加新驱动器）
            # 这不会影响其他不在JSON中的形态键及其驱动器
            kb.driver_remove('value')
            
            # 添加新驱动器
            fc = kb.driver_add('value')
            driver = fc.driver
            driver.type = driver_info["type"]
            driver.expression = driver_info["expression"]
            
            # 清空并重新添加变量
            for var_dict in driver_info["variables"]:
                var = driver.variables.new()
                var.name = var_dict["name"]
                var.type = var_dict["type"]
                if var.targets and var_dict["target_id"] and var_dict["data_path"]:
                    target_obj = bpy.data.objects.get(var_dict["target_id"])
                    if target_obj:
                        var.targets[0].id = target_obj
                        var.targets[0].data_path = var_dict["data_path"]
                    else:
                        print(f"警告: 未找到目标对象 {var_dict['target_id']} (形态键: {key_name})")
            
            applied_count += 1
            print(f"成功应用驱动器到形态键: {key_name}")
        else:
            # 未找到形态键 → 完全跳过，不做任何操作，也不影响其他形态键
            print(f"跳过: 未找到形态键 '{key_name}'，不影响其他形态键")
            skipped_count += 1
    
    print(f"\n导入完成！")
    print(f"  - 成功应用: {applied_count} 个驱动器")
    print(f"  - 跳过（形态键不存在）: {skipped_count} 个")
    print(f"  - 其他不在JSON中的形态键及其驱动器保持完全不变")