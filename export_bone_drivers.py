import bpy
import json
import re

# 确保在Pose模式
if bpy.context.mode != 'POSE':
    print("请切换到Pose模式!")
else:
    armature = bpy.context.active_object
    if armature.type != 'ARMATURE':
        print("激活对象必须是Armature!")
    else:
        selected_bones = bpy.context.selected_pose_bones
        if not selected_bones:
            print("没有选中骨骼!")
        else:
            drivers_data = {}
            # 迭代动画数据的drivers
            if armature.animation_data and armature.animation_data.drivers:
                for fcurve in armature.animation_data.drivers:
                    data_path = fcurve.data_path
                    # 使用正则匹配约束驱动器: pose.bones["bone"].constraints["con"].prop
                    match = re.match(r'pose\.bones\["([^"]+)"\]\.constraints\["([^"]+)"\]\.(\w+)', data_path)
                    if match:
                        bone_name, con_name, prop_name = match.groups()
                        
                        # 只处理选中的骨骼
                        if any(b.name == bone_name for b in selected_bones):
                            driver = fcurve.driver
                            var_list = []
                            for var in driver.variables:
                                var_dict = {
                                    "name": var.name,
                                    #"type": var.type,
                                    #"target_id": var.targets[0].id.name if var.targets and var.targets[0].id else None,
                                    "data_path": var.targets[0].data_path if var.targets else None
                                }
                                var_list.append(var_dict)
                            
                            driver_info = {
                                "Armature_name": armature.name,
                                #"type": driver.type,
                                "expression": driver.expression,
                                "variables": var_list
                            }
                            
                            if bone_name not in drivers_data:
                                drivers_data[bone_name] = {}
                            if con_name not in drivers_data[bone_name]:
                                drivers_data[bone_name][con_name] = {}
                            drivers_data[bone_name][con_name][prop_name] = driver_info
            
            # 导出到JSON
            if drivers_data:
                with open("D:\\Desktop\\四元数分解\\pose_constraints_drivers_export.json", "w") as f:  # 修改路径
                    json.dump(drivers_data, f, indent=4)
                print("导出成功!")
            else:
                print("没有找到驱动器! 如果有drivers但未匹配，检查data_path格式。")