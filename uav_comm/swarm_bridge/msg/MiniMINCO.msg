uint8 traj_id
uint16[3] max_p
uint16[3] min_p
uint8[3] start_p
uint8[3] end_p
uint16[3] start_v
uint16[3] start_a
uint16[3] end_v
uint16[3] end_a
uint8[] inner_x # 4cm左右精度损失
uint8[] inner_y
uint8[] inner_z 
uint8[] duration # max=5s， 精度0.02s

# MINCO_FULL:
# int16 drone_id # uwb上有id √
# int32 traj_id # 改成uint8 √
# time start_time # 认为uwb没有延迟 √
# float32 des_clearance # 大家都一样，不用管了 √

# uint8 order  # 全都是5 √
# float32[3] start_p
# float32[3] start_v
# float32[3] start_a
# float32[3] end_p
# float32[3] end_v
# float32[3] end_a
# float32[] inner_x
# float32[] inner_y
# float32[] inner_z
# float32[] duration