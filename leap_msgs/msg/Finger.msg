int32 id                                    # 指を一意に識別するID
#uint8 type                                 # 指の種類 (0: Thumb, 1: Index, 2: Middle, 3: Ring, 4: Pinky)
geometry_msgs/Point tip_position            # 指先の位置 (x, y, z)
geometry_msgs/Vector3 tip_direction             # 指の方向ベクトル (x, y, z)
geometry_msgs/Quaternion tip_orientation       # 指の速度ベクトル (x, y, z)
float32 length                              # 指の長さ (mm)
float32 width                               # 指の幅 (mm)

# 指を構成する骨の情報
Bone[] bones                       # 指の骨のリスト (Metacarpal, Proximal, Intermediate, Distal)