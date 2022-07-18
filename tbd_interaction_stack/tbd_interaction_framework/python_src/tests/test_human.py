

import numpy as np
from tbd_interaction_framework.entities.human import Human
from tbd_ros_msgs.msg import HumanBody, HumanJoint
from geometry_msgs.msg import Pose, Point, Quaternion
from alloy.spatial.primitives.box import Box


def create_test_human_msg():
    # pylint: disable=C0303
    """ Original Human Msg
bodies: 
  - 
    header: 
      seq: 792
      stamp: 
        secs: 1617766034
        nsecs:    890112
      frame_id: "PsiWorld"
    body_id: 2
    joints: 
      - 
        joint_id: 1
        pose: 
          position: 
            x: 3.1471661822869264
            y: -0.3026022445176863
            z: 1.1130842081948389
          orientation: 
            x: 0.060221269726753235
            y: -0.08863215893507004
            z: 0.6690384149551392
            w: 0.7354627847671509
      - 
        joint_id: 2
        pose: 
          position: 
            x: 3.1401419191785496
            y: -0.33183391313739025
            z: 1.250908394965599
          orientation: 
            x: 0.06321205198764801
            y: -0.08726140856742859
            z: 0.7049915790557861
            w: 0.7009826898574829
      - 
        joint_id: 3
        pose: 
          position: 
            x: 3.1330195990270635
            y: -0.3781918956962058
            z: 1.4604516164162207
          orientation: 
            x: 0.006472807843238115
            y: -0.030766338109970093
            z: 0.7077901363372803
            w: 0.7057228684425354
      - 
        joint_id: 26
        pose: 
          position: 
            x: 3.130251499679729
            y: -0.3824486729407554
            z: 1.5410844454033263
          orientation: 
            x: -0.1508311927318573
            y: 0.016160858795046806
            z: 0.6617270112037659
            w: 0.7342385053634644
      - 
        joint_id: 27
        pose: 
          position: 
            x: 3.14483966292883
            y: -0.22499983377098462
            z: 1.5570753258040995
          orientation: 
            x: -0.1508311927318573
            y: 0.016160858795046806
            z: 0.6617270112037659
            w: 0.7342385053634644
      - 
        joint_id: 28
        pose: 
          position: 
            x: 3.1101837474575884
            y: -0.25078022391346333
            z: 1.591701814468943
          orientation: 
            x: -0.5745653510093689
            y: 0.5306124687194824
            z: 0.3612576127052307
            w: 0.5077575445175171
      - 
        joint_id: 29
        pose: 
          position: 
            x: 3.041471407813769
            y: -0.3555676479748584
            z: 1.5873057826826888
          orientation: 
            x: 0.66170734167099
            y: -0.7342050075531006
            z: 0.15081754326820374
            w: 0.0161616001278162
      - 
        joint_id: 30
        pose: 
          position: 
            x: 3.1616229787439663
            y: -0.25288253252749593
            z: 1.5965639405057894
          orientation: 
            x: -0.5745653510093689
            y: 0.5306124687194824
            z: 0.3612576127052307
            w: 0.5077575445175171
      - 
        joint_id: 31
        pose: 
          position: 
            x: 3.201802253082512
            y: -0.3744160699432844
            z: 1.6167600831033242
          orientation: 
            x: -0.1508311927318573
            y: 0.016160858795046806
            z: 0.6617270112037659
            w: 0.7342385053634644
      - 
        joint_id: 4
        pose: 
          position: 
            x: 3.1008830425714735
            y: -0.3716759490955071
            z: 1.4247455738567432
          orientation: 
            x: -0.5435274243354797
            y: -0.5680488348007202
            z: 0.39464783668518066
            w: 0.4755541682243347
      - 
        joint_id: 5
        pose: 
          position: 
            x: 2.9638393140536
            y: -0.36197748553595327
            z: 1.3913494713532588
          orientation: 
            x: -0.8766083717346191
            y: -0.2964231073856354
            z: 0.3742358982563019
            w: 0.060560762882232666
      - 
        joint_id: 6
        pose: 
          position: 
            x: 2.774704236310303
            y: -0.39359499550719035
            z: 1.1965925598695502
          orientation: 
            x: -0.8458552956581116
            y: -0.14151307940483093
            z: -0.43931564688682556
            w: 0.2674044072628021
      - 
        joint_id: 7
        pose: 
          position: 
            x: 2.92782962852107
            y: -0.26129961168212223
            z: 1.0885483229760318
          orientation: 
            x: 0.6094802021980286
            y: 0.4957045316696167
            z: 0.5543408393859863
            w: 0.2748021185398102
      - 
        joint_id: 8
        pose: 
          position: 
            x: 3.018260324425309
            y: -0.23560553082122573
            z: 1.0733004465319183
          orientation: 
            x: 0.5018994212150574
            y: 0.5364720821380615
            z: 0.6533419489860535
            w: 0.18285609781742096
      - 
        joint_id: 9
        pose: 
          position: 
            x: 3.1093168544741268
            y: -0.18030499539892397
            z: 1.0648132120560272
          orientation: 
            x: 0.5018994212150574
            y: 0.5364720821380615
            z: 0.6533419489860535
            w: 0.18285609781742096
      - 
        joint_id: 10
        pose: 
          position: 
            x: 3.046233558258439
            y: -0.21126052083107508
            z: 1.0999706430417997
          orientation: 
            x: -0.7073553204536438
            y: -0.2187245488166809
            z: -0.6125052571296692
            w: 0.27685004472732544
      - 
        joint_id: 11
        pose: 
          position: 
            x: 3.167420242367592
            y: -0.3714157097814106
            z: 1.4264597429837906
          orientation: 
            x: 0.46051305532455444
            y: -0.41826528310775757
            z: -0.5793983936309814
            w: 0.5265733003616333
      - 
        joint_id: 12
        pose: 
          position: 
            x: 3.294126641587881
            y: -0.37137629465728367
            z: 1.39706856391085
          orientation: 
            x: 0.1205640360713005
            y: -0.15877152979373932
            z: -0.6185951828956604
            w: 0.7599973678588867
      - 
        joint_id: 13
        pose: 
          position: 
            x: 3.402874023749722
            y: -0.37504516727087006
            z: 1.1407176798753247
          orientation: 
            x: 0.2739882469177246
            y: 0.03982318565249443
            z: -0.5675616264343262
            w: 0.7753828167915344
      - 
        joint_id: 14
        pose: 
          position: 
            x: 3.46077602379551
            y: -0.26584146438113415
            z: 0.944031165921532
          orientation: 
            x: -0.13355505466461182
            y: -0.06388271600008011
            z: 0.9043751955032349
            w: 0.4002354145050049
      - 
        joint_id: 18
        pose: 
          position: 
            x: 3.06389698347506
            y: -0.28223422257478853
            z: 0.9344228333437476
          orientation: 
            x: -0.011949771083891392
            y: -0.02406199648976326
            z: 0.7377555966377258
            w: 0.6745331883430481
      - 
        joint_id: 19
        pose: 
          position: 
            x: 3.083731262403587
            y: -0.2745596214957535
            z: 0.539042616614045
          orientation: 
            x: -0.1087103858590126
            y: 0.0646209642291069
            z: 0.7298008799552917
            w: 0.6718608140945435
      - 
        joint_id: 20
        pose: 
          position: 
            x: 3.110916316988135
            y: -0.36552795247965225
            z: 0.17273917425928242
          orientation: 
            x: -0.07350964844226837
            y: 0.023151371628046036
            z: 0.6569912433624268
            w: 0.7499485611915588
      - 
        joint_id: 21
        pose: 
          position: 
            x: 3.1377380087733457
            y: -0.236213487011472
            z: 0.042777656974786554
          orientation: 
            x: -0.5180795788764954
            y: 0.5426492094993591
            z: 0.41736412048339844
            w: 0.512769341468811
      - 
        joint_id: 22
        pose: 
          position: 
            x: 3.2345078113361385
            y: -0.29895969930704325
            z: 0.939570751965022
          orientation: 
            x: -0.6110990047454834
            y: 0.7777519822120667
            z: -0.0057307216338813305
            w: 0.14704710245132446
      - 
        joint_id: 23
        pose: 
          position: 
            x: 3.327747068060541
            y: -0.2314009437760438
            z: 0.5611757485225768
          orientation: 
            x: 0.5966394543647766
            y: -0.791249692440033
            z: 0.1321316510438919
            w: 0.01737944968044758
      - 
        joint_id: 24
        pose: 
          position: 
            x: 3.3775349101593872
            y: -0.31930727974250656
            z: 0.19237440971493702
          orientation: 
            x: 0.6407111883163452
            y: -0.7479745745658875
            z: 0.16460664570331573
            w: 0.05407211557030678
      - 
        joint_id: 25
        pose: 
          position: 
            x: 3.4149797414736764
            y: -0.22296801219832094
            z: 0.056808160339019725
          orientation: 
            x: 0.33562105894088745
            y: -0.4963071346282959
            z: 0.5651989579200745
            w: 0.5670874714851379
      - 
        joint_id: 0
        pose: 
          position: 
            x: 3.1536098078357164
            y: -0.29102897305392716
            z: 0.9371298131385674
          orientation: 
            x: 0.007054818794131279
            y: -0.030116770416498184
            z: 0.6717061996459961
            w: 0.7401716709136963
---
    """
# pylint: enable=C0303
    msg = HumanBody(body_id=2)
    msg.joints.append(HumanJoint(joint_id=1,
                                 pose=Pose(
                                     position=Point(x=3.1472, y=-0.3026, z=1.1131),
                                     orientation=Quaternion(x=0.0602, y=-0.0886, z=0.6690, w=0.7355)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=2,
                                 pose=Pose(
                                     position=Point(x=3.1401, y=-0.3318, z=1.2509),
                                     orientation=Quaternion(x=0.0632, y=-0.0872, z=0.7050, w=0.7010)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=3,
                                 pose=Pose(
                                     position=Point(x=3.1330, y=-0.3782, z=1.4605),
                                     orientation=Quaternion(x=0.0647, y=-0.0308, z=0.7078, w=0.7057)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=26,
                                 pose=Pose(
                                     position=Point(x=3.1303, y=-0.3824, z=1.5411),
                                     orientation=Quaternion(x=-0.1508, y=0.0162, z=0.6617, w=0.7342)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=27,
                                 pose=Pose(
                                     position=Point(x=3.1448, y=-0.2250, z=1.5571),
                                     orientation=Quaternion(x=-0.1508, y=0.0162, z=0.6617, w=0.7342)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=28,
                                 pose=Pose(
                                     position=Point(x=3.1102, y=-0.2508, z=1.5917),
                                     orientation=Quaternion(x=-0.5745, y=0.5306, z=0.3613, w=0.5078)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=29,
                                 pose=Pose(
                                     position=Point(x=3.0415, y=-0.3556, z=1.5873),
                                     orientation=Quaternion(x=0.6617, y=-0.7342, z=0.1508, w=0.0162)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=30,
                                 pose=Pose(
                                     position=Point(x=3.1616, y=-0.2529, z=1.5966),
                                     orientation=Quaternion(x=-0.5746, y=0.5306, z=0.3613, w=0.5078)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=31,
                                 pose=Pose(
                                     position=Point(x=3.2018, y=-0.3744, z=1.6168),
                                     orientation=Quaternion(x=-0.1508, y=0.0162, z=0.6617, w=0.7342)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=4,
                                 pose=Pose(
                                     position=Point(x=3.1009, y=-0.3717, z=1.4247),
                                     orientation=Quaternion(x=-0.5435, y=-0.5680, z=0.3946, w=0.4755)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=4,
                                 pose=Pose(
                                     position=Point(x=2.9638, y=-0.3620, z=1.3913),
                                     orientation=Quaternion(x=-0.8766, y=-0.2964, z=0.3742, w=0.0606)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=6,
                                 pose=Pose(
                                     position=Point(x=2.775, y=-0.3936, z=1.1966),
                                     orientation=Quaternion(x=-0.8458, y=-0.1415, z=-0.4393, w=0.2674)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=7,
                                 pose=Pose(
                                     position=Point(x=2.9278, y=-0.2613, z=1.0885),
                                     orientation=Quaternion(x=0.6095, y=0.4957, z=0.5543, w=0.2748)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=8,
                                 pose=Pose(
                                     position=Point(x=3.0183, y=-0.2356, z=1.0733),
                                     orientation=Quaternion(x=0.5019, y=0.5365, z=0.6533, w=0.1826)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=9,
                                 pose=Pose(
                                     position=Point(x=3.1093, y=-0.1803, z=1.0648),
                                     orientation=Quaternion(x=0.5019, y=0.5365, z=0.6533, w=0.1826)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=10,
                                 pose=Pose(
                                     position=Point(x=3.0462, y=-0.2113, z=1.1000),
                                     orientation=Quaternion(x=-0.7074, y=-0.2187, z=-0.6125, w=0.2769)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=11,
                                 pose=Pose(
                                     position=Point(x=3.1674, y=-0.3714, z=1.4265),
                                     orientation=Quaternion(x=0.4605, y=-0.4183, z=-0.5794, w=0.5266)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=12,
                                 pose=Pose(
                                     position=Point(x=3.2941, y=-0.3714, z=1.3971),
                                     orientation=Quaternion(x=0.1206, y=-0.1588, z=-0.6186, w=0.7600)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=13,
                                 pose=Pose(
                                     position=Point(x=3.4029, y=-0.3750, z=1.1407),
                                     orientation=Quaternion(x=0.2740, y=-0.0398, z=-0.5676, w=0.7754)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=14,
                                 pose=Pose(
                                     position=Point(x=3.46077, y=-0.2658, z=0.9440),
                                     orientation=Quaternion(x=-0.1336, y=-0.0639, z=0.9044, w=0.4002)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=18,
                                 pose=Pose(
                                     position=Point(x=3.0639, y=-0.2822, z=0.9344),
                                     orientation=Quaternion(x=-0.0119, y=-0.0241, z=0.7378, w=0.6745)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=19,
                                 pose=Pose(
                                     position=Point(x=3.0837, y=-0.2745, z=0.5390),
                                     orientation=Quaternion(x=-0.1087, y=0.0646, z=0.7298, w=0.6719)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=20,
                                 pose=Pose(
                                     position=Point(x=3.1109, y=-0.3655, z=0.1727),
                                     orientation=Quaternion(x=-0.0735, y=0.0232, z=0.6569, w=0.7499)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=21,
                                 pose=Pose(
                                     position=Point(x=3.1377, y=-0.2362, z=0.0428),
                                     orientation=Quaternion(x=-0.5181, y=0.5426, z=0.4174, w=0.5128)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=21,
                                 pose=Pose(
                                     position=Point(x=3.1377, y=-0.2362, z=0.0428),
                                     orientation=Quaternion(x=-0.5181, y=0.5426, z=0.4174, w=0.5128)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=22,
                                 pose=Pose(
                                     position=Point(x=3.2345, y=-0.2989, z=0.9396),
                                     orientation=Quaternion(x=-0.6110, y=0.7777, z=-0.0057, w=0.1470)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=23,
                                 pose=Pose(
                                     position=Point(x=3.3277, y=-0.2314, z=0.5612),
                                     orientation=Quaternion(x=0.5966, y=-0.7912, z=0.1321, w=0.0174)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=24,
                                 pose=Pose(
                                     position=Point(x=3.3775, y=-0.3193, z=0.1924),
                                     orientation=Quaternion(x=0.6407, y=-0.7480, z=0.1646, w=0.0541)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=25,
                                 pose=Pose(
                                     position=Point(x=3.4150, y=-0.2230, z=0.0568),
                                     orientation=Quaternion(x=0.3356, y=-0.04963, z=0.5620, w=0.5671)
                                 )
                                 ))
    msg.joints.append(HumanJoint(joint_id=0,
                                 pose=Pose(
                                     position=Point(x=3.1536, y=-0.2910, z=0.9371),
                                     orientation=Quaternion(x=0.0071, y=-0.0301, z=0.6717, w=0.7402)
                                 )
                                 ))
    return msg


def test_human_bounding_box():

    msg = create_test_human_msg()
    h = Human(msg)
    box = h.bounding_box
    # The X-Y should be about equal to the human base
    np.testing.assert_array_almost_equal(box.center[:2], h.base[:2, 3], decimal=1)


def test_human_bounding_box_collision():

    msg = create_test_human_msg()
    h = Human(msg)
    box = h.bounding_box
    inflated_box = Box.inflate_box(box, 0.1, 0.1)
    np.testing.assert_approx_equal(inflated_box.length, box.length + 0.1)
    np.testing.assert_approx_equal(inflated_box.width, box.width + 0.1)

    # test collision point
    assert inflated_box.contains_point(h.base[:3, 3])
    # test zero height
    assert inflated_box.contains_point([3.3775, -0.3193, 0])
    assert inflated_box.contains_point([3.3, -0.319, 0])
    # test negative height
    assert not inflated_box.contains_point([3.3, -0.319, -0.1])

    # test a point slightly larger
    test_point = box.center + box.half_extents
    test_point[2] = 0
    test_point = test_point + [0.01, 0.01, 0]
    assert not box.contains_point(test_point)
    assert inflated_box.contains_point(test_point)
