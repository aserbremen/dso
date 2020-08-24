# ========================= eCAL LICENSE =================================
#
# Copyright (C) 2016 - 2019 Continental Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ========================= eCAL LICENSE =================================
import os
import sys
import time
import csv
import math
import numpy as np
import copy

import ecal
from ecal.measurement.measurement import Measurement, Channel

deg2rad = math.pi / 180
us2s = 1.0 / 1000.0 / 1000.0

IMU = "VehicleDynamicsImuPb"
VEH_DYNAMICS = "VehicleDynamicsInPb"
RT4000 = "RT4000DataInPb"



# koeln_meas_paths=[
#     # runde 1
#     "/data/datasets/conti/20191021_Koeln_lang/Runde01/2019-10-21-10-10-47-153_F-TZ_9900_@City_FKA_Messkampagne",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde01/2019-10-21-10-25-06-678_F-TZ_9900_@City_FKA_Messkampagne", 
#     "/data/datasets/conti/20191021_Koeln_lang/Runde01/2019-10-21-10-27-15-957_F-TZ_9900_@City_FKA_Messkampagne",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde01/2019-10-21-10-34-08-564_F-TZ_9900_@City_FKA_Messkampagne",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde01/2019-10-21-10-43-56-300_F-TZ_9900_@City_FKA_Messkampagne",
#     # /data/datasets/conti/20191021_Koeln_lang/Runde01/2019-10-21-11-01-26-652_F-TZ_9900_@City_FKA_Messkampagne
#     # runde 2
#     "/data/datasets/conti/20191021_Koeln_lang/Runde02/2019-10-21-11-21-54-442_F-TZ_9900_@City_FKA_Messkampagne",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde02/2019-10-21-11-23-48-033_F-TZ_9900_@City_FKA_Messkampagne",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde02/2019-10-21-11-30-50-344_F-TZ_9900_@City_FKA_Messkampagne",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde02/2019-10-21-11-32-32-792_F-TZ_9900_@City_FKA_Messkampagne",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde02/2019-10-21-11-39-02-444_F-TZ_9900_@City_FKA_Messkampagne",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde02/2019-10-21-11-46-03-646_F-TZ_9900_@City_FKA_Messkampagne",
#     # runde 3
#     "/data/datasets/conti/20191021_Koeln_lang/Runde03/2019-10-21-14-27-49-996_F-TZ_9900_@City_FKA_Messkampagne",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde03/2019-10-21-14-45-55-742_F-TZ_9900_@City_FKA_Messkampagne",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde03/2019-10-21-14-54-22-334_F-TZ_9900_@City_FKA_Messkampagne",
#     # runde 4
#     "/data/datasets/conti/20191021_Koeln_lang/Runde04/2019-10-21-15-03-07-219_F-TZ_9900_@City_FKA_Messkampagne",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde04/2019-10-21-15-12-31-707_F-TZ_9900_@City_FKA_Messkampagne",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde04/2019-10-21-15-25-03-019_F-TZ_9900_@City_FKA_Messkampagne",
#     # runde 5
#     "/data/datasets/conti/20191021_Koeln_lang/Runde05/2019-10-21-15-32-29-226_F-TZ_9900_@City_FKA_Messkampagne",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde05/2019-10-21-15-40-29-596_F-TZ_9900_@City_FKA_Messkampagne",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde05/2019-10-21-15-52-50-473_F-TZ_9900_@City_FKA_Messkampagne",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde05/2019-10-21-15-57-00-471_F-TZ_9900_@City_FKA_Messkampagne",
#     # runde 6
#     "/data/datasets/conti/20191021_Koeln_lang/Runde06/2019-10-21-16-07-16-504_F-TZ_9900_@City_FKA_Messkampagne",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde06/2019-10-21-16-17-46-423_F-TZ_9900_@City_FKA_Messkampagne"
#     # /data/datasets/conti/20191021_Koeln_lang/Runde06/2019-10-21-16-27-13-912_F-TZ_9900_@City_FKA_Messkampagne
# ]

# koeln_image_paths=(
#     #runde 1
#     "/data/datasets/conti/20191021_Koeln_lang/Runde01/dw_2019_10_21_03_24_23_000000_192.0.0.112_f3a53a6a-f3ec-11e9-ac60-00044babffd2",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde01/dw_2019_10_21_03_26_36_000000_192.0.0.112_42e0a718-f3ed-11e9-ac60-00044babffd2",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde01/dw_2019_10_21_03_28_45_000000_192.0.0.112_8fc2d060-f3ed-11e9-ac60-00044babffd2",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde01/dw_2019_10_21_03_35_37_000000_192.0.0.112_858e465a-f3ee-11e9-ac60-00044babffd2",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde01/dw_2019_10_21_03_44_38_000000_192.0.0.112_c7d55606-f3ef-11e9-ac60-00044babffd2",
#     # /data/datasets/conti/20191021_Koeln_lang/Runde01/dw_2019_10_21_03_45_30_000000_192.0.0.112_e6c9957c-f3ef-11e9-ac60-00044babffd2
#     # runde 2
#     "/data/datasets/conti/20191021_Koeln_lang/Runde02/dw_2019_10_21_04_23_34_000000_192.0.0.112_3847d6e8-f3f5-11e9-ac60-00044babffd2",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde02/dw_2019_10_21_04_25_22_000000_192.0.0.112_784ce7f6-f3f5-11e9-ac60-00044babffd2",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde02/dw_2019_10_21_04_32_19_000000_192.0.0.112_714c5954-f3f6-11e9-ac60-00044babffd2",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde02/dw_2019_10_21_04_34_07_000000_192.0.0.112_b14d3456-f3f6-11e9-ac60-00044babffd2",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde02/dw_2019_10_21_04_40_41_000000_192.0.0.112_9c3a019c-f3f7-11e9-ac60-00044babffd2",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde02/dw_2019_10_21_04_47_37_000000_192.0.0.112_940c98da-f3f8-11e9-ac60-00044babffd2",
#     # runde 3
#     "/data/datasets/conti/20191021_Koeln_lang/Runde03/dw_2019_10_21_07_39_08_000000_192.0.0.112_8a073bac-f410-11e9-837b-00044babffd2",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde03/dw_2019_10_21_07_48_06_000000_192.0.0.112_ca82a3e6-f411-11e9-ae39-00044babffd2",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde03/dw_2019_10_21_07_55_50_000000_192.0.0.112_df8566e2-f412-11e9-ae39-00044babffd2",
#     #runde 4
#     "/data/datasets/conti/20191021_Koeln_lang/Runde04/dw_2019_10_21_08_04_35_000000_192.0.0.112_188191fe-f414-11e9-ae39-00044babffd2",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde04/dw_2019_10_21_08_14_00_000000_192.0.0.112_68d6832a-f415-11e9-ae39-00044babffd2",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde04/dw_2019_10_21_08_26_32_000000_192.0.0.112_291a0f7a-f417-11e9-ae39-00044babffd2",
#     #runde 5
#     "/data/datasets/conti/20191021_Koeln_lang/Runde05/dw_2019_10_21_08_33_57_000000_192.0.0.112_32a07f74-f418-11e9-ae39-00044babffd2",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde05/dw_2019_10_21_08_41_58_000000_192.0.0.112_50fe8046-f419-11e9-ae39-00044babffd2",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde05/dw_2019_10_21_08_54_18_000000_192.0.0.112_0a050c9e-f41b-11e9-ae39-00044babffd2",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde05/dw_2019_10_21_08_58_28_000000_192.0.0.112_9f825088-f41b-11e9-ae39-00044babffd2",
#     #runde 6
#     "/data/datasets/conti/20191021_Koeln_lang/Runde06/dw_2019_10_21_09_08_42_000000_192.0.0.112_0d5c815e-f41d-11e9-ae39-00044babffd2",
#     "/data/datasets/conti/20191021_Koeln_lang/Runde06/dw_2019_10_21_09_19_08_000000_192.0.0.112_822b31f0-f41e-11e9-ae39-00044babffd2"
#     # /data/datasets/conti/20191021_Koeln_lang/Runde06/dw_2019_10_21_09_28_41_000000_192.0.0.112_d830471a-f41f-11e9-ae39-00044babffd2
# )

# wuppertal_meas_paths=(
#     # runde 1
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde01/2019-10-24-13-53-17-538_F-TZ_9900_@City_FKA_Messkampagne
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde01/2019-10-24-13-55-05-174_F-TZ_9900_@City_FKA_Messkampagne
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde01/2019-10-24-14-05-26-133_F-TZ_9900_@City_FKA_Messkampagne
#     # runde 2
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde02/2019-10-24-14-15-05-243_F-TZ_9900_@City_FKA_Messkampagne
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde02/2019-10-24-14-16-31-814_F-TZ_9900_@City_FKA_Messkampagne
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde02/2019-10-24-14-21-56-271_F-TZ_9900_@City_FKA_Messkampagne
#     # runde 3
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde03/2019-10-24-14-28-15-287_F-TZ_9900_@City_FKA_Messkampagne
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde03/2019-10-24-14-29-47-611_F-TZ_9900_@City_FKA_Messkampagne
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde03/2019-10-24-14-35-34-779_F-TZ_9900_@City_FKA_Messkampagne
#     # runde 4
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde04/2019-10-24-14-41-09-682_F-TZ_9900_@City_FKA_Messkampagne
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde04/2019-10-24-14-45-44-422_F-TZ_9900_@City_FKA_Messkampagne
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde04/2019-10-24-14-56-57-610_F-TZ_9900_@City_FKA_Messkampagne
#     # runde 5
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde05/2019-10-24-15-01-12-701_F-TZ_9900_@City_FKA_Messkampagne
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde05/2019-10-24-15-04-29-377_F-TZ_9900_@City_FKA_Messkampagne
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde05/2019-10-24-15-09-28-349_F-TZ_9900_@City_FKA_Messkampagne
#     #runde 6
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde06/2019-10-24-15-14-57-604_F-TZ_9900_@City_FKA_Messkampagne
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde06/2019-10-24-15-17-41-829_F-TZ_9900_@City_FKA_Messkampagne
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde06/2019-10-24-15-45-10-902_F-TZ_9900_@City_FKA_Messkampagne
# )

# wuppertal_adl_files=(
#     # runde 1
#     wuppertal_2019_10_24_13_53_17
#     wuppertal_2019_10_24_13_55_05
#     wuppertal_2019_10_24_14_05_26
#     # runde 2
#     wuppertal_2019_10_24_14_15_05
#     wuppertal_2019_10_24_14_16_31
#     wuppertal_2019_10_24_14_21_56
#     # runde 3
#     wuppertal_2019_10_24_14_28_15
#     wuppertal_2019_10_24_14_29_47
#     wuppertal_2019_10_24_14_35_34
#     # runde 4
#     wuppertal_2019_10_24_14_41_09
#     wuppertal_2019_10_24_14_45_44
#     wuppertal_2019_10_24_14_56_57
#     # runde 5
#     wuppertal_2019_10_24_15_01_12
#     wuppertal_2019_10_24_15_04_29
#     wuppertal_2019_10_24_15_09_28
#     #runde 6
#     wuppertal_2019_10_24_15_14_57
#     wuppertal_2019_10_24_15_17_41
#     wuppertal_2019_10_24_15_45_10
# )

# wuppertal_adl_video_files=(
#     # runde 1
#     wuppertal_video_2019_10_24_13_53_17
#     wuppertal_video_2019_10_24_13_55_05
#     wuppertal_video_2019_10_24_14_05_26
#     # runde 2
#     wuppertal_video_2019_10_24_14_15_05
#     wuppertal_video_2019_10_24_14_16_31
#     wuppertal_video_2019_10_24_14_21_56
#     # runde 3
#     wuppertal_video_2019_10_24_14_28_15
#     wuppertal_video_2019_10_24_14_29_47
#     wuppertal_video_2019_10_24_14_35_34
#     # runde 4
#     wuppertal_video_2019_10_24_14_41_09
#     wuppertal_video_2019_10_24_14_45_44
#     wuppertal_video_2019_10_24_14_56_57
#     # runde 5
#     wuppertal_video_2019_10_24_15_01_12
#     wuppertal_video_2019_10_24_15_04_29
#     wuppertal_video_2019_10_24_15_09_28
#     #runde 6
#     wuppertal_video_2019_10_24_15_14_57
#     wuppertal_video_2019_10_24_15_17_41
#     wuppertal_video_2019_10_24_15_45_10
# )

# wuppertal_adl_video_gnss_files=(
#     # runde 1
#     wuppertal_video_gnss_2019_10_24_13_53_17
#     wuppertal_video_gnss_2019_10_24_13_55_05
#     wuppertal_video_gnss_2019_10_24_14_05_26
#     # runde 2
#     wuppertal_video_gnss_2019_10_24_14_15_05
#     wuppertal_video_gnss_2019_10_24_14_16_31
#     wuppertal_video_gnss_2019_10_24_14_21_56
#     # runde 3
#     wuppertal_video_gnss_2019_10_24_14_28_15
#     wuppertal_video_gnss_2019_10_24_14_29_47
#     wuppertal_video_gnss_2019_10_24_14_35_34
#     # runde 4
#     wuppertal_video_gnss_2019_10_24_14_41_09
#     wuppertal_video_gnss_2019_10_24_14_45_44
#     wuppertal_video_gnss_2019_10_24_14_56_57
#     # runde 5
#     wuppertal_video_gnss_2019_10_24_15_01_12
#     wuppertal_video_gnss_2019_10_24_15_04_29
#     wuppertal_video_gnss_2019_10_24_15_09_28
#     #runde 6
#     wuppertal_video_gnss_2019_10_24_15_14_57
#     wuppertal_video_gnss_2019_10_24_15_17_41
#     wuppertal_video_gnss_2019_10_24_15_45_10    
# )

# wuppertal_image_paths=(
#     # runde 1
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde01/dw_2019_10_24_06_54_46_000000_192.0.0.112_d67dca62-f665-11e9-988a-00044babffd2
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde01/dw_2019_10_24_06_56_34_000000_192.0.0.112_1710becc-f666-11e9-988a-00044babffd2
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde01/dw_2019_10_24_07_06_56_000000_192.0.0.112_89a06018-f667-11e9-988a-00044babffd2
#     # runde 2
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde02/dw_2019_10_24_07_16_34_000000_192.0.0.112_e26f55ae-f668-11e9-988a-00044babffd2
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde02/dw_2019_10_24_07_18_01_000000_192.0.0.112_15e29fa4-f669-11e9-988a-00044babffd2
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde02/dw_2019_10_24_07_23_26_000000_192.0.0.112_d7bd006a-f669-11e9-988a-00044babffd2
#     # runde 3
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde03/dw_2019_10_24_07_29_44_000000_192.0.0.112_b968f500-f66a-11e9-988a-00044babffd2
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde03/dw_2019_10_24_07_31_21_000000_192.0.0.112_f2fa93fa-f66a-11e9-988a-00044babffd2
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde03/dw_2019_10_24_07_37_02_000000_192.0.0.112_be40fd24-f66b-11e9-988a-00044babffd2
#     # runde 4
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde04/dw_2019_10_24_07_42_38_000000_192.0.0.112_8687e5f4-f66c-11e9-988a-00044babffd2
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde04/dw_2019_10_24_07_47_18_000000_192.0.0.112_2d515082-f66d-11e9-988a-00044babffd2
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde04/dw_2019_10_24_07_58_25_000000_192.0.0.112_bb265776-f66e-11e9-988a-00044babffd2
#     # runde 5
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde05/dw_2019_10_24_08_02_40_000000_192.0.0.112_52b3766e-f66f-11e9-988a-00044babffd2
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde05/dw_2019_10_24_08_05_57_000000_192.0.0.112_c899284c-f66f-11e9-988a-00044babffd2
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde05/dw_2019_10_24_08_10_56_000000_192.0.0.112_7a76fb98-f670-11e9-988a-00044babffd2
#     # runde 6
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde06/dw_2019_10_24_08_16_27_000000_192.0.0.112_3fcf9db4-f671-11e9-988a-00044babffd2
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde06/dw_2019_10_24_08_19_11_000000_192.0.0.112_a17af4e6-f671-11e9-988a-00044babffd2
#     /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde06/dw_2019_10_24_08_46_41_000000_192.0.0.112_78db5568-f675-11e9-988a-00044babffd2
# )



# reference: [1] Wendel, 2011, Integrierte Navigationssysteme: Sensordatenfusion, GPS und Inertiale Navigation 2. Auflage
# WGS84 Erdmodell, page 31
a = 6378737.0           # grosse Halbachse
b = 6356752.3142        # kleine Halbachse
e = 0.0818191908426     # excentricity

# [1] WGS84 Erdmodell, page 31
def eastWestCurvature(lat):
   return a / np.sqrt(1-e**2*np.sin(lat)**2)

# [1] WGS84 Erdmodell, page 31
def nortSouthCurvature(lat):
   return a * (1-e**2) / (1-e**2*np.sin(lat))**(3.0/2.0)

def euler_to_Quaternion(roll, pitch, yaw):
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return Quaternion(w, np.array([x, y, z]))

# according to [1] Equation. 8.68
def pos_boxminus(latlonaltA, latlonaltB):
   return np.array([(latlonaltA[1] - latlonaltB[1]) * (eastWestCurvature(latlonaltA[0]) - latlonaltB[2]) * np.cos(latlonaltA[0]),
                    (latlonaltA[0] - latlonaltB[0]) * (nortSouthCurvature(latlonaltA[0]) - latlonaltB[2]),
                    latlonaltA[2] - latlonaltB[2]])

class Quaternion:
    def __init__(self, w=1, v=np.zeros((3, 1))):
        self.w = w
        self.v = np.matrix(v)
        if self.v.shape in ((3,), (1, 3)):
            self.v = self.v.transpose()
        if self.v.shape != (3, 1):
            raise Exception(
                "Invalid shape of vector part: " + str(self.v.shape))

    def __mul__(self, other):
        if isinstance(other, Quaternion):
            res = Quaternion()
            res.w = self.w*other.w - self.v[0]*other.v[0] - self.v[1] * other.v[1] - self.v[2]*other.v[2]
            res.v[0] = self.w*other.v[0] + self.v[0]*other.w + self.v[1]*other.v[2] - self.v[2]*other.v[1]
            res.v[1] = self.w*other.v[1] - self.v[0]*other.v[2] + self.v[1]*other.w + self.v[2]*other.v[0]
            res.v[2] = self.w*other.v[2] + self.v[0]*other.v[1] - self.v[1]*other.v[0] + self.v[2]*other.w
            # convert to scalar, otherwise we have a 1x1 matrix
            res.w = res.w[0, 0]
            return res
        else:
            res = self * Quaternion(0, other) * self.conjugation()
            return res.v

    def conjugation(self):
        res = copy.deepcopy(self)
        res.conjugate()
        return res

    def conjugate(self):
        self.v *= -1

    def inverse(self):
        res = copy.deepcopy(self)
        res.invert()
        return res

    def invert(self):
        norm_sqr = self.squaredNorm()
        if norm_sqr > 0:
            self.conjugate()
            self.w /= norm_sqr
            self.v /= norm_sqr

    def squaredNorm(self):
        return self.w**2 + np.linalg.norm(self.v)**2

    def norm(self):
        return np.sqrt(self.squaredNorm())

    def boxplus(self, d):
        d *= 0.5
        d_norm = np.linalg.norm(d)
        if np.fabs(d_norm) < 1e-8:
            return self
        else:
            exp_d = Quaternion(np.cos(d_norm), np.sin(d_norm) / d_norm * d)
        return self * exp_d

    def boxminus(self, other):
        delta = other.inverse() * self
        v_norm = np.linalg.norm(delta.v)
        if np.fabs(v_norm) < 1e-6:
            return 2 * np.matrix(np.zeros((3, 1)))
        if np.fabs(delta.w) < 1e-6:
            return np.pi / v_norm * delta.v
        return 2 * np.arctan(v_norm / delta.w) / v_norm * delta.v

    def to_matrix(self):
        ii = self.v[0, 0]**2
        jj = self.v[1, 0]**2
        kk = self.v[2, 0]**2
        n = self.w**2 + ii + jj + kk
        if n == 0:
            s = 0
        else:
            s = 2.0 / n
        ij = s * self.v[0, 0] * self.v[1, 0]
        ik = s * self.v[0, 0] * self.v[2, 0]
        jk = s * self.v[1, 0] * self.v[2, 0]
        ri = s * self.w * self.v[0, 0]
        rj = s * self.w * self.v[1, 0]
        rk = s * self.w * self.v[2, 0]

        ii *= s
        jj *= s
        kk *= s

        return np.matrix([[1.0 - (jj + kk),        ij - rk,   	 ik + rj],
                          [ij + rk, 1.0 - (ii + kk),   	 jk - ri],
                          [ik - rj,        jk + ri, 1.0 - (ii + jj)]])

    def __str__(self):
        return str(self.w) + " " + str(self.v)

    def toEuler(self):
        sqrq1 = self.v[0]**2
        sqrq2 = self.v[1]**2
        sqrq3 = self.v[2]**2
        scale = sqrq1 + sqrq2 + sqrq3 + self.w**2
        if scale < 1e-16:
            scale = 1

        return np.matrix([[(np.arctan2(2.0*(self.w*self.v[0] + self.v[1]*self.v[2]), 1.0 - 2.0*(sqrq1 + sqrq2)))[0, 0]],
                          [(np.arcsin(2.0*(self.w*self.v[1] - self.v[2]*self.v[0]) / scale))[0, 0]],
                          [(np.arctan2(2.0*(self.w*self.v[2] + self.v[0]*self.v[1]), 1.0 - 2.0*(sqrq2 + sqrq3)))[0, 0]]])

def sort_tuple(tup, num_item=0):
    tup.sort(key=lambda x: x[num_item])
    return tup

def main():
    # create measurement by passing .hdf5 file or ecal measurement folder
    if len(sys.argv) < 3:
        print("Usage", sys.argv[0], " <path_to_ecal_meas> <path_to_rpg_result_folder>")
        return -1

    measurement = Measurement(sys.argv[1])
    rpgfolder = sys.argv[2]

    # print("All channels containted in the measurement: {}".format(measurement.channel_names))

    selected_channels=[]
    selected_channels.append(RT4000)

    all_msgs = []
    print("selected channels:")
    for channel in measurement.channel_names:
        if channel in selected_channels:
            print("    {}".format(channel))
            channel_content = measurement[channel]

            # Iterate through all messages in a channel, iterator will return a tuple (timestamp, message)
            for (timestamp, message) in channel_content:
                all_msgs.append([timestamp, message, channel])

    all_msgs = sort_tuple(all_msgs)

    # create start position and attitude and timestamp
    first_ts, first_msg, first_channel = all_msgs[0]

    lat_start, lon_start, alt_start = first_msg.lat * deg2rad, first_msg.lon*deg2rad, first_msg.altitude
    latlonaltB = np.array([lat_start, lon_start, alt_start])
    roll_start, pitch_start, yaw_start = first_msg.roll_angle * deg2rad, first_msg.pitch_angle*deg2rad, -first_msg.yaw_angle*deg2rad + np.pi/2
    q_start = euler_to_Quaternion(roll_start, pitch_start, yaw_start)
    q_start_inverse = q_start.inverse()

    outfile_name = os.path.join(rpgfolder, "stamped_groundtruth.txt")
    print("trying to create folder:", rpgfolder)
    if not os.path.exists(rpgfolder):
        os.makedirs(rpgfolder)

    # print("len all msgs", len(all_msgs))
    with open(outfile_name, mode='w') as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=' ')

        rt_counter = 0
        msgs_written = 0
        take_every_nth_message = 7
        for ts, msg, channel in all_msgs:
            if channel == RT4000:
                if rt_counter % take_every_nth_message == 0:
                    lat, lon, alt = msg.lat * deg2rad, msg.lon * deg2rad, msg.altitude
                    latlonaltA = np.array([lat, lon, alt])
                    xyz_delta = pos_boxminus(latlonaltA, latlonaltB)
                    xyz_delta = q_start_inverse * xyz_delta # rotated position vector

                    roll, pitch, yaw = msg.roll_angle * deg2rad, msg.pitch_angle * deg2rad, -msg.yaw_angle * deg2rad + np.pi/2
                    q = euler_to_Quaternion(roll, pitch, yaw)
                    q_delta = q * q_start_inverse

                    # format according to https://github.com/uzh-rpg/rpg_trajectory_evaluation
                    # timestamp tx ty tz qx qy qz qw
                    ts = str(ts)
                    ts = ts[:-6] + "." + ts[-6:]
                    csv_writer.writerow([ts, xyz_delta[0,0], xyz_delta[1,0], xyz_delta[2,0],
                                        q_delta.v[0, 0], q_delta.v[1, 0], q_delta.v[2, 0], q_delta.w])
                    msgs_written+=1
                    # print([ts, xyz_delta[0,0], xyz_delta[1,0], xyz_delta[2,0],
                    #                     q_delta.v[0, 0], q_delta.v[1, 0], q_delta.v[2, 0], q_delta.w])


                rt_counter += 1

    print("total RT4000 msgs {}, ground truth poses {} written to \"{}\"".format(len(all_msgs), msgs_written, outfile_name))


if __name__ == "__main__":
   main()
