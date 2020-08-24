#!/bin/bash

koeln_image_paths=(
    #runde 1
    /data/datasets/conti/20191021_Koeln_lang/Runde01/dw_2019_10_21_03_24_23_000000_192.0.0.112_f3a53a6a-f3ec-11e9-ac60-00044babffd2
    /data/datasets/conti/20191021_Koeln_lang/Runde01/dw_2019_10_21_03_26_36_000000_192.0.0.112_42e0a718-f3ed-11e9-ac60-00044babffd2
    /data/datasets/conti/20191021_Koeln_lang/Runde01/dw_2019_10_21_03_28_45_000000_192.0.0.112_8fc2d060-f3ed-11e9-ac60-00044babffd2
    /data/datasets/conti/20191021_Koeln_lang/Runde01/dw_2019_10_21_03_35_37_000000_192.0.0.112_858e465a-f3ee-11e9-ac60-00044babffd2
    /data/datasets/conti/20191021_Koeln_lang/Runde01/dw_2019_10_21_03_44_38_000000_192.0.0.112_c7d55606-f3ef-11e9-ac60-00044babffd2
    # /data/datasets/conti/20191021_Koeln_lang/Runde01/dw_2019_10_21_03_45_30_000000_192.0.0.112_e6c9957c-f3ef-11e9-ac60-00044babffd2
    # runde 2
    /data/datasets/conti/20191021_Koeln_lang/Runde02/dw_2019_10_21_04_23_34_000000_192.0.0.112_3847d6e8-f3f5-11e9-ac60-00044babffd2
    /data/datasets/conti/20191021_Koeln_lang/Runde02/dw_2019_10_21_04_25_22_000000_192.0.0.112_784ce7f6-f3f5-11e9-ac60-00044babffd2
    /data/datasets/conti/20191021_Koeln_lang/Runde02/dw_2019_10_21_04_32_19_000000_192.0.0.112_714c5954-f3f6-11e9-ac60-00044babffd2
    /data/datasets/conti/20191021_Koeln_lang/Runde02/dw_2019_10_21_04_34_07_000000_192.0.0.112_b14d3456-f3f6-11e9-ac60-00044babffd2
    /data/datasets/conti/20191021_Koeln_lang/Runde02/dw_2019_10_21_04_40_41_000000_192.0.0.112_9c3a019c-f3f7-11e9-ac60-00044babffd2
    /data/datasets/conti/20191021_Koeln_lang/Runde02/dw_2019_10_21_04_47_37_000000_192.0.0.112_940c98da-f3f8-11e9-ac60-00044babffd2
    # runde 3
    /data/datasets/conti/20191021_Koeln_lang/Runde03/dw_2019_10_21_07_39_08_000000_192.0.0.112_8a073bac-f410-11e9-837b-00044babffd2
    /data/datasets/conti/20191021_Koeln_lang/Runde03/dw_2019_10_21_07_48_06_000000_192.0.0.112_ca82a3e6-f411-11e9-ae39-00044babffd2
    /data/datasets/conti/20191021_Koeln_lang/Runde03/dw_2019_10_21_07_55_50_000000_192.0.0.112_df8566e2-f412-11e9-ae39-00044babffd2
    #runde 4
    /data/datasets/conti/20191021_Koeln_lang/Runde04/dw_2019_10_21_08_04_35_000000_192.0.0.112_188191fe-f414-11e9-ae39-00044babffd2
    /data/datasets/conti/20191021_Koeln_lang/Runde04/dw_2019_10_21_08_14_00_000000_192.0.0.112_68d6832a-f415-11e9-ae39-00044babffd2
    /data/datasets/conti/20191021_Koeln_lang/Runde04/dw_2019_10_21_08_26_32_000000_192.0.0.112_291a0f7a-f417-11e9-ae39-00044babffd2
    #runde 5
    /data/datasets/conti/20191021_Koeln_lang/Runde05/dw_2019_10_21_08_33_57_000000_192.0.0.112_32a07f74-f418-11e9-ae39-00044babffd2
    /data/datasets/conti/20191021_Koeln_lang/Runde05/dw_2019_10_21_08_41_58_000000_192.0.0.112_50fe8046-f419-11e9-ae39-00044babffd2
    /data/datasets/conti/20191021_Koeln_lang/Runde05/dw_2019_10_21_08_54_18_000000_192.0.0.112_0a050c9e-f41b-11e9-ae39-00044babffd2
    /data/datasets/conti/20191021_Koeln_lang/Runde05/dw_2019_10_21_08_58_28_000000_192.0.0.112_9f825088-f41b-11e9-ae39-00044babffd2
    #runde 6
    /data/datasets/conti/20191021_Koeln_lang/Runde06/dw_2019_10_21_09_08_42_000000_192.0.0.112_0d5c815e-f41d-11e9-ae39-00044babffd2
    /data/datasets/conti/20191021_Koeln_lang/Runde06/dw_2019_10_21_09_19_08_000000_192.0.0.112_822b31f0-f41e-11e9-ae39-00044babffd2
    # /data/datasets/conti/20191021_Koeln_lang/Runde06/dw_2019_10_21_09_28_41_000000_192.0.0.112_d830471a-f41f-11e9-ae39-00044babffd2
)
koeln_adl_files=(
    # runde 1 
    koeln_2019_10_21_10_10_47
    koeln_2019_10_21_10_25_06
    koeln_2019_10_21_10_27_15
    koeln_2019_10_21_10_34_08
    koeln_2019_10_21_10_43_56
    # koeln_2019_10_21_11_01_26 video starts way before ecal measurement
    # runde 2
    koeln_2019_10_21_11_21_54
    koeln_2019_10_21_11_23_48
    koeln_2019_10_21_11_30_50
    koeln_2019_10_21_11_32_32
    koeln_2019_10_21_11_39_02
    koeln_2019_10_21_11_46_03
    # runde 3
    koeln_2019_10_21_14_27_49 # rot_dist fails investigate
    koeln_2019_10_21_14_45_55 # rot_dist fails investigate
    koeln_2019_10_21_14_54_22
    # runde 4
    koeln_2019_10_21_15_03_07 # good rot_dist
    koeln_2019_10_21_15_12_31
    koeln_2019_10_21_15_25_03 # good rot_dist
    # runde 5
    koeln_2019_10_21_15_32_29 # rot_dist completely fails, investigat
    koeln_2019_10_21_15_40_29
    koeln_2019_10_21_15_52_50
    koeln_2019_10_21_15_57_00
    # runde 6
    koeln_2019_10_21_16_07_16 # rot_dist fails, investigate
    koeln_2019_10_21_16_17_46
    # koeln_2019_10_21_16_27_13  no ecal measurement data
)

wuppertal_image_paths=(
    # runde 1
    /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde01/dw_2019_10_24_06_54_46_000000_192.0.0.112_d67dca62-f665-11e9-988a-00044babffd2
    /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde01/dw_2019_10_24_06_56_34_000000_192.0.0.112_1710becc-f666-11e9-988a-00044babffd2
    /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde01/dw_2019_10_24_07_06_56_000000_192.0.0.112_89a06018-f667-11e9-988a-00044babffd2
    # runde 2
    /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde02/dw_2019_10_24_07_16_34_000000_192.0.0.112_e26f55ae-f668-11e9-988a-00044babffd2
    /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde02/dw_2019_10_24_07_18_01_000000_192.0.0.112_15e29fa4-f669-11e9-988a-00044babffd2
    /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde02/dw_2019_10_24_07_23_26_000000_192.0.0.112_d7bd006a-f669-11e9-988a-00044babffd2
    # runde 3
    /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde03/dw_2019_10_24_07_29_44_000000_192.0.0.112_b968f500-f66a-11e9-988a-00044babffd2
    /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde03/dw_2019_10_24_07_31_21_000000_192.0.0.112_f2fa93fa-f66a-11e9-988a-00044babffd2
    /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde03/dw_2019_10_24_07_37_02_000000_192.0.0.112_be40fd24-f66b-11e9-988a-00044babffd2
    # runde 4
    /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde04/dw_2019_10_24_07_42_38_000000_192.0.0.112_8687e5f4-f66c-11e9-988a-00044babffd2
    /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde04/dw_2019_10_24_07_47_18_000000_192.0.0.112_2d515082-f66d-11e9-988a-00044babffd2
    /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde04/dw_2019_10_24_07_58_25_000000_192.0.0.112_bb265776-f66e-11e9-988a-00044babffd2
    # runde 5
    /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde05/dw_2019_10_24_08_02_40_000000_192.0.0.112_52b3766e-f66f-11e9-988a-00044babffd2
    /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde05/dw_2019_10_24_08_05_57_000000_192.0.0.112_c899284c-f66f-11e9-988a-00044babffd2
    /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde05/dw_2019_10_24_08_10_56_000000_192.0.0.112_7a76fb98-f670-11e9-988a-00044babffd2
    # runde 6
    /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde06/dw_2019_10_24_08_16_27_000000_192.0.0.112_3fcf9db4-f671-11e9-988a-00044babffd2
    /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde06/dw_2019_10_24_08_19_11_000000_192.0.0.112_a17af4e6-f671-11e9-988a-00044babffd2
    /data/datasets/conti/atCity_wuppertal/20191024_Wuppertal_Grifflenberg/Runde06/dw_2019_10_24_08_46_41_000000_192.0.0.112_78db5568-f675-11e9-988a-00044babffd2
)

wuppertal_adl_files=(
    # runde 1
    wuppertal_2019_10_24_13_53_17
    wuppertal_2019_10_24_13_55_05
    wuppertal_2019_10_24_14_05_26
    # runde 2
    wuppertal_2019_10_24_14_15_05
    wuppertal_2019_10_24_14_16_31
    wuppertal_2019_10_24_14_21_56
    # runde 3
    wuppertal_2019_10_24_14_28_15
    wuppertal_2019_10_24_14_29_47
    wuppertal_2019_10_24_14_35_34
    # runde 4
    wuppertal_2019_10_24_14_41_09
    wuppertal_2019_10_24_14_45_44
    wuppertal_2019_10_24_14_56_57
    # runde 5
    wuppertal_2019_10_24_15_01_12
    wuppertal_2019_10_24_15_04_29
    wuppertal_2019_10_24_15_09_28
    #runde 6
    wuppertal_2019_10_24_15_14_57
    wuppertal_2019_10_24_15_17_41
    wuppertal_2019_10_24_15_45_10
)

BIN="/home/serov/code/cpp/dso/build/bin/dso_dataset"
calib="/home/serov/code/cpp/dso/calib/calib_prerectified_koeln.txt"
rpg_base="/home/serov/code/python/rpg_trajectory_evaluation/results/conti"
preset="2"
mode="1"
quiet="1"
platform="conti"
algo="dso_fast"
index=0
for index in ${!koeln_image_paths[*]};
do
    image_path="${koeln_image_paths[$index]}"
    drive="${koeln_adl_files[$index]}"
    cmd=${BIN}
    cmd+=" files=${image_path}/60fov_rect/"
    cmd+=" calib=${calib}"
    cmd+=" preset=${preset}"
    cmd+=" mode=${mode}"
    cmd+=" quiet=${quiet}"
    cmd+=" rpg=${rpg_base}/${algo}/${platform}_${algo}_${drive}/stamped_traj_estimate.txt"

    echo $cmd
    $cmd
done

# files=/data/datasets/conti/20191021_Koeln_lang/Runde02/dw_2019_10_21_04_25_22_000000_192.0.0.112_784ce7f6-f3f5-11e9-ac60-00044babffd2/60fov_rect 
# calib=./calib/calib_prerectified_koeln.txt 
# preset=2 
# mode=1 
# quiet=1
# rpg=/data/datasets/test/test/stamped_traj_estimate.txt 


