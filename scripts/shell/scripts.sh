# /home/gy/DAGSfM/build/src/exe/colmap mapper \
# /16t/gy/Data1/SFM/tankandtemples/training/Courthouse/ColmapOutput_full_matches_thresh_50 \
# --database_path=/16t/gy/Data1/SFM/tankandtemples/training/Courthouse/database.db \
# --image_path=/16t/gy/Data1/SFM/tankandtemples/training/Courthouse/images \
# --output_path=/16t/gy/Data1/SFM/tankandtemples/training/Courthouse/ColmapOutput_full_matches_thresh_50 \
# --Mapper.num_threads=8 \
# --Mapper.min_num_matches=50


# mkdir -p /16t/gy/Data1/SFM/tankandtemples/training/Courthouse/gsfm_metric_1105_v14.5_nofilter50_test/seq_results

# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data1/SFM/tankandtemples/training/Courthouse \
# 100 \
# gsfm_metric_1105_v14.5_nofilter50_test \
# mylog61_0_1105/MetricResults_0_1105.txt


# mkdir -p /16t/gy/Data/SFM/hkust/data_ambiguous/books/brute/gsfm_metric_1merge_subseq_21_v14.4.2_kmeans/seq_results/

# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data/SFM/hkust/data_ambiguous/books/brute/ \
# 100 \
# gsfm_metric_1105_v14.5_nofilter50_test \
# mylog61_0_21/MetricResults_0_21.txt


# bash /home/gy/DAGSfM/scripts/shell/extract_feature.sh \
# /16t/gy/Data1/SFM/tankandtemples/new/Courthouse

# bash /home/gy/DAGSfM/scripts/shell/exhaustive_matcher.sh \
# /16t/gy/Data1/SFM/tankandtemples/new/Courthouse

# bash /home/gy/DAGSfM/scripts/shell/extract_feature.sh \
# /16t/gy/Data1/SFM/tankandtemples/new/Palace

# bash /home/gy/DAGSfM/scripts/shell/exhaustive_matcher.sh \
# /16t/gy/Data1/SFM/tankandtemples/new/Palace


# bash /home/gy/DAGSfM/scripts/shell/distributed_sfm.sh \
# /16t/gy/Data1/SFM/tankandtemples/new/Courthouse \
# 50 \
# mylog61_0_1105 \
# 0 \
# 1105 \
# 0

# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data1/SFM/tankandtemples/new/Courthouse \
# 100 \
# gsfm_metric_1105_v14.5_nofilter50 \
# mylog61_0_1105/MetricResults_0_1105.txt


# bash /home/gy/DAGSfM/scripts/shell/distributed_sfm.sh \
# /16t/gy/Data1/SFM/tankandtemples/new/Palace \
# 50 \
# mylog61_0_3649 \
# 0 \
# 3649 \
# 0

# /home/gy/DAGSfM/build/src/exe/colmap feature_extractor \
# --database_path=/16t/gy/Data1/SFM/tankandtemples/new/wenku/database.db \
# --image_path=/16t/gy/Data1/SFM/tankandtemples/new/wenku/images \
# --SiftExtraction.num_threads=8 \
# --SiftExtraction.use_gpu=1 \
# --SiftExtraction.gpu_index=0 \
# --ImageReader.camera_model=SIMPLE_RADIAL \ #OPENCV \
# --ImageReader.camera_params=1068.0,949.2,505.5,-0.05309 #1068.18998,1066.93406,949.18750,505.48301,-0.05309,0.02165,-0.00021,-0.00168

# bash /home/gy/DAGSfM/scripts/shell/exhaustive_matcher.sh \
# /16t/gy/Data1/SFM/tankandtemples/new/wenku/


# bash /home/gy/DAGSfM/scripts/shell/distributed_sfm.sh \
# /16t/gy/Data1/SFM/tankandtemples/new/wenku \
# 50 \
# mylog61_0_500 \
# 0 \
# 500 \
# 0

bash /home/gy/DAGSfM_merge_v15/scripts/shell/distributed_sfm_subcluster.sh \
/16t/gy/Data1/SFM/tankandtemples/new/wenku \
100 \
gsfm_metric_500_v15.2_nofilter50_t15 \
mylog61_0_500/MetricResults_0_500.txt

bash /home/gy/DAGSfM_merge_v15/scripts/shell/distributed_sfm_subcluster.sh \
/16t/gy/Data1/SFM/tankandtemples/new/wenku \
100 \
gsfm_metric_2600_v15.2_nofilter50_t15 \
mylog61_0_2600/MetricResults_0_2600.txt


# bash /home/gy/DAGSfM/scripts/shell/my_colmap_sfm_brute_full.sh \
# /16t/gy/Data/SFM/duplicate/church/ \
# 50 \
# mylog61_0.25_0 \
# 0.25 \
# 0 \
# 0 \
# /16t/gy/Data/SFM/duplicate/church/ColmapOutput_v61_0.25_0 \
# /16t/gy/Data/SFM/duplicate/church/ColmapAddimages_v61_0.25_0

# bash /home/gy/DAGSfM/scripts/shell/my_colmap_sfm_brute_full.sh \
# /16t/gy/Data/SFM/duplicate/arc_de_triomphe/ \
# 50 \
# mylog61_0.25_0 \
# 0.25 \
# 0 \
# 0 \
# /16t/gy/Data/SFM/duplicate/arc_de_triomphe/ColmapOutput_v61_0.25_0 \
# /16t/gy/Data/SFM/duplicate/arc_de_triomphe/ColmapAddimages_v61_0.25_0

# bash /home/gy/DAGSfM/scripts/shell/my_colmap_sfm_brute_full.sh \
# /16t/gy/Data/SFM/duplicate/berliner_dom/ \
# 50 \
# mylog61_0.25_0 \
# 0.25 \
# 0 \
# 0 \
# /16t/gy/Data/SFM/duplicate/berliner_dom/ColmapOutput_v61_0.25_0 \
# /16t/gy/Data/SFM/duplicate/berliner_dom/ColmapAddimages_v61_0.25_0


# bash /home/gy/DAGSfM/scripts/shell/my_colmap_sfm_brute_full.sh \
# /16t/gy/Data/SFM/duplicate/big_ben/ \
# 50 \
# mylog61_0.25_0 \
# 0.25 \
# 0 \
# 0 \
# /16t/gy/Data/SFM/duplicate/big_ben/ColmapOutput_v61_0.25_0 \
# /16t/gy/Data/SFM/duplicate/big_ben/ColmapAddimages_v61_0.25_0


