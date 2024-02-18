# mkdir -p /16t/gy/Data/SFM/hkust/temple/brute/gsfm_ncut_seq/seq_results

# bash /home/gy/DAGSfM_sequential_merging/scripts/shell/distributed_sfm_NCUT.sh \
# /16t/gy/Data/SFM/hkust/temple/brute/ \
# 100 \
# gsfm_ncut_seq


# bash /home/gy/DAGSfM/scripts/shell/extract_feature.sh \
# /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/raw/

# bash /home/gy/DAGSfM/scripts/shell/vocabulary_tree_matcher.sh \
# /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/raw/ \
# /home/gy/DAGSfM/vocab_tree_flickr100K_words32K.bin


# bash /home/gy/DAGSfM/scripts/shell/distributed_sfm.sh \
# /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/raw/ \
# 50 \
# mylog63_0_500 \
# 0 \
# 500 \
# 0



# mkdir -p /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/raw/gsfm_ncut_subseq/seq_results

# bash /home/gy/DAGSfM_merge_v3/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/raw/ \
# 100 \
# gsfm_ncut_subseq \
# mylog63_0_500/MetricResults_0_500.txt


# mkdir -p /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/raw/gsfm_ncut_seq2/seq_results

# bash /home/gy/DAGSfM_sequential_merging/scripts/shell/distributed_sfm_NCUT.sh \
# /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/raw/ \
# 100 \
# gsfm_ncut_seq2



# bash /home/gy/DAGSfM/scripts/shell/distributed_sfm.sh \
# /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/ \
# 50 \
# mylog63_0.35_0 \
# 0.35 \
# 0 \
# 0

# mkdir -p /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/gsfm_ncut_subseq5_aeai_ub30_0.7/seq_results

# bash /home/gy/DAGSfM_merge_v3/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/ \
# 100 \
# gsfm_ncut_subseq5_aeai_ub30_0.7 \
# mylog63_0_6514/MetricResults_0_6514.txt

# mkdir -p /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/gsfm_metric_1merge_subseq_6514_3/seq_results

# bash /home/gy/DAGSfM_merge_v3/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/ \
# 100 \
# gsfm_metric_1merge_subseq_6514_3 \
# mylog63_0_6514/MetricResults_0_6514.txt

# mkdir -p /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/gsfm_metric_1merge_subseq_6514_v15_kmeans_100/seq_results

# bash /home/gy/DAGSfM_merge_v13/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/ \
# 100 \
# gsfm_metric_1merge_subseq_6514_v15_kmeans_100 \
# mylog63_0_6514/MetricResults_0_6514.txt


# mkdir -p /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/gsfm_metric_1merge_subseq_6514_v15.1_kmeans_100/seq_results

# bash /home/gy/DAGSfM_merge_v13/scripts/shell/distributed_sfm_subcluster_id.sh \
# /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/ \
# 100 \
# gsfm_metric_1merge_subseq_6514_v15.1_kmeans_100 \
# mylog63_0_6514/MetricResults_0_6514.txt \
# 5



bash /home/gy/DAGSfM/scripts/shell/extract_feature.sh \
/16t/gy/Data1/SFM/tankandtemples/new/all_Palace/right_exif/raw

bash /home/gy/DAGSfM/scripts/shell/exhaustive_matcher.sh \
/16t/gy/Data1/SFM/tankandtemples/new/all_Palace/right_exif/raw

bash /home/gy/DAGSfM/scripts/shell/distributed_sfm.sh \
/16t/gy/Data1/SFM/tankandtemples/new/all_Palace/right_exif/raw \
50 \
mylog61_0_730 \
0 \
730 \
0

bash /home/gy/DAGSfM_merge_v15/scripts/shell/distributed_sfm_subcluster.sh \
/16t/gy/Data1/SFM/tankandtemples/new/all_Palace/right_exif/raw \
100 \
gsfm_metric_730_v15.2_nofilter50_t15 \
mylog61_0_730/MetricResults_0_730.txt


bash /home/gy/DAGSfM/scripts/shell/extract_feature.sh \
/16t/gy/Data1/SFM/tankandtemples/new/all_Palace/right_exif/fps5

bash /home/gy/DAGSfM/scripts/shell/exhaustive_matcher.sh \
/16t/gy/Data1/SFM/tankandtemples/new/all_Palace/right_exif/fps5

bash /home/gy/DAGSfM/scripts/shell/distributed_sfm.sh \
/16t/gy/Data1/SFM/tankandtemples/new/all_Palace/right_exif/fps5 \
50 \
mylog61_0_3649 \
0 \
3649 \
0

bash /home/gy/DAGSfM_merge_v15/scripts/shell/distributed_sfm_subcluster.sh \
/16t/gy/Data1/SFM/tankandtemples/new/all_Palace/right_exif/fps5 \
100 \
gsfm_metric_3649_v15.2_nofilter50_t15 \
mylog61_0_3649/MetricResults_0_3649.txt




# bash /home/gy/DAGSfM/scripts/shell/extract_feature.sh \
# /16t/gy/Data/SFM/My/wenku/

# bash /home/gy/DAGSfM/scripts/shell/exhaustive_matcher.sh \
# /16t/gy/Data/SFM/My/wenku/


# bash /home/gy/DAGSfM/scripts/shell/distributed_sfm.sh \
# /16t/gy/Data/SFM/My/wenku/ \
# 50 \
# mylog61_0_500 \
# 0 \
# 500 \
# 0

# mkdir -p /16t/gy/Data/SFM/My/wenku/gsfm_metric_1merge_subseq_2600_v14.4.2_kmeans/seq_results

# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data/SFM/My/wenku/ \
# 100 \
# gsfm_metric_1merge_subseq_2600_v14.4.2_kmeans \
# mylog61_0_2600/MetricResults_0_2600.txt


# bash /home/gy/DAGSfM/scripts/shell/distributed_sfm.sh \
# /16t/gy/Data/SFM/hkust/data_ambiguous/books/brute/ \
# 50 \
# mylog61_0_21 \
# 0 \
# 21 \
# 0

# mkdir -p /16t/gy/Data/SFM/hkust/data_ambiguous/books/brute/gsfm_metric_1merge_subseq_21_v14.4.2_kmeans/seq_results/

# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data/SFM/hkust/data_ambiguous/books/brute/ \
# 100 \
# gsfm_metric_1merge_subseq_21_v14.4.2_kmeans \
# mylog61_0_21/MetricResults_0_21.txt


# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data/SFM/My/wenku/ \
# 100 \
# gsfm_metric_1merge_subseq_2600_v14.4_kmeans \
# mylog61_0_2600/MetricResults_0_2600.txt


# # mkdir -p /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/gsfm_metric_1merge_subseq_6514_v14.3_kmeans/seq_results

# bash /home/gy/DAGSfM/scripts/shell/distributed_sfm.sh \
# /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/ \
# 50 \
# mylog61_0_6514 \
# 0 \
# 6514 \
# 0


# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/ \
# 100 \
# gsfm_metric_1merge_subseq_6514_v14.4.1_kmeans_100 \
# mylog61_0_6514/MetricResults_0_6514.txt



# bash /home/gy/DAGSfM/scripts/shell/colmap_sfm_brute.sh \
# /16t/gy/Data1/SFM/tankandtemples/training/Courthouse \
# /16t/gy/Data1/SFM/tankandtemples/training/Courthouse/ColmapOutput_full

# bash /home/gy/DAGSfM/scripts/shell/distributed_sfm.sh \
# /16t/gy/Data1/SFM/tankandtemples/training/Courthouse \
# 50 \
# mylog61_0_1105 \
# 0 \
# 1105 \
# 0

# mkdir -p /16t/gy/Data1/SFM/tankandtemples/training/Courthouse/gsfm_metric_1merge_subseq_1105_v14.4_kmeans_100/seq_results

# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data1/SFM/tankandtemples/training/Courthouse \
# 100 \
# gsfm_metric_1merge_subseq_1105_v14.4_kmeans_100 \
# mylog61_0_1105/MetricResults_0_1105.txt

# mkdir -p /16t/gy/Data1/SFM/tankandtemples/training/Courthouse/gsfm_metric_1merge_subseq_1105_v14.4_kmeans_100_alledges/seq_results

# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data1/SFM/tankandtemples/training/Courthouse \
# 100 \
# gsfm_metric_1merge_subseq_1105_v14.4_kmeans_100_alledges \
# mylog61_0_1105/MetricResults_0_1105.txt


# /home/gy/DAGSfM_merge_v14/build/src/exe/colmap mapper \
# /16t/gy/Data1/SFM/tankandtemples/training/Courthouse/ColmapOutput_full_DAG \
# --database_path=/16t/gy/Data1/SFM/tankandtemples/training/Courthouse/database.db \
# --image_path=/16t/gy/Data1/SFM/tankandtemples/training/Courthouse/images \
# --output_path=/16t/gy/Data1/SFM/tankandtemples/training/Courthouse/ColmapOutput_full_DAG \
# --Mapper.num_threads=8


# /home/gy/DAGSfM/build/src/exe/colmap mapper \
# /16t/gy/Data1/SFM/tankandtemples/training/Courthouse/ColmapOutput_full_matches_thresh_50 \
# --database_path=/16t/gy/Data1/SFM/tankandtemples/training/Courthouse/database.db \
# --image_path=/16t/gy/Data1/SFM/tankandtemples/training/Courthouse/images \
# --output_path=/16t/gy/Data1/SFM/tankandtemples/training/Courthouse/ColmapOutput_full_matches_thresh_50 \
# --Mapper.num_threads=8 \
# --Mapper.min_num_matches=50


# bash /home/gy/DAGSfM/scripts/shell/my_colmap_sfm_vocab_full_nofeatures.sh \
# /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/raw \
# 50 \
# mylog61_0_500 \
# 0 \
# 500 \
# 0 \
# /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/raw/ColmapOutput_v61_0_500 \
# /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/raw/ColmapAddimages_v61_0_500










# bash /home/gy/DAGSfM/scripts/shell/my_colmap_sfm_vocab_full_nofeatures.sh \
# /16t/gy/Data1/SFM/tankandtemples/training/Courthouse/ \
# 50 \
# mylog61_0_500 \
# 0 \
# 500 \
# 0 \
# /16t/gy/Data1/SFM/tankandtemples/training/Courthouse/ColmapOutput_v61_0_500 \
# /16t/gy/Data1/SFM/tankandtemples/training/Courthouse/ColmapAddimages_v61_0_500

# mkdir -p /16t/gy/Data/SFM/My/wenku/gsfm_metric_1merge_subseq_2600_v14.4.1_kmeans/seq_results

# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data/SFM/My/wenku/ \
# 100 \
# gsfm_metric_1merge_subseq_2600_v14.4.1_kmeans \
# mylog61_0_500/MetricResults_0_500.txt


# /home/gy/DAGSfM/build/src/exe/colmap mapper \
# /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/FrameRetrives/fps5/ColmapOutput_full_matches_thresh_200 \
# --database_path=/16t/gy/Data1/SFM/tankandtemples/advanced/Palace/FrameRetrives/fps5/database.db \
# --image_path=/16t/gy/Data1/SFM/tankandtemples/advanced/Palace/FrameRetrives/fps5/images \
# --output_path=/16t/gy/Data1/SFM/tankandtemples/advanced/Palace/FrameRetrives/fps5/ColmapOutput_full_matches_thresh_200 \
# --Mapper.num_threads=8 \
# --Mapper.min_num_matches=200


# mkdir -p /16t/gy/Data1/SFM/tankandtemples/training/Meetingroom/test_simudata_random_sigma2.0/seq_results

# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data1/SFM/tankandtemples/training/Meetingroom \
# 100 \
# test_simudata_random_sigma2.0 \
# mylog61_0_370/MetricResults_0_370.txt

# bash /home/gy/DAGSfM/scripts/shell/colmap_sfm_brute.sh \
# /16t/gy/Data1/SFM/tankandtemples/training/Meetingroom \
# /16t/gy/Data1/SFM/tankandtemples/training/Meetingroom/ColmapOutput_full

# bash /home/gy/DAGSfM/scripts/shell/distributed_sfm.sh \
# /16t/gy/Data1/SFM/tankandtemples/training/Meetingroom \
# 50 \
# mylog61_0_370 \
# 0 \
# 370 \
# 0

# mkdir -p /16t/gy/Data1/SFM/tankandtemples/training/Meetingroom/gsfm_metric_1merge_subseq_370_v14.4_kmeans_100/seq_results

# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data1/SFM/tankandtemples/training/Meetingroom \
# 100 \
# gsfm_metric_1merge_subseq_370_v14.4_kmeans_100 \
# mylog61_0_370/MetricResults_0_370.txt



# bash /home/gy/DAGSfM/scripts/shell/distributed_sfm.sh \
# /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/FrameRetrives/fps5/ \
# 50 \
# mylog61_0_3649 \
# 0 \
# 3649 \
# 0

# mkdir -p /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/FrameRetrives/fps5/gsfm_metric_3649_v14.5_kmeans_mst_nofilter50/seq_results

# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/FrameRetrives/fps5/ \
# 100 \
# gsfm_metric_3649_v14.5_kmeans_mst_nofilter50 \
# mylog61_0_3649/MetricResults_0_3649.txt

# # mkdir -p /16t/gy/Data/SFM/hkust/data_ambiguous/books/brute/gsfm_metric_1merge_subseq_21_v14.2_kmeans_10/seq_results

# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data/SFM/hkust/data_ambiguous/books/brute/ \
# 100 \
# gsfm_metric_1merge_subseq_21_v14.4_kmeans_10 \
# mylog61_0_21/MetricResults_0_21.txt


# # # bash /home/gy/DAGSfM/scripts/shell/distributed_sfm.sh \
# # # /16t/gy/Data/SFM/hkust/data_ambiguous/cereal/brute/ \
# # # 50 \
# # # mylog61_0_25 \
# # # 0 \
# # # 25 \
# # # 0

# # mkdir -p /16t/gy/Data/SFM/hkust/data_ambiguous/cereal/brute/gsfm_metric_1merge_subseq_25_v14.2_kmeans/seq_results

# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data/SFM/hkust/data_ambiguous/cereal/brute/ \
# 100 \
# gsfm_metric_1merge_subseq_25_v14.4_kmeans \
# mylog61_0_25/MetricResults_0_25.txt


# # # bash /home/gy/DAGSfM/scripts/shell/distributed_sfm.sh \
# # # /16t/gy/Data/SFM/hkust/data_ambiguous/cup/brute/ \
# # # 50 \
# # # mylog61_0_64 \
# # # 0 \
# # # 64 \
# # # 0

# # mkdir -p /16t/gy/Data/SFM/hkust/data_ambiguous/cup/brute/gsfm_metric_1merge_subseq_64_v14.2_kmeans/seq_results

# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data/SFM/hkust/data_ambiguous/cup/brute/ \
# 100 \
# gsfm_metric_1merge_subseq_64_v14.4_kmeans \
# mylog61_0_64/MetricResults_0_64.txt


# # # bash /home/gy/DAGSfM/scripts/shell/distributed_sfm.sh \
# # # /16t/gy/Data/SFM/hkust/data_ambiguous/desk/brute/ \
# # # 50 \
# # # mylog61_0_31 \
# # # 0 \
# # # 31 \
# # # 0

# # mkdir -p /16t/gy/Data/SFM/hkust/data_ambiguous/desk/brute/gsfm_metric_1merge_subseq_31_v14.2_kmeans/seq_results

# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data/SFM/hkust/data_ambiguous/desk/brute/ \
# 100 \
# gsfm_metric_1merge_subseq_31_v14.4_kmeans \
# mylog61_0_31/MetricResults_0_31.txt


# # # bash /home/gy/DAGSfM/scripts/shell/distributed_sfm.sh \
# # # /16t/gy/Data/SFM/hkust/data_ambiguous/indoor/brute/ \
# # # 50 \
# # # mylog61_0_153 \
# # # 0 \
# # # 153 \
# # # 0

# # mkdir -p /16t/gy/Data/SFM/hkust/data_ambiguous/indoor/brute/gsfm_metric_1merge_subseq_153_v14.2_kmeans/seq_results

# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data/SFM/hkust/data_ambiguous/indoor/brute/ \
# 100 \
# gsfm_metric_1merge_subseq_153_v14.4_kmeans \
# mylog61_0_153/MetricResults_0_153.txt


# # mkdir -p /16t/gy/Data/SFM/hkust/data_ambiguous/oats/brute/gsfm_metric_1merge_subseq_23_v14.2_kmeans/seq_results

# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data/SFM/hkust/data_ambiguous/oats/brute/ \
# 100 \
# gsfm_metric_1merge_subseq_23_v14.4_kmeans \
# mylog61_0_23/MetricResults_0_23.txt




# # bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# # /16t/gy/Data/SFM/hkust/data_ambiguous/books/brute \
# # 100 \
# # gsfm_test1 \
# # mylog61_0_21/MetricResults_0_21.txt



# # # bash /home/gy/DAGSfM/scripts/shell/distributed_sfm.sh \
# # # /16t/gy/Data/SFM/hkust/data_ambiguous/street/brute/ \
# # # 50 \
# # # mylog61_0_19 \
# # # 0 \
# # # 19 \
# # # 0

# # mkdir -p /16t/gy/Data/SFM/hkust/data_ambiguous/street/brute/gsfm_metric_1merge_subseq_19_v14.3_kmeans/seq_results

# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data/SFM/hkust/data_ambiguous/street/brute/ \
# 100 \
# gsfm_metric_1merge_subseq_19_v14.4_kmeans \
# mylog61_0_19/MetricResults_0_19.txt


# # mkdir -p /16t/gy/Data/SFM/hkust/temple/brute/gsfm_metric_1merge_subseq_341_v14.2_kmeans/seq_results

# bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data/SFM/hkust/temple/brute/ \
# 100 \
# gsfm_metric_1merge_subseq_341_v14.4_kmeans \
# mylog61_0_341/MetricResults_0_341.txt






# # # mkdir -p /16t/gy/Data/SFM/hkust/data_ambiguous/books/brute/gsfm_metric_1merge_subseq_21_v14_kmeans_10/seq_results

# # # bash /home/gy/DAGSfM_merge_v14/scripts/shell/distributed_sfm_subcluster.sh \
# # # /16t/gy/Data/SFM/hkust/data_ambiguous/books/brute/ \
# # # 100 \
# # # gsfm_metric_1merge_subseq_21_v14_kmeans_10 \
# # # mylog61_0_13/MetricResults_0_13.txt

# # # mv  /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/database.db /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/database_gsfm.db
# # # mv  /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/database.db-shm /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/database_gsfm.db-shm
# # # mv  /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/database.db-wal /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/database_gsfm.db-wal

# # # bash /home/gy/DAGSfM/scripts/shell/extract_feature.sh \
# # # /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/

# # # bash /home/gy/DAGSfM/scripts/shell/exhaustive_matcher.sh \
# # # /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/

# # # cp /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/database.db /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/database_backup.db

# # # mkdir -p /16t/gy/Data1/SFM/tankandtemples/intermediate/Lighthouse/raw/gsfm_ncut_subseq_test/seq_results

# # # bash /home/gy/DAGSfM_merge_v10/scripts/shell/distributed_sfm_subcluster.sh \
# # # /16t/gy/Data1/SFM/tankandtemples/intermediate/Lighthouse/raw/ \
# # # 100 \
# # # gsfm_ncut_subseq_test \
# # # mylog63_0_309/MetricResults_0_309.txt


# # # mkdir -p /16t/gy/Data1/SFM/tankandtemples/intermediate/Lighthouse/raw/gsfm_ncut_subseq5_alledges_allimgaes/seq_results

# # # bash /home/gy/DAGSfM_merge_v3/scripts/shell/distributed_sfm_subcluster.sh \
# # # /16t/gy/Data1/SFM/tankandtemples/intermediate/Lighthouse/raw/ \
# # # 100 \
# # # gsfm_ncut_subseq5_alledges_allimgaes \
# # # mylog63_0_309/MetricResults_0_309.txt




# # mkdir -p /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/raw/gsfm_metric_1merge_subseq_500_v10_0.5_igraph/seq_results

# # bash /home/gy/DAGSfM_merge_v10/scripts/shell/distributed_sfm_subcluster.sh \
# # /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/raw/ \
# # 100 \
# # gsfm_metric_1merge_subseq_500_v10_0.5_igraph \
# # mylog63_0_500/MetricResults_0_500.txt

# # mkdir -p /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/raw/gsfm_metric_1merge_subseq_500_v10_0.7_igraph/seq_results

# # bash /home/gy/DAGSfM_merge_v10/scripts/shell/distributed_sfm_subcluster.sh \
# # /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/raw/ \
# # 100 \
# # gsfm_metric_1merge_subseq_500_v10_0.7_igraph \
# # mylog63_0_500/MetricResults_0_500.txt





# # mkdir -p /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/gsfm_ncut_subseq5_alledges/seq_results

# # bash /home/gy/DAGSfM_merge_v3/scripts/shell/distributed_sfm_subcluster.sh \
# # /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/ \
# # 100 \
# # gsfm_ncut_subseq5_alledges \
# # mylog63_0.35_0/MetricResults_0.35_0.txt

# # bash /home/gy/DAGSfM/scripts/shell/colmap_cluster_imglists_reconstruction.sh \
# # /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/ \
# # gsfm_ncut_subseq5_alledges \
# # 47


# # bash /home/gy/DAGSfM/scripts/shell/colmap_mapper_manylist.sh \
# # /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/ \
# # /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/gsfm_ncut_subseq4_alledges/cluster_images_0.txt \
# # /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/gsfm_ncut_subseq4_alledges/ColmapOutput_cluster0

# # bash /home/gy/DAGSfM/scripts/shell/colmap_mapper_manylist.sh \
# # /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/ \
# # /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/gsfm_ncut_subseq4_alledges/cluster_images_1.txt \
# # /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/gsfm_ncut_subseq4_alledges/ColmapOutput_cluster1

# # bash /home/gy/DAGSfM/scripts/shell/colmap_mapper_manylist.sh \
# # /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/ \
# # /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/gsfm_ncut_subseq4_alledges/cluster_images_2.txt \
# # /16t/gy/Data/SFM/IndianaUni/Quad6k/brute/gsfm_ncut_subseq4_alledges/ColmapOutput_cluster2



# mkdir -p /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/raw/gsfm_ncut_subseq2/seq_results

# bash /home/gy/DAGSfM_merge_v3/scripts/shell/distributed_sfm_subcluster.sh \
# /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/raw/ \
# 100 \
# gsfm_ncut_subseq2 \
# mylog63_0_500/MetricResults_0_500.txt



# mkdir -p /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/FrameRetrives/fps5/gsfm_community_seq1/seq_results

# bash /home/gy/DAGSfM_sequential_merging/scripts/shell/distributed_sfm_community.sh \
# /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/FrameRetrives/fps5/ \
# 100 \
# gsfm_community_seq1


# mkdir -p /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/FrameRetrives/fps5/gsfm_ncut_seq/seq_results

# bash /home/gy/DAGSfM_sequential_merging/scripts/shell/distributed_sfm_NCUT.sh \
# /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/FrameRetrives/fps5/ \
# 100 \
# gsfm_ncut_seq


# # bash /home/gy/DAGSfM-dev/scripts/shell/distributed_sfm_spec.sh \
# # /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/FrameRetrives/fps5/ \
# # 100 \
# # gsfm_spec


# # bash /home/gy/DAGSfM-dev/scripts/shell/distributed_sfm_hybrid.sh \
# # /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/FrameRetrives/fps5/ \
# # 100 \
# # gsfm_hybrid

# mkdir -p /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/FrameRetrives/fps5/gsfm_community_seq/seq_results

# bash /home/gy/DAGSfM_sequential_merging/scripts/shell/distributed_sfm_community.sh \
# /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/FrameRetrives/fps5/ \
# 100 \
# gsfm_community_seq