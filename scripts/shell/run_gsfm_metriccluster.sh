# mkdir -p /16t/gy/Data/SFM/hkust/temple/brute/gsfm_ncut_seq/seq_results

# bash /home/gy/DAGSfM_sequential_merging/scripts/shell/distributed_sfm_NCUT.sh \
# /16t/gy/Data/SFM/hkust/temple/brute/ \
# 100 \
# gsfm_ncut_seq

mkdir -p /16t/gy/Data1/SFM/tankandtemples/advanced/Palace/FrameRetrives/fps5/gsfm_ncut_metricseq/seq_results

bash /home/gy/DAGSfM_merge_v3/scripts/shell/distributed_sfm_mycluster.sh \
/16t/gy/Data1/SFM/tankandtemples/advanced/Palace/FrameRetrives/fps5/ \
100 \
gsfm_ncut_metricseq \
mylog63_0_1157/MetricResults_0_1157.txt

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