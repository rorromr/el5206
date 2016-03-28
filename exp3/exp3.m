vid_bg = db2video('db/fondo/gray/');
[m,s]=backgroud_model(vid_bg);
seq = db2video('db/seq1/gray/');
motion_detector_blob(seq, m,s,50);