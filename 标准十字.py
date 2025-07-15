
def cross_detection():
    """十字路口检测 - 移植参考代码的检测策略"""
    global cross_flag, cross_encoder, encoder_integral
    global pre_cross_flag,cross_count
    global CCD1_left_flag, CCD1_right_flag, CCD2_left_flag, CCD2_right_flag
    global ring_state, ring_left, ring_right
    global cross_delay_encoder, black_write_2, TARGET_SPEED, original_target_speed
    
    if ((not cross_flag) and (not pre_cross_flag)):
        if((not black_write_2) and (not CCD2_left_flag) and (not CCD2_right_flag) 
           and (Trk.right_sideline1-Trk.left_sideline1)<(Trk.right_sideline2-Trk.left_sideline2)):
            cross_encoder = encoder_integral  # 记录预十字检测时的编码器值
            pre_cross_flag = True
            cross_flag = False
        # 检查近端CCD宽度是否大于100
    elif ((not cross_flag) and pre_cross_flag):
        if (abs(cross_encoder-encoder_integral)<15
            and (not CCD1_left_flag) and (not CCD1_right_flag)):
            cross_encoder = encoder_integral  # 记录预十字检测时的编码器值 
            pre_cross_flag = False
            cross_flag = True
            set_beep_short()
             # 十字路口提速到75
            #  TARGET_SPEED = 75
             
        elif (abs(cross_encoder-encoder_integral)>=15 or black_write_2):
            pre_cross_flag = False
            cross_flag = False
            # 恢复原速度
            # TARGET_SPEED = original_target_speed
    elif (cross_flag and (not pre_cross_flag)):
        if ((CCD1_left_flag and CCD1_right_flag) or abs(cross_encoder-encoder_integral)>=15):
            pre_cross_flag = False
            cross_flag = False
            # 恢复原速度
            # TARGET_SPEED = original_target_speed