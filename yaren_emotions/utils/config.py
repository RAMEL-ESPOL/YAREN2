class Config:
    IMAGE_SIZE = 48
    FINAL_CONV_LAYER = -2
    NUM_CLASSES = 7
    LEARNING_RATE = 0.0001
    BATCH_SIZE = 32
    N_EPOCHS = 25
    
    TEST_SIZE = 0.1
    RANDOM_STATE = 42
    
    DATA_PATH = 'archive/fer2013/fer2013/fer2013.csv'
    MODEL_YAML_PATH = 'model_mbn.yaml'
    MODEL_H5_PATH = 'model_mbn_1.h5'
    
    EMOTION_LABELS = {
        0: 'anger',
        1: 'disgust', 
        2: 'fear',
        3: 'happiness',
        4: 'sadness',
        5: 'surprise',
        6: 'neutral'
    }
    
    DATA_AUG_CONFIG = {
        'rotation_range': 15,
        'width_shift_range': 0.15,
        'height_shift_range': 0.15,
        'shear_range': 0.15,
        'zoom_range': 0.15,
        'horizontal_flip': True
    }
    
    EARLY_STOPPING_CONFIG = {
        'monitor': 'val_accuracy',
        'min_delta': 0.00005,
        'patience': 11,
        'verbose': 1,
        'restore_best_weights': True
    }
    
    LR_SCHEDULER_CONFIG = {
        'monitor': 'val_accuracy',
        'factor': 0.3,
        'patience': 7,
        'min_lr': 1e-7,
        'verbose': 1
    }
