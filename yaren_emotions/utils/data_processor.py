import numpy as np
import pandas as pd
import cv2
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
import tf_keras.utils as np_utils
from tf_keras.preprocessing.image import ImageDataGenerator
from config import Config

class DataProcessor:
    def __init__(self):
        self.config = Config()
        self.label_encoder = LabelEncoder()
        self.train_datagen = None
        
    def load_data(self, data_path=None):
        if data_path is None:
            data_path = self.config.DATA_PATH
            
        print(f"Data loading from {data_path}...")
        df = pd.read_csv(data_path)
        
        print(f'There are {len(df)} images in the dataset, and the number of columns is {df.shape[1]}.')
        print(f'Unique emotions: {df.emotion.nunique()}')
        print(f'Number of images per emotion: \n{df.emotion.value_counts()}')
        
        return df
    
    def pixels_to_images(self, pixel_data):
        print("Converting pixel data to images...")
        
        # Convert pixels to numpy arrays and reshape to 48x48
        img_array = pixel_data.apply(
            lambda x: np.array(x.split(' ')).reshape(48, 48).astype('float32')
        )
        img_array = np.stack(img_array, axis=0)
        
        img_features = []
        for i in range(len(img_array)):
            temp = cv2.cvtColor(img_array[i], cv2.COLOR_GRAY2RGB)
            img_features.append(temp)
            
        img_features = np.array(img_features)
        
        return img_features
    
    def encode_labels(self, emotions):
        print("Encoding labels...")
        
        img_labels = self.label_encoder.fit_transform(emotions)
        img_labels = np_utils.to_categorical(img_labels)
        
        le_name_mapping = dict(zip(
            self.label_encoder.classes_, 
            self.label_encoder.transform(self.label_encoder.classes_)
        ))
        print(f"Mapping of emotions: {le_name_mapping}")
        
        return img_labels
    
    def split_data(self, X, y):
        print("Splitting data into training and validation sets...")
        
        X_train, X_valid, y_train, y_valid = train_test_split(
            X, y,
            shuffle=True,
            stratify=y,
            test_size=self.config.TEST_SIZE,
            random_state=self.config.RANDOM_STATE
        )
        
        print(f"Shapes - X_train: {X_train.shape}, X_valid: {X_valid.shape}")
        print(f"Shapes - y_train: {y_train.shape}, y_valid: {y_valid.shape}")
        
        return X_train, X_valid, y_train, y_valid
    
    def normalize_data(self, X_train, X_valid):
        print("Normalizing data...")
        X_train = X_train / 255.0
        X_valid = X_valid / 255.0
        return X_train, X_valid
    
    def setup_data_augmentation(self, X_train):
        print("Setting up data augmentation...")
        
        self.train_datagen = ImageDataGenerator(**self.config.DATA_AUG_CONFIG)
        self.train_datagen.fit(X_train)
        
        return self.train_datagen
    
    def process_all_data(self, data_path=None):
        # 1. Load data
        df = self.load_data(data_path)
        
        # 2. Convert pixel data to images
        img_features = self.pixels_to_images(df.pixels)
        
        # 3. Encode labels
        img_labels = self.encode_labels(df.emotion)
        
        # 4. Split data into training and validation sets
        X_train, X_valid, y_train, y_valid = self.split_data(img_features, img_labels)
        
        # 5. Normalize data
        X_train, X_valid = self.normalize_data(X_train, X_valid)
        
        # 6. Setup data augmentation
        train_datagen = self.setup_data_augmentation(X_train)
        
        del df, img_features, img_labels
        
        return X_train, X_valid, y_train, y_valid, train_datagen
    
    def get_emotion_name(self, emotion_id):
        # Returns the name of the emotion based on its ID
        return self.config.EMOTION_LABELS.get(emotion_id, "unknown")