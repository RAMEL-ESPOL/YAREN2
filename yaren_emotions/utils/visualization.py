import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import classification_report, confusion_matrix

from config import Config

class Visualizer:
    def __init__(self):
        self.config = Config()
    
    def plot_sample_images(self, df, samples_per_emotion=7):
        print("Generating sample images for each emotion...")
        
        fig = plt.figure(figsize=(12, 10))
        
        k = 0
        for label in sorted(df.emotion.unique()):
            emotion_data = df[df.emotion == label]
            
            for j in range(min(samples_per_emotion, len(emotion_data))):
                if k < len(emotion_data):
                    px = emotion_data.pixels.iloc[k]
                    px = np.array(px.split(' ')).reshape(48, 48).astype('float32')
                    
                    k += 1
                    ax = plt.subplot(7, samples_per_emotion, k)
                    ax.imshow(px, cmap='gray')
                    ax.set_xticks([])
                    ax.set_yticks([])
                    ax.set_title(self.config.EMOTION_LABELS[label])
                    
        plt.tight_layout()
        plt.show()
    
    def plot_emotion_distribution(self, df):
        plt.figure(figsize=(10, 6))
        
        emotion_counts = df.emotion.value_counts().sort_index()
        emotion_names = [self.config.EMOTION_LABELS[i] for i in emotion_counts.index]
        
        bars = plt.bar(emotion_names, emotion_counts.values)
        plt.title('Distribution of Emotions in Dataset')
        plt.xlabel('Emotions')
        plt.ylabel('Number of Images')
        plt.xticks(rotation=45)
        
        for bar, count in zip(bars, emotion_counts.values):
            plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 50,
                    str(count), ha='center', va='bottom')
        
        plt.tight_layout()
        plt.show()
    
    def plot_training_history(self, history):
        if history is None:
            print("The training history is None. Cannot plot.")
            return
            
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))
        
        ax1.plot(history.history['loss'], label='Training Loss')
        ax1.plot(history.history['val_loss'], label='Validation Loss')
        ax1.set_title('Model Loss')
        ax1.set_ylabel('Loss')
        ax1.set_xlabel('Epoch')
        ax1.legend()
        ax1.grid(True)
        
        ax2.plot(history.history['accuracy'], label='Training Accuracy')
        ax2.plot(history.history['val_accuracy'], label='Validation Accuracy')
        ax2.set_title('Model Accuracy')
        ax2.set_ylabel('Accuracy')
        ax2.set_xlabel('Epoch')
        ax2.legend()
        ax2.grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def plot_confusion_matrix(self, y_true, y_pred, normalize=False):
        print("Generating confusion matrix...")
        
        if len(y_true.shape) > 1:
            y_true = np.argmax(y_true, axis=1)
        if len(y_pred.shape) > 1:
            y_pred = np.argmax(y_pred, axis=1)
            
        cm = confusion_matrix(y_true, y_pred)
        
        if normalize:
            cm = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]
            title = 'Normalized Confusion Matrix'
            fmt = '.2f'
        else:
            title = 'Confusion Matrix'
            fmt = 'd'
        
        plt.figure(figsize=(10, 8))
        sns.heatmap(cm, 
                   annot=True, 
                   fmt=fmt, 
                   cmap='Blues',
                   xticklabels=list(self.config.EMOTION_LABELS.values()),
                   yticklabels=list(self.config.EMOTION_LABELS.values()))
        
        plt.title(title)
        plt.xlabel('Predicted Labels')
        plt.ylabel('True Labels')
        plt.tight_layout()
        plt.show()
        
        return cm
    
    def generate_classification_report(self, y_true, y_pred):
        print("Generating classification report...")
        
        if len(y_true.shape) > 1:
            y_true = np.argmax(y_true, axis=1)
        if len(y_pred.shape) > 1:
            y_pred = np.argmax(y_pred, axis=1)
        
        wrong_predictions = np.sum(y_true != y_pred)
        total_predictions = len(y_true)
        accuracy = (total_predictions - wrong_predictions) / total_predictions
        
        print(f'Total wrong predictions: {wrong_predictions}/{total_predictions}')
        print(f'Accuracy: {accuracy:.4f}\n')
        
        target_names = list(self.config.EMOTION_LABELS.values())
        report = classification_report(y_true, y_pred, target_names=target_names)
        
        return report
    
    def plot_prediction_examples(self, X, y_true, y_pred, num_examples=12):
        print("Generating prediction examples...")

        if len(y_true.shape) > 1:
            y_true = np.argmax(y_true, axis=1)
        if len(y_pred.shape) > 1:
            y_pred = np.argmax(y_pred, axis=1)
            
        correct_mask = y_true == y_pred
        incorrect_mask = ~correct_mask
        
        correct_indices = np.where(correct_mask)[0]
        incorrect_indices = np.where(incorrect_mask)[0]
        
        fig, axes = plt.subplots(3, 4, figsize=(16, 12))
        axes = axes.ravel()
        
        for i in range(min(6, len(correct_indices))):
            idx = correct_indices[i]
            img = X[idx]
            
            if img.shape[-1] == 3:
                img_gray = np.dot(img[...,:3], [0.2989, 0.5870, 0.1140])
            else:
                img_gray = img.squeeze()
                
            axes[i].imshow(img_gray, cmap='gray')
            axes[i].set_title(f'True: {self.config.EMOTION_LABELS[y_true[idx]]}\n'
                            f'Pred: {self.config.EMOTION_LABELS[y_pred[idx]]}', 
                            color='green')
            axes[i].axis('off')
        
        for i in range(min(6, len(incorrect_indices))):
            idx = incorrect_indices[i]
            img = X[idx]
            
            if img.shape[-1] == 3:
                img_gray = np.dot(img[...,:3], [0.2989, 0.5870, 0.1140])
            else:
                img_gray = img.squeeze()
                
            axes[i + 6].imshow(img_gray, cmap='gray')
            axes[i + 6].set_title(f'✗ True: {self.config.EMOTION_LABELS[y_true[idx]]}\n'
                                f'Pred: {self.config.EMOTION_LABELS[y_pred[idx]]}', 
                                color='red')
            axes[i + 6].axis('off')
        
        plt.suptitle('Examples of Predictions: Correct (top) vs Incorrect (bottom)', fontsize=16)
        plt.tight_layout()
        plt.show()