import math
from tf_keras.optimizers import Adam
from tf_keras.models import Model
from tf_keras.layers import Dense, GlobalAveragePooling2D
from tf_keras.callbacks import EarlyStopping, ReduceLROnPlateau
from tf_keras.metrics import CategoricalAccuracy, TopKCategoricalAccuracy
from tf_keras.applications import VGG19

from config import Config


class EmotionModel:
    def __init__(self):
        self.config = Config()
        self.model = None
        self.base_model = None
        self.history = None
        
    def create_base_model(self):
        print("Using VGG19 as the base model...")
        
        self.base_model = VGG19(
            weights='imagenet',
            include_top=False,
            input_shape=(
                self.config.IMAGE_SIZE,
                self.config.IMAGE_SIZE,
                3
            )
        )
        
        return self.base_model
    
    def build_head(self, base_model):
        print("Building head of the model...")
        
        # Get the output of the specified final convolution layer
        x = base_model.layers[self.config.FINAL_CONV_LAYER].output
        
        # Add global average pooling and output layer
        x = GlobalAveragePooling2D()(x)
        predictions = Dense(
            self.config.NUM_CLASSES,
            activation='softmax',
            name='out_layer'
        )(x)
        
        return predictions
    
    def create_model(self):
        print("Creating the model for emotion recognition...")
        
        # 1. Use VGG19 as the base model
        base_model = self.create_base_model()
        
        # 2. Build the head of the model
        head = self.build_head(base_model)
        
        # 3. Create the final model
        self.model = Model(inputs=base_model.input, outputs=head)
        
        print("Model created successfully!")
        print(f"Total number of parameters: {self.model.count_params():,}")
        
        return self.model
    
    def compile_model(self):
        if self.model is None:
            raise ValueError("You should create the model before compiling it")
            
        print("Compiling the model...")      

        optimizer = Adam(
            learning_rate=self.config.LEARNING_RATE,
            beta_1=0.9,
            beta_2=0.999
        )
        
        metrics = [
            CategoricalAccuracy(name='accuracy'),
            TopKCategoricalAccuracy(k=3, name='top_3_accuracy')
        ]
        
        self.model.compile(
            loss='categorical_crossentropy',
            optimizer=optimizer,
            metrics=metrics
        )
        
        print("Model compiled successfully!")
    
    def get_callbacks(self):
        print("Getting callbacks for training...")
        
        early_stopping = EarlyStopping(**self.config.EARLY_STOPPING_CONFIG)
        lr_scheduler = ReduceLROnPlateau(**self.config.LR_SCHEDULER_CONFIG)

        callbacks = [early_stopping, lr_scheduler]
        
        return callbacks
    
    def train(self, X_train, y_train, X_valid, y_valid, train_datagen):
        if self.model is None:
            raise ValueError("The model should be created and compiled before training")
            
        print("Initiating training...")
        
        steps_per_epoch = math.ceil(len(X_train) / self.config.BATCH_SIZE)
        
        callbacks = self.get_callbacks()
        
        self.history = self.model.fit(
            train_datagen.flow(X_train, y_train, batch_size=self.config.BATCH_SIZE),
            validation_data=(X_valid, y_valid),
            steps_per_epoch=steps_per_epoch,
            epochs=self.config.N_EPOCHS,
            callbacks=callbacks,
            verbose=1
        )
        
        print("Training completed!")
        return self.history
    
    def save_model(self, yaml_path=None, h5_path=None):
        if self.model is None:
            raise ValueError("No model to save. Please create and compile the model first.")
            
        if yaml_path is None:
            yaml_path = self.config.MODEL_YAML_PATH
        if h5_path is None:
            h5_path = self.config.MODEL_H5_PATH

        print("Saving the model architecture and weights...")

        model_yaml = self.model.to_json()
        with open(yaml_path, "w") as yaml_file:
            yaml_file.write(model_yaml)
            
        self.model.save(h5_path)
        
        print(f"Model saved successfully to {yaml_path} and {h5_path}")
    
    def evaluate(self, X_valid, y_valid):
        if self.model is None:
            raise ValueError("The model should be created and compiled before evaluation")
            
        print("Evaluating the model on validation data...")
        
        results = self.model.evaluate(X_valid, y_valid, verbose=0)
        
        # Get the loss, accuracy, and top-3 accuracy
        loss = results[0]
        accuracy = results[1]
        top_3_accuracy = results[2] if len(results) > 2 else None
        
        print(f"Validation Loss: {loss:.4f}")
        print(f"Validation Accuracy: {accuracy:.4f}")
        if top_3_accuracy:
            print(f"Validation Top-3 Accuracy: {top_3_accuracy:.4f}")
            
        return {
            'loss': loss,
            'accuracy': accuracy,
            'top_3_accuracy': top_3_accuracy
        }
    
    def predict(self, X):
        if self.model is None:
            raise ValueError("The model should be created and compiled before making predictions")
            
        return self.model.predict(X)
    
    def get_model_summary(self):
        if self.model is None:
            raise ValueError("The model should be created before getting the summary")
            
        return self.model.summary()