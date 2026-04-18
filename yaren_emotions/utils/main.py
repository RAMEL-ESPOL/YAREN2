import os
from data_processor import DataProcessor
from emotion_model import EmotionModel
from visualization import Visualizer
from config import Config
from tf_keras.models import load_model

def main():    
    config = Config()
    data_processor = DataProcessor()
    model = EmotionModel()
    visualizer = Visualizer()
    
    try:
        # 1. Load and process data
        X_train, X_valid, y_train, y_valid, train_datagen = data_processor.process_all_data()
        
        # 2. Create and compile model
        model.create_model()
        model.compile_model()
        model.get_model_summary()
        
        # 3. Train model
        history = model.train(X_train, y_train, X_valid, y_valid, train_datagen)
        
        # 4. Save model
        model.save_model()
        
        # 5. Evaluate model
        eval_results = model.evaluate(X_valid, y_valid)
        
        # 6. Generate visualizations
        visualizer.plot_training_history(history)
        
        # 7. Predict and visualize results
        y_pred = model.predict(X_valid)
        
        # 8. Plot confusion matrix
        visualizer.plot_confusion_matrix(y_valid, y_pred)
        
        # 9. Classification report
        visualizer.generate_classification_report(y_valid, y_pred)
        
        # 10. Plot prediction emotion examples
        visualizer.plot_prediction_examples(X_valid, y_valid, y_pred)
        
        print(f"Validation accuracy: {eval_results['accuracy']:.4f}")
        print(f"Validation loss: {eval_results['loss']:.4f}")
        if eval_results['top_3_accuracy']:
            print(f"Top-3 Accuracy: {eval_results['top_3_accuracy']:.4f}")

        print(f"Model saved to {config.MODEL_H5_PATH} and architecture saved to {config.MODEL_YAML_PATH}\n")

        print("Training and evaluation completed successfully!\n")

    except Exception as e:
        print(f"ERROR during training: {e}")
        raise

def analyze_data_only():    
    data_processor = DataProcessor()
    visualizer = Visualizer()
    
    df = data_processor.load_data()

    visualizer.plot_emotion_distribution(df)
    visualizer.plot_sample_images(df)
    
    print("Analysis completed successfully!\n")

def load_and_evaluate_model(model_path, X_test, y_test):
    print(f"Loading model from {model_path} and evaluating...")
    model = load_model(model_path)
    
    results = model.evaluate(X_test, y_test, verbose=0)
    print(f"Accuracy: {results[1]:.4f}")
    print(f"Loss: {results[0]:.4f}")
    
    return model

if __name__ == "__main__":    
    main()